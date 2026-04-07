#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]
#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
use C99L_controller_panel::*; // 定数定義等.
use core::fmt::Write;
use core::sync::atomic::{AtomicU8, Ordering};
use display_interface_spi::SPIInterface;
use embassy_executor::Spawner;
use embassy_futures::select::{Either3, select3};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_can::{Frame, Id};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::text::{Baseline, Text};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use esp_backtrace as _;
use esp_hal::delay::Delay;
use esp_hal::spi::Mode as SpiMode;
use esp_hal::spi::master::Config as SpiConfig;
use esp_hal::spi::master::Spi;
use esp_hal::time::Rate; // For specifying SPI frequency
use esp_hal::uart::{Config as UartConfig, DataBits, Parity, StopBits, Uart, UartTx};
use esp_hal::{
    Async,
    clock::CpuClock,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    interrupt::software::SoftwareInterruptControl,
    system::Stack,
    timer::timg::TimerGroup,
    twai::{self, BaudRate, EspTwaiFrame, StandardId, TwaiMode, filter::SingleStandardFilter},
};
use esp_println::println;
use esp_rtos::embassy::Executor;
use heapless::String;
use ili9341::{DisplaySize240x320, Ili9341, Orientation};
use static_cell::StaticCell;

// Larger font
use profont::{PROFONT_18_POINT, PROFONT_24_POINT};
// チャタリング防止用(各bitがswのオンオフに対応).
static BUTTON_STATE: AtomicU8 = AtomicU8::new(0);
static VALVE_STATE: AtomicU8 = AtomicU8::new(0);
static VALVE_ANGLE: AtomicU8 = AtomicU8::new(0);
static MAIN_STATE: AtomicU8 = AtomicU8::new(0);

static MAIN_RX_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static VALVE_RX_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[embassy_executor::task]
async fn can_transmit_task(mut tx: twai::TwaiTx<'static, Async>) {
    let mut data: u8 = 0;
    loop {
        let state = BUTTON_STATE.load(Ordering::Relaxed);
        let fire = (state >> 1) & 1; // dataの2bit目は点火(fire).
        let fill = (state >> 2) & 1; // dataの3bit目は充填(fill).
        let separate = (state >> 3) & 1; // dataの4bit目は切り離し(separate).
        let o2_test = (state >> 5) & 1; // dataの6bit目は試験用のo2.
        data = 0;
        data |= state & 1; // dataの1bit目は機体内脱圧(dump).
        data |= (fire & !fill) << 1; // 充填中に点火しないように.
        data |= fill << 2;
        data |= (separate & !fill) << 3; // 充填中に切り離ししないように.
        data |= state & (1 << 4); // dataの5bit目はバルブセット(valve set).
        data |= (!fire & o2_test) << 5; // 点火中はo2_testを無効にする.
        data |= state & (1 << 6); // dataの7bit目はstate_reset.
        let regular_frame =
            EspTwaiFrame::new(StandardId::new(CAN_ID_BUTTON_STATE).unwrap(), &[data]).unwrap();
        let result1 =
            with_timeout(Duration::from_millis(50), tx.transmit_async(&regular_frame)).await;
        match result1 {
            //  指定時間内に終わらなかった (外側のResultがErr).
            Err(_timeout_error) => {}

            //  時間内に終わったが、通信エラーまたは切断が発生した (内側のResultがErr).
            Ok(Err(can_err)) => {
                println!("can_tx_err: {:?}", can_err);
            }
            //  時間内に正常に受信完了 (両方ともOk).
            Ok(Ok(())) => {}
        }

        let ack_frame =
            EspTwaiFrame::new(StandardId::new(CAN_ID_TO_VALVE_ACK).unwrap(), &[0]).unwrap();
        let result2 = with_timeout(Duration::from_millis(50), tx.transmit_async(&ack_frame)).await;
        match result2 {
            //  指定時間内に終わらなかった (外側のResultがErr).
            Err(_timeout_error) => {}

            //  時間内に終わったが、通信エラーまたは切断が発生した (内側のResultがErr).
            Ok(Err(can_err)) => {
                println!("can_tx_err: {:?}", can_err);
            }
            //  時間内に正常に受信完了 (両方ともOk).
            Ok(Ok(())) => {}
        }
        Timer::after(Duration::from_millis(50)).await;
    }
}

#[embassy_executor::task]
async fn can_receive_task(mut rx: twai::TwaiRx<'static, Async>) {
    loop {
        let result = with_timeout(Duration::from_secs(3), rx.receive_async()).await;
        match result {
            Ok(frame_result) => match frame_result {
                Ok(payload) => {
                    match payload.id() {
                        Id::Standard(s_id) if s_id.as_raw() == CAN_ID_FROM_PLC_ACK => {
                            MAIN_RX_SIGNAL.signal(());
                            MAIN_STATE.store(payload.data()[0], Ordering::Relaxed);
                        }
                        Id::Standard(s_id) if s_id.as_raw() == CAN_ID_MAIN_VALVE_STATE => {
                            let can_data = payload.data();
                            if can_data.len() > 1 {
                                let valve_state = can_data[0];
                                VALVE_STATE.store(valve_state, Ordering::Relaxed);
                            }
                        }
                        Id::Standard(s_id) if s_id.as_raw() == CAN_ID_MAIN_VALVE_ANGLE => {
                            let can_data = payload.data();
                            if can_data.len() > 1 {
                                let angle = can_data[0] - 130;
                                VALVE_RX_SIGNAL.signal(());
                                VALVE_ANGLE.store(angle, Ordering::Relaxed);
                            }
                        }
                        _ => {} // ignore the others
                    }
                }
                Err(e) => {
                    // CAN受信エラー時はLEDを点滅させる
                    println!("CAN receive error: {:?}", e);
                }
            },
            Err(_) => {
                // CAN受信エラー(timeout).
            }
        }
    }
}

/// sample the raw button states and eliminate the noise by comparing state with previous state
#[embassy_executor::task]
async fn button_update_task(
    dump: Input<'static>,
    fire: Input<'static>,
    fill: Input<'static>,
    separate: Input<'static>,
    valve_set: Input<'static>,
    o2: Input<'static>,
    main_reset: Input<'static>,
) {
    let mut state = 0;
    let mut prev = 0;
    loop {
        state = 0;
        state |= dump.is_high() as u8;
        state |= (fire.is_high() as u8) << 1;
        state |= (fill.is_high() as u8) << 2;
        state |= (separate.is_high() as u8) << 3;
        state |= (valve_set.is_high() as u8) << 4;
        state |= (o2.is_high() as u8) << 5;
        state |= (main_reset.is_high() as u8) << 6;
        if state == prev {
            BUTTON_STATE.store(prev, Ordering::Relaxed);
        }
        prev = state;
        Timer::after(Duration::from_millis(SAMPLING_RATE_MS)).await; // 25msごとに.
    }
}

#[embassy_executor::task]
async fn pc_display_task(mut tx: UartTx<'static, Async>) {
    loop {
        let valve_com_state = VALVE_STATE.load(Ordering::Relaxed);
        let angle = VALVE_ANGLE.load(Ordering::Relaxed) - 130;
        let main_state = MAIN_STATE.load(Ordering::Relaxed);
        let mut msg: String<128> = String::new();
        let valve_state_str = match valve_com_state {
            0 => "Normal",
            1 => "Communication Error",
            2 => "VALVE-MAIN CAN Error",
            3 => "VALVE-CONTROL CAN Error",
            _ => "Unreachable",
        };
        let main_state_str = match main_state {
            0 => "Normal",
            1 => "IGNITION",
            2 => "TIMEOUT",
            3 => "CONTROL-MAIN CAN Error",
            _ => "Unreachable",
        };
        let angle_status = if angle > CLOSE_ANGLE - 20 && angle < CLOSE_ANGLE + 20 {
            "Close!"
        } else if angle > OPEN_ANGLE - 20 && angle < OPEN_ANGLE + 20 {
            "Open!"
        } else {
            "Invalid"
        };

        if let Err(_) = write!(
            msg,
            "valve_com_state: {}, main_state: {}, MainAngle: {} ({}) \r\n",
            valve_state_str, main_state_str, angle_status, angle
        ) {
            println!("Format error: buffer overflow");
            continue;
        }
        // msg.as_bytes() で文字列を &[u8] に変換して送信する
        match tx.write_async(msg.as_bytes()).await {
            Ok(_n) => {
                // println!("write success: {} bytes", n);
            }
            Err(_e) => {
                // println!("UART1 write error: {:?}", e);
            }
        }
        Timer::after(Duration::from_millis(1000)).await; // 1秒ごとにPCに状態を送信.
    }
}
#[embassy_executor::task]
async fn lcd_display_task(
    mut display: Ili9341<
        SPIInterface<
            ExclusiveDevice<Spi<'static, Async>, Output<'static>, NoDelay>,
            Output<'static>,
        >,
        Output<'static>,
    >,
) {
    display.clear(Rgb565::WHITE).unwrap();
    loop {
        let text_style = MonoTextStyle::new(&PROFONT_24_POINT, Rgb565::RED);
        Text::with_baseline("impl Rust", Point::new(50, 150), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();

        let text_style = MonoTextStyle::new(&PROFONT_18_POINT, Rgb565::CSS_DIM_GRAY);

        Text::with_baseline("for ESP32", Point::new(60, 180), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
    }
}
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);
    static APP_CORE_STACK: StaticCell<Stack<8192>> = StaticCell::new();
    let app_core_stack = APP_CORE_STACK.init(Stack::new());
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    // --- pin definitions ---
    // GPIO 35,36,39 are input only pins.They can be used as switch pins.
    // GPIO34 is sw6. it is used as safety switch for emergency dump on controller panel
    let dump = Input::new(
        peripherals.GPIO16,
        InputConfig::default().with_pull(Pull::Down),
    ); // sch : SW1 機体外脱圧用.
    let fire = Input::new(
        peripherals.GPIO18,
        InputConfig::default().with_pull(Pull::Down),
    ); // sch : SW2 イグナイター点火用.
    let fill = Input::new(
        peripherals.GPIO21,
        InputConfig::default().with_pull(Pull::Down),
    ); // sch : SW3 充填用.
    let valve_set = Input::new(
        peripherals.GPIO19,
        InputConfig::default().with_pull(Pull::Down),
    ); // sch : SW4 バルブセット用.
    let separate = Input::new(
        peripherals.GPIO17,
        InputConfig::default().with_pull(Pull::Down),
    ); // sch : SW5 切り離し用.
    let main_reset = Input::new(
        peripherals.GPIO35,
        InputConfig::default().with_pull(Pull::Down),
    ); // main controlのstateを燃焼待機状態にリセットするためのスイッチ.
    let uart1_tx = Output::new(peripherals.GPIO33, Level::Low, OutputConfig::default());
    let uart1_rx = Input::new(peripherals.GPIO32, InputConfig::default());
    let mut state_led = Output::new(peripherals.GPIO25, Level::Low, OutputConfig::default()); // sch: Logic_LED 制御基板とのCAN通信の状態表示用.
    let o2 = Input::new(peripherals.GPIO34, InputConfig::default()); // sch : SW6 酸素電磁弁用(スイッチ増設用のスペア.
    let can_tx = Output::new(peripherals.GPIO26, Level::Low, OutputConfig::default());
    let can_rx = Input::new(peripherals.GPIO27, InputConfig::default());

    let uart1_config = UartConfig::default()
        .with_baudrate(115_200)
        .with_data_bits(DataBits::_8)
        .with_parity(Parity::None)
        .with_stop_bits(StopBits::_1);

    let uart1 = Uart::new(peripherals.UART1, uart1_config)
        .unwrap()
        .with_rx(uart1_rx)
        .with_tx(uart1_tx)
        .into_async();
    let (_display_rx, display_tx) = uart1.split();

    // Initialize SPI
    let spi = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(4))
            .with_mode(SpiMode::_0),
    )
    .unwrap()
    .into_async()
    //CLK
    .with_sck(peripherals.GPIO14)
    //DIN
    .with_mosi(peripherals.GPIO4);
    let cs = Output::new(peripherals.GPIO23, Level::Low, OutputConfig::default());
    let dc = Output::new(peripherals.GPIO13, Level::Low, OutputConfig::default());
    let reset = Output::new(peripherals.GPIO22, Level::Low, OutputConfig::default());
    let spi_dev = ExclusiveDevice::new_no_delay(spi, cs).unwrap();
    let interface = SPIInterface::new(spi_dev, dc);
    let mut display = Ili9341::new(
        interface,
        reset,
        &mut Delay::new(),
        Orientation::Portrait,
        DisplaySize240x320,
    )
    .unwrap();

    //  Spawn some tasks
    esp_rtos::start_second_core(
        peripherals.CPU_CTRL,
        #[cfg(target_arch = "xtensa")]
        sw_int.software_interrupt0,
        sw_int.software_interrupt1,
        app_core_stack,
        move || {
            static EXECUTOR: StaticCell<Executor> = StaticCell::new();
            let executor = EXECUTOR.init(Executor::new());
            executor.run(|spawner| {
                // CAN設定
                const TWAI_BAUDRATE: twai::BaudRate = BaudRate::B125K;
                let mut can_config = twai::TwaiConfiguration::new(
                    peripherals.TWAI0,
                    can_rx,
                    can_tx,
                    TWAI_BAUDRATE,
                    TwaiMode::Normal,
                )
                .into_async();
                // Partially filter the incoming messages to reduce overhead of receiving
                // undesired messages
                can_config.set_filter(
        const { SingleStandardFilter::new(b"0xxxxxxxxxx", b"x", [b"xxxxxxxx", b"xxxxxxxx"]) },
    );
                let can = can_config.start();
                let (rx, tx) = can.split();
                spawner
                    .spawn(can_receive_task(rx))
                    .expect("can_receive_task should spawn during setup");
                spawner
                    .spawn(can_transmit_task(tx))
                    .expect("can_transmit_task should spawn during setup");
                spawner
                    .spawn(button_update_task(
                        dump, fire, fill, separate, valve_set, o2, main_reset,
                    ))
                    .expect("button_update_task should spawn during setup");
            });
        },
    );
    spawner
        .spawn(lcd_display_task(display))
        .expect("lcd_display_task should spawn during setup");
    spawner
        .spawn(pc_display_task(display_tx))
        .expect("pc_display_task should spawn during setup");
    let timeout_duration = Duration::from_millis(COMMUNICATION_TIMEOUT_MS);
    let mut main_deadline = Instant::now() + timeout_duration;
    let mut valve_deadline = Instant::now() + timeout_duration;
    loop {
        let next_timeout = main_deadline.min(valve_deadline);
        match select3(
            MAIN_RX_SIGNAL.wait(),
            VALVE_RX_SIGNAL.wait(),
            Timer::at(next_timeout),
        )
        .await
        {
            Either3::First(_) => {
                main_deadline = Instant::now() + timeout_duration;
                state_led.set_high();
            }
            Either3::Second(_) => {
                valve_deadline = Instant::now() + timeout_duration;
                state_led.set_high();
            }
            Either3::Third(_) => {
                // タイムアウト
                VALVE_STATE.store(4, Ordering::Relaxed);
                // どちらか一方でもタイムアウトしている -> 点滅
                state_led.toggle();
            }
        }
    }
}
