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
use c99l_controller_panel::{
    tasks::{button_update::*, can_communication::*, lcd_display::*, pc_display::*}, // 各タスクをインポート
    *, // 定数をインポート
};
use core::sync::atomic::Ordering;
use display_interface_spi::SPIInterface;
use embassy_executor::Spawner;
use embassy_futures::select::{Either3, select3};
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_backtrace as _;
use esp_hal::delay::Delay;
use esp_hal::spi::Mode as SpiMode;
use esp_hal::spi::master::Config as SpiConfig;
use esp_hal::spi::master::Spi;
use esp_hal::time::Rate; // For specifying SPI frequency
use esp_hal::uart::{Config as UartConfig, DataBits, Parity, StopBits, Uart};
use esp_hal::{
    clock::CpuClock,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    interrupt::software::SoftwareInterruptControl,
    system::Stack,
    timer::timg::TimerGroup,
    twai::{self, BaudRate, TwaiMode, filter::SingleStandardFilter},
};
use esp_rtos::embassy::Executor;
use ili9341::{DisplaySize240x320, Ili9341, Orientation};
use static_cell::StaticCell;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

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
    let display = Ili9341::new(
        interface,
        reset,
        &mut Delay::new(),
        Orientation::Portrait,
        DisplaySize240x320,
    )
    .unwrap();

    //  Spawn some tasks
    spawner
        .spawn(button_update_task(
            dump, fire, fill, separate, valve_set, o2, main_reset,
        ))
        .expect("button_update_task should spawn during setup");
    // spawner
    //     .spawn(lcd_display_task(display))
    //     .expect("lcd_display_task should spawn during setup");
    // spawner
    //     .spawn(pc_display_task(display_tx))
    //     .expect("pc_display_task should spawn during setup");

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
            });
        },
    );

    esp_println::println!("setup done");
    let timeout_duration = Duration::from_millis(COMMUNICATION_TIMEOUT_MS);
    let error_timeout_duration = Duration::from_millis(COMMUNICATION_TIMEOUT_MS);
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
                esp_println::println!("get main");
                main_deadline = Instant::now() + timeout_duration;
                state_led.set_high();
            }
            Either3::Second(_) => {
                esp_println::println!("get valve");
                valve_deadline = Instant::now() + timeout_duration;
                state_led.set_high();
            }
            Either3::Third(_) => {
                esp_println::println!("timeout");
                // タイムアウト
                VALVE_STATE.store(4, Ordering::Relaxed);
                // どちらか一方でもタイムアウトしている -> 点滅
                state_led.toggle();
                valve_deadline = Instant::now() + error_timeout_duration;
                main_deadline = Instant::now() + error_timeout_duration;
            }
        }
    }
}
