#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]
use core::sync::atomic::{AtomicU8, Ordering};
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Timer};
use embedded_can::{Frame, Id};
use esp_backtrace as _;
use esp_hal::{
    Async,
    clock::CpuClock,
    gpio::{Input, InputConfig, Level, Output, OutputConfig},
    interrupt::software::SoftwareInterruptControl,
    system::Stack,
    timer::timg::TimerGroup,
    twai::{self, BaudRate, EspTwaiFrame, StandardId, TwaiMode, filter::SingleStandardFilter},
};
use esp_println::println;
use esp_rtos::embassy::Executor;
use static_cell::StaticCell;

// --- Time Definitions ---
const MAIN_CONTROL_TIMEOUT_MS: u64 = 3000;
const SAMPLING_RATE_MS: u64 = 25;

// --- CAN ID Definitions ---
const CAN_ID_BUTTON_STATE: u16 = 0x101;
const CAN_ID_MAIN_VALVE_ANGLE: u16 = 0x102;
const CAN_ID_FROM_PLC_ACK: u16 = 0x103;

// チャタリング防止用(各bitがswのオンオフに対応)
static BUTTON_STATE: AtomicU8 = AtomicU8::new(0);

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
static CANOK: Signal<CriticalSectionRawMutex, bool> = Signal::new();

#[embassy_executor::task]
async fn can_transmit_task(mut tx: twai::TwaiTx<'static, Async>) {
    let mut data: u8 = 0;
    loop {
        let state = AtomicU8::load(&BUTTON_STATE, Ordering::Relaxed);
        data = 0;
        data |= state & 1; // dataの1bit目は機体内脱圧(dump)
        data |= (state >> 1) & 1 << 1; // dataの2bit目は点火(fire)
        data |= (state >> 2) & 1 << 2; // dataの3bit目は充填(fill)
        data |= (state >> 3) & 1 << 3; // dataの4bit目は切り離し(separate)
        data |= (state >> 4) & 1 << 4; // dataの5bit目はバルブセット(valve_set)
        let frame_to_send =
            EspTwaiFrame::new(StandardId::new(CAN_ID_BUTTON_STATE).unwrap(), &[data]).unwrap();
        let _ = tx.transmit_async(&frame_to_send).await;
        Timer::after(Duration::from_millis(50)).await;
    }
}

#[embassy_executor::task]
async fn can_receive_task(mut rx: twai::TwaiRx<'static, Async>) {
    loop {
        let frame = rx.receive_async().await;
        match frame {
            Ok(payload) => {
                match payload.id() {
                    Id::Standard(s_id) if s_id.as_raw() == CAN_ID_FROM_PLC_ACK => {
                        CANOK.signal(true); // 正常にCAN受信したらエラー解除
                    }
                    Id::Standard(s_id) if s_id.as_raw() == CAN_ID_MAIN_VALVE_ANGLE => {
                        let can_data = payload.data();
                        let angle = can_data[0];
                        if angle > 80 && angle < 120 {
                            println!("Close! angle:{:?}", angle);
                        } else if angle > 170 && angle < 210 {
                            println!("Open! angle:{:?}", angle);
                        } else {
                            println!("Invalid angle:{:?}", angle)
                        };
                    }
                    _ => {} // ignore the others
                }
            }
            Err(e) => {
                // CAN受信エラー時はLEDを点滅させる
                println!("CAN receive error: {:?}", e);
                CANOK.signal(false);
            }
        }
        Timer::after(Duration::from_millis(100)).await;
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
        if state == prev {
            AtomicU8::store(&BUTTON_STATE, prev, Ordering::Relaxed);
        }
        prev = state;
        Timer::after(Duration::from_millis(SAMPLING_RATE_MS)).await; // 25msごとに
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
    let dump = Input::new(peripherals.GPIO16, InputConfig::default()); // sch : SW1 機体外脱圧用
    let fire = Input::new(peripherals.GPIO18, InputConfig::default()); // sch : SW2 イグナイター点火用
    let fill = Input::new(peripherals.GPIO21, InputConfig::default()); // sch : SW3 充填用
    let valve_set = Input::new(peripherals.GPIO19, InputConfig::default()); // sch : SW4 バルブセット用
    let separate = Input::new(peripherals.GPIO17, InputConfig::default()); // sch : SW5 切り離し用
    let mut state_led = Output::new(peripherals.GPIO25, Level::Low, OutputConfig::default()); // sch: Logic_LED 制御基板とのCAN通信の状態表示用
    let o2 = Input::new(peripherals.GPIO34, InputConfig::default()); // sch : SW6 酸素電磁弁用(スイッチ増設用のスペア
    let can_tx = Output::new(peripherals.GPIO26, Level::Low, OutputConfig::default());
    let can_rx = Input::new(peripherals.GPIO27, InputConfig::default());

    // Main_control の状態表示用
    let mut counter: u8 = 0;
    let mut last_receive_time: Instant = Instant::now();

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
                spawner.spawn(can_receive_task(rx)).ok();
                spawner.spawn(can_transmit_task(tx)).ok();
            });
        },
    );
    spawner
        .spawn(button_update_task(dump, fire, fill, separate, valve_set))
        .ok();
    loop {
        if CANOK.wait().await {
            last_receive_time = Instant::now();
        }
        if last_receive_time.elapsed() <= Duration::from_millis(MAIN_CONTROL_TIMEOUT_MS) {
            state_led.set_high();
        } else {
            if counter % 2 == 0 {
                state_led.set_low();
            } else {
                state_led.set_high();
            }
            if counter >= 255 {
                counter = 0;
            } else {
                counter += 1;
            }
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}
