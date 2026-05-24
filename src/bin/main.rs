#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]
use c99l_controller_panel::{
    tasks::{button_update::*, can_communication::*, pc_display::*}, // 各タスクをインポート
    *,                                                              // 定数をインポート
};

use core::sync::atomic::Ordering;
use embassy_executor::Spawner;
use embassy_futures::select::{Either3, select3};
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
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
use static_cell::StaticCell;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "main owns peripheral initialization objects before moving them into tasks"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);
    static APP_CORE_STACK: StaticCell<Stack<8192>> = StaticCell::new();
    let app_core_stack = APP_CORE_STACK.init_with(Stack::new);

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
        peripherals.GPIO34,
        InputConfig::default().with_pull(Pull::Down),
    ); // sch : SW3 充填用.
    let valve_set = Input::new(
        peripherals.GPIO17,
        InputConfig::default().with_pull(Pull::Down),
    ); // sch : SW4 バルブセット用.
    let separate = Input::new(
        peripherals.GPIO35,
        InputConfig::default().with_pull(Pull::Down),
    ); // sch : SW5 切り離し用.
    let o2 = Input::new(peripherals.GPIO39, InputConfig::default()); // sch : SW6 酸素電磁弁用
    let valve_open = Input::new(
        peripherals.GPIO36,
        InputConfig::default().with_pull(Pull::Down),
    ); // sch: SW7 main_valveをopenさせるためのスイッチ.

    let uart1_tx = Output::new(peripherals.GPIO33, Level::Low, OutputConfig::default());
    let uart1_rx = Input::new(peripherals.GPIO32, InputConfig::default());
    let mut state_led = Output::new(peripherals.GPIO25, Level::Low, OutputConfig::default()); // sch: Logic_LED 制御基板とのCAN通信の状態表示用.
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

    //  Spawn some tasks
    spawner.spawn(
        button_update_task(dump, fire, fill, separate, valve_set, o2, valve_open)
            .expect("button_update_task should spawn during setup"),
    );
    spawner.spawn(pc_display_task(display_tx).expect("pc_display_task should spawn during setup"));

    esp_rtos::start_second_core(
        peripherals.CPU_CTRL,
        sw_int.software_interrupt1,
        app_core_stack,
        move || {
            static EXECUTOR: StaticCell<Executor> = StaticCell::new();
            let executor = EXECUTOR.init(Executor::new());
            executor.run(|spawner| {
                // CAN設定
                const TWAI_BAUDRATE: twai::BaudRate = BaudRate::Custom(twai::TimingConfig {
                    baud_rate_prescaler: 40,
                    sync_jump_width: 3,
                    tseg_1: 15,
                    tseg_2: 4,
                    triple_sample: false,
                });
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
                spawner.spawn(
                    can_manager_task(can).expect("can_manager_task should spawn during setup"),
                );
            });
        },
    );

    let timeout_duration = Duration::from_millis(COMMUNICATION_TIMEOUT_MS);
    let error_blink_interval = Duration::from_millis(ERROR_COMMUNICATION_TIMEOUT_MS);

    let mut last_main_rx: Option<Instant> = None;
    let mut last_valve_rx: Option<Instant> = None;

    let mut toggle_deadline = Instant::now() + error_blink_interval;

    loop {
        let mut main_rx = false;
        let mut valve_rx = false;

        match select3(
            MAIN_RX_SIGNAL.wait(),
            VALVE_RX_SIGNAL.wait(),
            Timer::after(Duration::from_millis(100)),
        )
        .await
        {
            Either3::First(()) => {
                main_rx = true;
            }
            Either3::Second(()) => {
                valve_rx = true;
            }
            Either3::Third(()) => {
                // Wake periodically to keep timeout detection running.
            }
        }

        let now = Instant::now();
        if MAIN_RX_SIGNAL.try_take().is_some() {
            main_rx = true;
        }
        if VALVE_RX_SIGNAL.try_take().is_some() {
            valve_rx = true;
        }
        if main_rx {
            last_main_rx = Some(now);
        }
        if valve_rx {
            last_valve_rx = Some(now);
        }

        let main_alive = last_main_rx.is_some_and(|t| t.elapsed() < timeout_duration);
        let valve_alive = last_valve_rx.is_some_and(|t| t.elapsed() < timeout_duration);
        // esp_println::println!("main:{:?}, valve:{:?}", main_alive, valve_alive);
        if main_alive && valve_alive {
            // 正常時：常時点灯
            state_led.set_high();
            toggle_deadline = now + error_blink_interval;
        } else {
            // エラー時：一定周期で点滅
            if !valve_alive {
                VALVE_STATE.store(3, Ordering::Relaxed);
            }
            if !main_alive {
                MAIN_STATE.store(4, Ordering::Relaxed);
            }
            if now >= toggle_deadline {
                state_led.toggle();
                toggle_deadline = now + error_blink_interval;
            }
        }
        // esp_println::println!("main loop");
    }
}
