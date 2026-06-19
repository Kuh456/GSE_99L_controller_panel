#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]
use c99l_controller_panel::{
    tasks::{button_update::*, can_communication::*, lcd_display::*, pc_display::*}, // 各タスクをインポート
    *, // 定数をインポート
};

use core::sync::atomic::Ordering;
use display_interface_spi::SPIInterface;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    interrupt::software::SoftwareInterruptControl,
    spi::{
        Mode as SpiMode,
        master::{Config as SpiConfig, Spi},
    },
    system::Stack,
    time::Rate,
    timer::timg::TimerGroup,
    twai::{self, BaudRate, TwaiMode, filter::SingleStandardFilter},
    uart::{Config as UartConfig, DataBits, Parity, StopBits, Uart},
};
use esp_rtos::embassy::Executor;
use ili9341::{DisplaySize240x320, Ili9341, Orientation};
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
    // ESP32 GPIO34-39 are input-only and have no internal pull resistors.
    // Switches on these pins require external pull-down resistors on the board.
    let dump = Input::new(
        peripherals.GPIO16,
        InputConfig::default().with_pull(Pull::Down),
    ); // sch : SW1 機体外脱圧用.
    let fire = Input::new(
        peripherals.GPIO18,
        InputConfig::default().with_pull(Pull::Down),
    ); // sch : SW2 イグナイター点火用.
    let fill = Input::new(peripherals.GPIO34, InputConfig::default()); // sch : SW3 充填用. external pull-down required.
    let valve_set = Input::new(
        peripherals.GPIO17,
        InputConfig::default().with_pull(Pull::Down),
    ); // sch : SW4 バルブセット用.
    let separate = Input::new(peripherals.GPIO35, InputConfig::default()); // sch : SW5 切り離し用. external pull-down required.
    let o2 = Input::new(peripherals.GPIO39, InputConfig::default()); // sch : SW6 酸素電磁弁用. external pull-down required.
    let valve_open = Input::new(peripherals.GPIO36, InputConfig::default()); // sch: SW7 main_valve open switch. external pull-down required.

    let uart1_tx = Output::new(peripherals.GPIO23, Level::Low, OutputConfig::default());
    let uart1_rx = Input::new(peripherals.GPIO22, InputConfig::default());
    let mut state_led = Output::new(peripherals.GPIO13, Level::Low, OutputConfig::default()); // sch: Logic_LED 制御基板とのCAN通信の状態表示用.
    let mut solenoid_power_led =
        Output::new(peripherals.GPIO14, Level::Low, OutputConfig::default());
    let mut relay_12v_led = Output::new(peripherals.GPIO27, Level::Low, OutputConfig::default());
    let mut igniter_power_led =
        Output::new(peripherals.GPIO26, Level::Low, OutputConfig::default());
    let mut relay_24v_led = Output::new(peripherals.GPIO25, Level::Low, OutputConfig::default());

    let can_tx = Output::new(peripherals.GPIO33, Level::Low, OutputConfig::default());
    let can_rx = Input::new(peripherals.GPIO32, InputConfig::default());
    let lcd_dc = Output::new(peripherals.GPIO4, Level::Low, OutputConfig::default());
    let lcd_reset = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());
    let lcd_cs = Output::new(peripherals.GPIO15, Level::High, OutputConfig::default());

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
    let lcd_spi = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(20))
            .with_mode(SpiMode::_0),
    )
    .unwrap()
    .with_sck(peripherals.GPIO21)
    .with_mosi(peripherals.GPIO19);
    let lcd_device = ExclusiveDevice::new_no_delay(lcd_spi, lcd_cs).unwrap();
    let lcd_interface = SPIInterface::new(lcd_device, lcd_dc);
    let mut lcd_delay = Delay::new();
    let lcd_display = Ili9341::new(
        lcd_interface,
        lcd_reset,
        &mut lcd_delay,
        Orientation::LandscapeFlipped,
        DisplaySize240x320,
    )
    .unwrap();

    //  Spawn some tasks
    let button_update = button_update_task(dump, fire, fill, separate, valve_set, o2, valve_open)
        .expect("button_update_task token should be allocated during setup");
    spawner.spawn(button_update);
    let pc_display = pc_display_task(display_tx)
        .expect("pc_display_task token should be allocated during setup");
    spawner.spawn(pc_display);
    let lcd_display = lcd_display_task(lcd_display)
        .expect("lcd_display_task token should be allocated during setup");
    spawner.spawn(lcd_display);

    esp_rtos::start_second_core(
        peripherals.CPU_CTRL,
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
                can_config.set_filter(const {
                    SingleStandardFilter::new(b"0xxxxxxxxxx", b"x", [b"xxxxxxxx", b"xxxxxxxx"])
                });
                let can = can_config.start();
                let can_manager = can_manager_task(can)
                    .expect("can_manager_task token should be allocated during setup");
                spawner.spawn(can_manager);
            });
        },
    );

    let timeout_duration = Duration::from_millis(COMMUNICATION_TIMEOUT_MS);
    let can_manager_timeout = Duration::from_millis(CAN_MANAGER_HEARTBEAT_TIMEOUT_MS);
    let error_blink_interval = Duration::from_millis(ERROR_COMMUNICATION_TIMEOUT_MS);

    let mut last_can_peer_rx: Option<Instant> = None;
    let mut observed_can_manager_heartbeat = CAN_MANAGER_HEARTBEAT.load(Ordering::Relaxed);
    let mut last_can_manager_progress = Instant::now();

    let mut toggle_deadline = Instant::now() + error_blink_interval;

    loop {
        let now = Instant::now();
        let can_rx_events = CAN_RX_EVENT_FLAGS.swap(0, Ordering::Acquire);
        if can_rx_events & CAN_RX_EVENT_PEER != 0 {
            last_can_peer_rx = Some(now);
        }

        let can_manager_heartbeat = CAN_MANAGER_HEARTBEAT.load(Ordering::Relaxed);
        if can_manager_heartbeat != observed_can_manager_heartbeat {
            observed_can_manager_heartbeat = can_manager_heartbeat;
            last_can_manager_progress = now;
        }
        let can_manager_alive = now.duration_since(last_can_manager_progress) < can_manager_timeout;

        let can_bus_off = CAN_HEALTH.load(Ordering::Relaxed) == CanHealth::BusOff as u8;
        let can_tx_timeout = CAN_TX_TIMEOUT_ACTIVE.load(Ordering::Relaxed);
        let can_peer_alive =
            last_can_peer_rx.is_some_and(|last_rx| now.duration_since(last_rx) < timeout_duration);
        CAN_PEER_ALIVE.store(can_peer_alive, Ordering::Relaxed);

        let can_local_error = if can_bus_off {
            CanLocalError::BusOff
        } else if !can_manager_alive {
            CanLocalError::ManagerStalled
        } else {
            CanLocalError::None
        };
        CAN_LOCAL_ERROR.store(can_local_error as u8, Ordering::Relaxed);

        let can_status_ok =
            can_peer_alive && can_local_error == CanLocalError::None && !can_tx_timeout;

        if can_status_ok {
            // 正常時：常時点灯
            state_led.set_high();
            toggle_deadline = now + error_blink_interval;
        } else {
            // エラー時：一定周期で点滅
            if now >= toggle_deadline {
                state_led.toggle();
                toggle_deadline = now + error_blink_interval;
            }
        }

        if can_status_ok {
            let input_gpio_status = INPUT_GPIO_STATUS.load(Ordering::Relaxed);
            if input_gpio_status & IN_SOLENOID_POWER_PRESENT != 0 {
                solenoid_power_led.set_high();
            } else {
                solenoid_power_led.set_low();
            }
            if input_gpio_status & IN_RELAY_12V_ON != 0 {
                relay_12v_led.set_high();
            } else {
                relay_12v_led.set_low();
            }
            if input_gpio_status & IN_IGNITER_POWER_PRESENT != 0 {
                igniter_power_led.set_high();
            } else {
                igniter_power_led.set_low();
            }
            if input_gpio_status & IN_RELAY_24V_ON != 0 {
                relay_24v_led.set_high();
            } else {
                relay_24v_led.set_low();
            }
        } else {
            // Do nothing.Hold the Power LED state
        }

        Timer::after(Duration::from_millis(100)).await;
        // esp_println::println!("main loop");
    }
}
