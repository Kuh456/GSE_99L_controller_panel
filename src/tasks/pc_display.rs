use crate::{
    ANGLE_SCALE, ANGLE_STATUS_TOLERANCE_DEG, BUTTON_STATE, ButtonFlags, CAN_HEALTH,
    CAN_LOCAL_ERROR, CAN_PEER_ALIVE, CAN_TX_TIMEOUT_ACTIVE, CanHealth, CanLocalError,
    INPUT_GPIO_STATUS, INTERNAL_STATUS_FLAGS, INTERNAL_STATUS_PHASE, MAIN_VALVE_CLOSED_ANGLE_X10,
    MAIN_VALVE_OPEN_ANGLE_X10, OUTPUT_GPIO_STATUS, VALVE_ANGLE_X10,
};
use core::fmt::Write;
use core::sync::atomic::Ordering;
use embassy_time::{Duration, Timer};
use esp_hal::{Async, uart::UartTx};
use esp_println::println;
use heapless::String;

fn bit(flag: bool) -> u8 {
    flag as u8
}

fn get_main_sequence_state_str(state: u8) -> &'static str {
    match state {
        0 => "Idle",
        1 => "Firing",
        2 => "Timeout",
        3 => "Abort",
        _ => "Unknown",
    }
}

fn get_can_health_str(health: u8) -> &'static str {
    match health {
        x if x == CanHealth::Active as u8 => "Active",
        x if x == CanHealth::Warning as u8 => "Warning",
        x if x == CanHealth::Passive as u8 => "Passive",
        x if x == CanHealth::BusOff as u8 => "BusOff",
        _ => "Unknown",
    }
}

fn get_can_status_str(peer_alive: bool, local_error: u8, tx_timeout: bool) -> &'static str {
    match local_error {
        x if x == CanLocalError::BusOff as u8 => "CAN BUS-OFF",
        x if x == CanLocalError::ManagerStalled as u8 => "CAN TASK STUCK",
        _ if tx_timeout => "CAN TX TIMEOUT",
        _ if peer_alive => "CAN OK",
        _ => "CAN TIMEOUT",
    }
}

fn get_angle_status(angle_x10: i32) -> &'static str {
    let open_angle_x10 = i32::from(MAIN_VALVE_OPEN_ANGLE_X10);
    let close_angle_x10 = i32::from(MAIN_VALVE_CLOSED_ANGLE_X10);
    let tolerance_x10 = ANGLE_STATUS_TOLERANCE_DEG * ANGLE_SCALE;

    if (angle_x10 - close_angle_x10).abs() <= tolerance_x10 {
        return "Close!";
    }
    if (angle_x10 - open_angle_x10).abs() <= tolerance_x10 {
        return "Open!";
    }
    "Invalid"
}

#[embassy_executor::task]
pub async fn pc_display_task(mut tx: UartTx<'static, Async>) {
    let mut burned: bool = false;
    loop {
        let output_gpio_status = OUTPUT_GPIO_STATUS.load(Ordering::Relaxed);
        let input_gpio_status = INPUT_GPIO_STATUS.load(Ordering::Relaxed);
        let internal_status_flags = INTERNAL_STATUS_FLAGS.load(Ordering::Relaxed);
        let angle_x10 = VALVE_ANGLE_X10.load(Ordering::Relaxed);
        let main_sequence_state = INTERNAL_STATUS_PHASE.load(Ordering::Relaxed);
        let can_peer_alive = CAN_PEER_ALIVE.load(Ordering::Relaxed);
        let can_local_error = CAN_LOCAL_ERROR.load(Ordering::Relaxed);
        let can_tx_timeout = CAN_TX_TIMEOUT_ACTIVE.load(Ordering::Relaxed);
        let can_health = CAN_HEALTH.load(Ordering::Relaxed);
        let buttons = ButtonFlags::from_bits_truncate(BUTTON_STATE.load(Ordering::Relaxed));
        // Current Integrated Board firmware reports phase=2 for sequence end.
        if main_sequence_state == 2 {
            burned = true;
        }

        let mut msg: String<512> = String::new();
        let angle_status = get_angle_status(angle_x10);
        let angle_abs_x10 = angle_x10.abs();

        let format_result = write!(
            msg,
            "burned: {}, can: {} (health: {})\r\n\
             internal_flags: 0x{:02X}, output_gpio: 0x{:02X}, input_gpio: 0x{:02X}\r\n\
             main_sequence: {}\r\n\
             MainAngle: {} ({}{}.{}) \r\n\
             DUMP:{}  FIRE:{}\r\n\
             FILL:{}  SEP :{}\r\n\
             SET :{}  O2  :{}\r\n\
             VLV_OPN :{}\r\n",
            if burned { "Done" } else { "Yet" },
            get_can_status_str(can_peer_alive, can_local_error, can_tx_timeout),
            get_can_health_str(can_health),
            internal_status_flags,
            output_gpio_status,
            input_gpio_status,
            get_main_sequence_state_str(main_sequence_state),
            angle_status,
            if angle_x10 < 0 { "-" } else { "" },
            angle_abs_x10 / ANGLE_SCALE,
            angle_abs_x10 % ANGLE_SCALE,
            bit(buttons.contains(ButtonFlags::DUMP)),
            bit(buttons.contains(ButtonFlags::FIRE)),
            bit(buttons.contains(ButtonFlags::FILL)),
            bit(buttons.contains(ButtonFlags::SEPARATE)),
            bit(buttons.contains(ButtonFlags::VALVE_SET)),
            bit(buttons.contains(ButtonFlags::O2)),
            bit(buttons.contains(ButtonFlags::VALVE_OPEN)),
        );

        if format_result.is_err() {
            println!("Format error: buffer overflow");
        } else {
            // msg.as_bytes() で文字列を &[u8] に変換して送信する
            match tx.write_async(msg.as_bytes()).await {
                Ok(_n) => {
                    // println!("write success: {} bytes", n);
                }
                Err(_e) => {
                    // println!("UART1 write error: {:?}", e);
                }
            }
        }
        Timer::after(Duration::from_millis(1000)).await; // 1秒ごとにPCに状態を送信.
    }
}
