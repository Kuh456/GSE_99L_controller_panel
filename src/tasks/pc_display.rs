use crate::{
    ANGLE_SCALE, ANGLE_STATUS_TOLERANCE_DEG, BUTTON_STATE, ButtonFlags, CLOSE_ANGLE, MAIN_STATE,
    OPEN_ANGLE, VALVE_ANGLE_X10, VALVE_STATE,
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

fn get_valve_state_str(state: u8) -> &'static str {
    match state {
        0 => "Normal",
        1 => "Communication ERROR",
        2 => "VALVE CAN ERROR",
        3 => "VALVE-CONTROL CAN ERROR",
        _ => "Unreachable",
    }
}

fn get_main_state_str(state: u8) -> &'static str {
    match state {
        0 => "Normal",
        1 => "IGNITION",
        2 => "TIMEOUT",
        3 => "MAIN CAN ERROR",
        4 => "MAIN-CONTROL CAN ERROR",
        _ => "Unreachable",
    }
}

fn get_angle_status(angle_x10: i32) -> &'static str {
    let open_angle_x10 = OPEN_ANGLE * ANGLE_SCALE;
    let close_angle_x10 = CLOSE_ANGLE * ANGLE_SCALE;
    let tolerance_x10 = ANGLE_STATUS_TOLERANCE_DEG * ANGLE_SCALE;

    if angle_x10 > close_angle_x10 - tolerance_x10 && angle_x10 < close_angle_x10 + tolerance_x10 {
        return "Close!";
    }
    if angle_x10 > open_angle_x10 - tolerance_x10 && angle_x10 < open_angle_x10 + tolerance_x10 {
        return "Open!";
    }
    "Invalid"
}

#[embassy_executor::task]
pub async fn pc_display_task(mut tx: UartTx<'static, Async>) {
    let mut burned: bool = false;
    loop {
        let valve_com_state = VALVE_STATE.load(Ordering::Relaxed);
        let angle_x10 = VALVE_ANGLE_X10.load(Ordering::Relaxed);
        let main_state = MAIN_STATE.load(Ordering::Relaxed);
        let buttons = ButtonFlags::from_bits_truncate(BUTTON_STATE.load(Ordering::Relaxed));
        if main_state == 1 {
            burned = true;
        }

        let mut msg: String<256> = String::new();
        let angle_status = get_angle_status(angle_x10);
        let angle_abs_x10 = angle_x10.abs();

        if write!(
            msg,
            "burned: {}, valve_com_state: {}, main_state: {}, MainAngle: {} ({}{}.{}) \r\n\
             DUMP:{}  FIRE:{}\r\n\
             FILL:{}  SEP :{}\r\n\
             SET :{}  O2  :{}\r\n\
             VLV_OPN :{}\r\n",
            if burned { "Done" } else { "Yet" },
            get_valve_state_str(valve_com_state),
            get_main_state_str(main_state),
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
        )
        .is_err()
        {
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
