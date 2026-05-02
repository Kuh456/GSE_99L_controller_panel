use crate::{
    ANGLE_SCALE, ANGLE_STATUS_TOLERANCE_DEG, BUTTON_STATE, CLOSE_ANGLE, MAIN_STATE, OPEN_ANGLE,
    VALVE_ANGLE_X10, VALVE_STATE,
};
use core::fmt::Write;
use core::sync::atomic::Ordering;
use embassy_time::{Duration, Timer};
use esp_hal::{Async, uart::UartTx};
use esp_println::println;
use heapless::String;

#[embassy_executor::task]
pub async fn pc_display_task(mut tx: UartTx<'static, Async>) {
    let mut burned: bool = false;
    loop {
        let valve_com_state = VALVE_STATE.load(Ordering::Relaxed);
        let angle_x10 = VALVE_ANGLE_X10.load(Ordering::Relaxed);
        let main_state = MAIN_STATE.load(Ordering::Relaxed);
        let button_state = BUTTON_STATE.load(Ordering::Relaxed);
        if main_state == 1 {
            burned = true;
        }
        let mut msg: String<256> = String::new();
        let valve_state_str = match valve_com_state {
            0 => "Normal",
            1 => "Communication ERROR",
            2 => "VALVE CAN ERROR",
            3 => "VALVE-CONTROL CAN ERROR",
            _ => "Unreachable",
        };
        let main_state_str = match main_state {
            0 => "Normal",
            1 => "IGNITION",
            2 => "TIMEOUT",
            3 => "MAIN CAN ERROR",
            4 => "MAIN-CONTROL CAN ERROR",
            _ => "Unreachable",
        };
        let burned_str = if burned { "Done" } else { "Yet" };
        let open_angle_x10 = OPEN_ANGLE * ANGLE_SCALE;
        let close_angle_x10 = CLOSE_ANGLE * ANGLE_SCALE;
        let tolerance_x10 = ANGLE_STATUS_TOLERANCE_DEG * ANGLE_SCALE;
        let angle_status = if angle_x10 > close_angle_x10 - tolerance_x10
            && angle_x10 < close_angle_x10 + tolerance_x10
        {
            "Close!"
        } else if angle_x10 > open_angle_x10 - tolerance_x10
            && angle_x10 < open_angle_x10 + tolerance_x10
        {
            "Open!"
        } else {
            "Invalid"
        };
        let angle_abs_x10 = angle_x10.abs();

        if write!(
            msg,
            "burned: {}, valve_com_state: {}, main_state: {}, MainAngle: {} ({}{}.{}) \r\n\
             DUMP:{}  FIRE:{}\r\n\
             FILL:{}  SEP :{}\r\n\
             SET :{}  O2  :{}\r\n\
             VLV_OPN :{}\r\n",
            burned_str,
            valve_state_str,
            main_state_str,
            angle_status,
            if angle_x10 < 0 { "-" } else { "" },
            angle_abs_x10 / ANGLE_SCALE,
            angle_abs_x10 % ANGLE_SCALE,
            button_state & 1,
            (button_state >> 1) & 1,
            (button_state >> 2) & 1,
            (button_state >> 3) & 1,
            (button_state >> 4) & 1,
            (button_state >> 5) & 1,
            (button_state >> 6) & 1,
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
