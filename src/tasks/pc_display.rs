use crate::{CLOSE_ANGLE, MAIN_STATE, OPEN_ANGLE, VALVE_ANGLE, VALVE_STATE};
use core::fmt::Write;
use core::sync::atomic::Ordering;
use embassy_time::{Duration, Timer};
use esp_hal::{Async, uart::UartTx};
use esp_println::println;
use heapless::String;

#[embassy_executor::task]
pub async fn pc_display_task(mut tx: UartTx<'static, Async>) {
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
