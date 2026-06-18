use crate::{
    ANGLE_SCALE, LoggerDataFreshness, latest_logger_data, logger_data_freshness,
    tasks::display_common::{
        angle_status_str, can_health_str, can_status_str, display_snapshot, main_sequence_state_str,
    },
};
use core::fmt::Write;
use embassy_time::{Duration, Instant, Timer};
use esp_hal::{Async, uart::UartTx};
use esp_println::println;
use heapless::String;

fn get_logger_data_status_str(freshness: LoggerDataFreshness) -> &'static str {
    match freshness {
        LoggerDataFreshness::NotReceived => "none",
        LoggerDataFreshness::Fresh => "fresh",
        LoggerDataFreshness::Stale => "stale",
    }
}

#[embassy_executor::task]
pub async fn pc_display_task(mut tx: UartTx<'static, Async>) {
    let mut burned: bool = false;
    loop {
        let snapshot = display_snapshot();
        let logger_data = latest_logger_data();
        let logger_data_freshness = logger_data_freshness(Instant::now());
        // Current Integrated Board firmware treats phase=2 as sequence end.
        // If a dedicated Complete phase is added later, update this condition.
        if snapshot.main_sequence_state == 2 {
            burned = true;
        }

        let mut msg: String<768> = String::new();
        let angle_status = angle_status_str(snapshot.angle_x10);
        let angle_abs_x10 = snapshot.angle_x10.abs();

        let format_result = if let Some(logger_data) = logger_data {
            write!(
                msg,
                "burned: {}, can: {} (health: {})\r\n\
                 logger: {} adc0:{} adc2:{} adc3:{} counter:{}\r\n\
                 internal_flags: 0x{:02X}, output_gpio: 0x{:02X}, input_gpio: 0x{:02X}\r\n\
                 main_sequence: {}\r\n\
                 MainAngle: {} ({}{}.{}) \r\n\
                 DUMP:{}  FIRE:{}\r\n\
                 FILL:{}  SEP :{}\r\n\
                 SET :{}  O2  :{}\r\n\
                 VLV_OPN :{}\r\n",
                if burned { "Done" } else { "Yet" },
                can_status_str(
                    snapshot.can_peer_alive,
                    snapshot.can_local_error,
                    snapshot.can_tx_timeout
                ),
                can_health_str(snapshot.can_health),
                get_logger_data_status_str(logger_data_freshness),
                logger_data.adc0,
                logger_data.adc2,
                logger_data.adc3,
                logger_data.counter,
                snapshot.internal_status_flags,
                snapshot.output_gpio_status,
                snapshot.input_gpio_status,
                main_sequence_state_str(snapshot.main_sequence_state),
                angle_status,
                if snapshot.angle_x10 < 0 { "-" } else { "" },
                angle_abs_x10 / ANGLE_SCALE,
                angle_abs_x10 % ANGLE_SCALE,
                snapshot.buttons.dump,
                snapshot.buttons.fire,
                snapshot.buttons.fill,
                snapshot.buttons.separate,
                snapshot.buttons.valve_set,
                snapshot.buttons.o2,
                snapshot.buttons.valve_open,
            )
        } else {
            write!(
                msg,
                "burned: {}, can: {} (health: {})\r\n\
                 logger: {}\r\n\
                 internal_flags: 0x{:02X}, output_gpio: 0x{:02X}, input_gpio: 0x{:02X}\r\n\
                 main_sequence: {}\r\n\
                 MainAngle: {} ({}{}.{}) \r\n\
                 DUMP:{}  FIRE:{}\r\n\
                 FILL:{}  SEP :{}\r\n\
                 SET :{}  O2  :{}\r\n\
                 VLV_OPN :{}\r\n",
                if burned { "Done" } else { "Yet" },
                can_status_str(
                    snapshot.can_peer_alive,
                    snapshot.can_local_error,
                    snapshot.can_tx_timeout
                ),
                can_health_str(snapshot.can_health),
                get_logger_data_status_str(logger_data_freshness),
                snapshot.internal_status_flags,
                snapshot.output_gpio_status,
                snapshot.input_gpio_status,
                main_sequence_state_str(snapshot.main_sequence_state),
                angle_status,
                if snapshot.angle_x10 < 0 { "-" } else { "" },
                angle_abs_x10 / ANGLE_SCALE,
                angle_abs_x10 % ANGLE_SCALE,
                snapshot.buttons.dump,
                snapshot.buttons.fire,
                snapshot.buttons.fill,
                snapshot.buttons.separate,
                snapshot.buttons.valve_set,
                snapshot.buttons.o2,
                snapshot.buttons.valve_open,
            )
        };

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
