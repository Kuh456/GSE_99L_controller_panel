use crate::{LOGGER_DATA_QUEUE, LoggerDataSample};
use core::fmt::Write;
use embassy_sync::channel::TryReceiveError;
use embassy_time::{Duration, Timer};
use esp_hal::{Async, uart::UartTx};
use esp_println::println;
use heapless::String;

const PC_LOGGER_DISPLAY_INTERVAL_MS: u64 = 500;

async fn write_line(tx: &mut UartTx<'static, Async>, line: &str) {
    if tx.write_async(line.as_bytes()).await.is_err() {
        println!("UART1 write error");
    }
}

async fn write_sample(tx: &mut UartTx<'static, Async>, sample: LoggerDataSample) {
    let mut line: String<128> = String::new();
    let format_result = if let Some(delta_ms) = sample.delta_ms {
        write!(
            line,
            "c:{} dt:{}ms a0:{} a2:{} a3:{} miss:{}\r\n",
            sample.counter,
            delta_ms,
            sample.adc0,
            sample.adc2,
            sample.adc3,
            sample.missed_by_counter
        )
    } else {
        write!(
            line,
            "c:{} dt:- a0:{} a2:{} a3:{} miss:{}\r\n",
            sample.counter, sample.adc0, sample.adc2, sample.adc3, sample.missed_by_counter
        )
    };

    if format_result.is_err() {
        println!("Format error: buffer overflow");
        return;
    }

    write_line(tx, line.as_str()).await;
}

#[embassy_executor::task]
pub async fn pc_display_task(mut tx: UartTx<'static, Async>) {
    loop {
        write_line(&mut tx, "LOGGER batch\r\n").await;

        loop {
            match LOGGER_DATA_QUEUE.try_receive() {
                Ok(sample) => write_sample(&mut tx, sample).await,
                Err(TryReceiveError::Empty) => break,
            }
        }

        Timer::after(Duration::from_millis(PC_LOGGER_DISPLAY_INTERVAL_MS)).await;
    }
}
