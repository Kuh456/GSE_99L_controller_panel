use crate::{
    CAN_PEER_ALIVE, CAN_TX_TIMEOUT_ACTIVE, CanLocalError, INTERNAL_STATUS_FLAGS,
    INTERNAL_STATUS_PHASE, LOGGER_DATA_QUEUE, LoggerDataSample,
    tasks::display_common::{can_health_str, can_is_ok, display_snapshot, main_sequence_state_str},
};
use core::{fmt::Write, sync::atomic::Ordering};
use embassy_sync::channel::TryReceiveError;
use embassy_time::{Duration, Timer};
use esp_hal::{Async, uart::UartTx};
use esp_println::println;
use heapless::String;

const PC_LOGGER_DISPLAY_INTERVAL_MS: u64 = 500;
const PC_FAULT_DISPLAY_INTERVAL_MS: u64 = 2000;
const VOLTAGE_REF_5V: f32 = 4.99;
const ADC_TO_RATIO: f32 = 0.0002442;
const INTERNAL_CAN_PEER_LOST: u8 = 1 << 0;
const INTERNAL_CAN_BUS_OFF: u8 = 1 << 1;
const INTERNAL_SERVO_COMM_ERROR: u8 = 1 << 2;
const INTERNAL_CAN_TX_TIMEOUT: u8 = 1 << 6;
const INTERNAL_CAN_TX_FRAME_CREATE_FAILED: u8 = 1 << 7;
const INTERNAL_KNOWN_FAULT_FLAGS: u8 = INTERNAL_CAN_PEER_LOST
    | INTERNAL_CAN_BUS_OFF
    | INTERNAL_SERVO_COMM_ERROR
    | INTERNAL_CAN_TX_TIMEOUT
    | INTERNAL_CAN_TX_FRAME_CREATE_FAILED;

async fn write_line(tx: &mut UartTx<'static, Async>, line: &str) {
    if tx.write_async(line.as_bytes()).await.is_err() {
        println!("UART1 write error");
    }
}

fn adc_voltage(adc_val: u16) -> f32 {
    VOLTAGE_REF_5V * adc_val as f32 * ADC_TO_RATIO
}

fn thrust_500_lbf(adc_val: u16) -> f32 {
    1.2836 * adc_val as f32 - 49.93
}

fn pressure(adc_val: u16) -> f32 {
    2.0 * adc_voltage(adc_val) * 1e6
}

async fn write_sample(tx: &mut UartTx<'static, Async>, sample: LoggerDataSample) {
    let mut line: String<256> = String::new();
    let adc0_thrust = thrust_500_lbf(sample.adc0);
    let adc2_pressure = pressure(sample.adc2);
    let adc3_pressure = pressure(sample.adc3);
    let format_result = if let Some(delta_ms) = sample.delta_ms {
        write!(
            line,
            "c:{} dt:{}ms a0:{} a0t:{:.2}N a2:{} a2p:{:.0}Pa a3:{} a3p:{:.0}Pa miss:{}\r\n",
            sample.counter,
            delta_ms,
            sample.adc0,
            adc0_thrust,
            sample.adc2,
            adc2_pressure,
            sample.adc3,
            adc3_pressure,
            sample.missed_by_counter
        )
    } else {
        write!(
            line,
            "c:{} dt:- a0:{} a0t:{:.2}N a2:{} a2p:{:.0}Pa a3:{} a3p:{:.0}Pa miss:{}\r\n",
            sample.counter,
            sample.adc0,
            adc0_thrust,
            sample.adc2,
            adc2_pressure,
            sample.adc3,
            adc3_pressure,
            sample.missed_by_counter
        )
    };

    if format_result.is_err() {
        println!("Format error: buffer overflow");
        return;
    }

    write_line(tx, line.as_str()).await;
}

fn can_local_error_str(error: u8) -> &'static str {
    match error {
        x if x == CanLocalError::None as u8 => "NONE",
        x if x == CanLocalError::BusOff as u8 => "BUS_OFF",
        x if x == CanLocalError::ManagerStalled as u8 => "MANAGER_STALLED",
        _ => "UNKNOWN",
    }
}

fn push_fault_flag_name(text: &mut String<1024>, needs_separator: &mut bool, name: &str) {
    if *needs_separator {
        let _ = write!(text, ", ");
    }
    let _ = write!(text, "{}", name);
    *needs_separator = true;
}

fn write_fault_flags(text: &mut String<1024>, flags: u8) {
    let _ = write!(
        text,
        "internal_flags: raw=0x{flags:02X} fault_flags: 0x{flags:02X} ["
    );

    if flags == 0 {
        let _ = write!(text, "none]");
        return;
    }

    let mut needs_separator = false;
    if flags & INTERNAL_CAN_PEER_LOST != 0 {
        push_fault_flag_name(text, &mut needs_separator, "CAN_PEER_LOST");
    }
    if flags & INTERNAL_CAN_BUS_OFF != 0 {
        push_fault_flag_name(text, &mut needs_separator, "CAN_BUS_OFF");
    }
    if flags & INTERNAL_SERVO_COMM_ERROR != 0 {
        push_fault_flag_name(text, &mut needs_separator, "SERVO_COMM_ERROR");
    }
    if flags & INTERNAL_CAN_TX_TIMEOUT != 0 {
        push_fault_flag_name(text, &mut needs_separator, "CAN_TX_TIMEOUT");
    }
    if flags & INTERNAL_CAN_TX_FRAME_CREATE_FAILED != 0 {
        push_fault_flag_name(text, &mut needs_separator, "CAN_TX_FRAME_CREATE_FAILED");
    }

    let unknown = flags & !INTERNAL_KNOWN_FAULT_FLAGS;
    if unknown != 0 {
        if needs_separator {
            let _ = write!(text, ", ");
        }
        let _ = write!(text, "UNKNOWN=0x{:02X}", unknown);
    }

    let _ = write!(text, "]");
}

async fn write_fault_status(tx: &mut UartTx<'static, Async>) {
    let snapshot = display_snapshot();
    let can_tx_timeout = CAN_TX_TIMEOUT_ACTIVE.load(Ordering::Relaxed);
    let can_peer_alive = CAN_PEER_ALIVE.load(Ordering::Relaxed);
    let internal_flags = INTERNAL_STATUS_FLAGS.load(Ordering::Relaxed);
    let servo_comm_error = internal_flags & INTERNAL_SERVO_COMM_ERROR != 0;
    let has_fault =
        !can_peer_alive || !can_is_ok(snapshot) || can_tx_timeout || internal_flags != 0;

    if !has_fault {
        return;
    }

    let mut line: String<1024> = String::new();
    let _ = write!(
        line,
        "CAN HEALTH={} PEER={} LOCAL={}\r\n",
        can_health_str(snapshot.can_health),
        if can_peer_alive { "ALIVE" } else { "LOST" },
        can_local_error_str(snapshot.can_local_error),
    );
    write_line(tx, line.as_str()).await;

    line.clear();
    write_fault_flags(&mut line, internal_flags);
    let _ = write!(
        line,
        " PHASE={}\r\n",
        main_sequence_state_str(INTERNAL_STATUS_PHASE.load(Ordering::Relaxed)),
    );
    write_line(tx, line.as_str()).await;

    line.clear();
    let _ = write!(
        line,
        "SERVO STATUS={}\r\n",
        if servo_comm_error { "ERROR" } else { "OK" },
    );
    write_line(tx, line.as_str()).await;
}

#[embassy_executor::task]
pub async fn pc_display_task(mut tx: UartTx<'static, Async>) {
    let mut fault_elapsed_ms = PC_FAULT_DISPLAY_INTERVAL_MS;

    loop {
        if fault_elapsed_ms >= PC_FAULT_DISPLAY_INTERVAL_MS {
            write_fault_status(&mut tx).await;
            fault_elapsed_ms = 0;
        }

        loop {
            match LOGGER_DATA_QUEUE.try_receive() {
                Ok(sample) => write_sample(&mut tx, sample).await,
                Err(TryReceiveError::Empty) => break,
            }
        }

        Timer::after(Duration::from_millis(PC_LOGGER_DISPLAY_INTERVAL_MS)).await;
        fault_elapsed_ms += PC_LOGGER_DISPLAY_INTERVAL_MS;
    }
}
