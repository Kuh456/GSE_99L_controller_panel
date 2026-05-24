use core::sync::atomic::Ordering;
use embassy_futures::select::{Either3, select3};
use embassy_time::{Duration, Ticker};
use embedded_can::{Frame, Id};
use esp_hal::{
    Async,
    twai::{self, ErrorKind as TwaiErrorKind, EspTwaiError, EspTwaiFrame, StandardId},
};

use crate::{
    ANGLE_CAN_SIZE, BUTTON_STATE, ButtonFlags, CAN_HEALTH, CAN_ID_BUTTON_STATE, CAN_ID_MAIN_STATE,
    CAN_ID_MAIN_VALVE_ANGLE, CAN_ID_MAIN_VALVE_STATE, CAN_MANAGER_HEARTBEAT, CAN_REC,
    CAN_RX_ERROR_COUNT, CAN_TEC, CAN_TX_ERROR_COUNT, CanHealth, MAIN_RX_SIGNAL, MAIN_STATE,
    VALVE_ANGLE_X10, VALVE_RX_SIGNAL, VALVE_STATE,
};

const BUTTON_TX_INTERVAL_MS: u64 = 50;
const CAN_HEALTH_MONITOR_INTERVAL_MS: u64 = 10;
const CAN_ERROR_WARNING_THRESHOLD: u8 = 96;
const CAN_ERROR_PASSIVE_THRESHOLD: u8 = 128;

#[embassy_executor::task]
pub async fn can_manager_task(mut can: twai::Twai<'static, Async>) {
    let mut button_tx_ticker = Ticker::every(Duration::from_millis(BUTTON_TX_INTERVAL_MS));
    let mut health_ticker = Ticker::every(Duration::from_millis(CAN_HEALTH_MONITOR_INTERVAL_MS));

    update_can_health(&can);

    loop {
        CAN_MANAGER_HEARTBEAT.fetch_add(1, Ordering::Relaxed);

        match select3(
            can.receive_async(),
            button_tx_ticker.next(),
            health_ticker.next(),
        )
        .await
        {
            Either3::First(receive_result) => match receive_result {
                Ok(frame) => {
                    handle_received_frame(&frame);
                    update_can_health(&can);
                }
                Err(error) => {
                    record_rx_error(error, &can);
                    update_can_health(&can);
                }
            },
            Either3::Second(()) => {
                if update_can_health(&can) == CanHealth::BusOff {
                    can.clear_receive_fifo();
                    continue;
                }

                let cmd = build_button_payload(BUTTON_STATE.load(Ordering::Relaxed));
                if let Some(frame) = create_can_frame_to_send(CAN_ID_BUTTON_STATE, cmd) {
                    match can.transmit_async(&frame).await {
                        Ok(()) => {
                            update_can_health(&can);
                        }
                        Err(error) => {
                            record_tx_error(error, &can);
                            update_can_health(&can);
                        }
                    }
                }
            }
            Either3::Third(()) => {
                if update_can_health(&can) == CanHealth::BusOff {
                    can.clear_receive_fifo();
                }
            }
        }
    }
}

fn build_button_payload(raw_state: u8) -> u8 {
    let state = ButtonFlags::from_bits_truncate(raw_state);
    let fire = state.contains(ButtonFlags::FIRE);
    let fill = state.contains(ButtonFlags::FILL);
    let separate = state.contains(ButtonFlags::SEPARATE);
    let o2 = state.contains(ButtonFlags::O2);

    let mut data = ButtonFlags::empty();
    if state.contains(ButtonFlags::DUMP) {
        data.insert(ButtonFlags::DUMP);
    }
    if fire && !fill {
        data.insert(ButtonFlags::FIRE);
    }
    if fill {
        data.insert(ButtonFlags::FILL);
    }
    if separate && !fill {
        data.insert(ButtonFlags::SEPARATE);
    }
    if state.contains(ButtonFlags::VALVE_SET) {
        data.insert(ButtonFlags::VALVE_SET);
    }
    if !fire && o2 {
        data.insert(ButtonFlags::O2);
    }
    if state.contains(ButtonFlags::VALVE_OPEN) {
        data.insert(ButtonFlags::VALVE_OPEN);
    }

    data.bits()
}

fn create_can_frame_to_send(can_id: u16, cmd: u8) -> Option<EspTwaiFrame> {
    let id = StandardId::new(can_id)?;
    EspTwaiFrame::new(id, &[cmd])
}

fn handle_received_frame(frame: &EspTwaiFrame) {
    match frame.id() {
        Id::Standard(s_id) if s_id.as_raw() == CAN_ID_MAIN_STATE => {
            let can_data = frame.data();
            if !can_data.is_empty() {
                MAIN_STATE.store(can_data[0], Ordering::Relaxed);
                MAIN_RX_SIGNAL.signal(());
            }
        }
        Id::Standard(s_id) if s_id.as_raw() == CAN_ID_MAIN_VALVE_STATE => {
            let can_data = frame.data();
            if !can_data.is_empty() {
                VALVE_STATE.store(can_data[0], Ordering::Relaxed);
                VALVE_RX_SIGNAL.signal(());
            }
        }
        Id::Standard(s_id) if s_id.as_raw() == CAN_ID_MAIN_VALVE_ANGLE => {
            let can_data = frame.data();
            if can_data.len() >= ANGLE_CAN_SIZE {
                let angle_x10 = i16::from_le_bytes([can_data[0], can_data[1]]) as i32;
                VALVE_ANGLE_X10.store(angle_x10, Ordering::Relaxed);
                VALVE_RX_SIGNAL.signal(());
            }
        }
        _ => {}
    }
}

fn update_can_health(can: &twai::Twai<'static, Async>) -> CanHealth {
    let tec = can.transmit_error_count();
    let rec = can.receive_error_count();
    CAN_TEC.store(tec, Ordering::Relaxed);
    CAN_REC.store(rec, Ordering::Relaxed);

    let health = if can.is_bus_off() {
        CanHealth::BusOff
    } else {
        let error_count = tec.max(rec);
        if error_count >= CAN_ERROR_PASSIVE_THRESHOLD {
            CanHealth::Passive
        } else if error_count >= CAN_ERROR_WARNING_THRESHOLD {
            CanHealth::Warning
        } else {
            CanHealth::Active
        }
    };
    CAN_HEALTH.store(health as u8, Ordering::Relaxed);
    health
}

fn record_tx_error(error: EspTwaiError, can: &twai::Twai<'static, Async>) {
    CAN_TX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
    handle_can_error(error, can);
}

fn record_rx_error(error: EspTwaiError, can: &twai::Twai<'static, Async>) {
    CAN_RX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
    handle_can_error(error, can);
}

fn handle_can_error(error: EspTwaiError, can: &twai::Twai<'static, Async>) {
    match error {
        EspTwaiError::BusOff => {
            CAN_HEALTH.store(CanHealth::BusOff as u8, Ordering::Relaxed);
            can.clear_receive_fifo();
            // TODO: Add an explicit stop/start recovery path after the reset policy is defined.
        }
        EspTwaiError::EmbeddedHAL(TwaiErrorKind::Overrun) => {
            can.clear_receive_fifo();
        }
        EspTwaiError::EmbeddedHAL(_) | EspTwaiError::NonCompliantDlc(_) => {}
    }
}
