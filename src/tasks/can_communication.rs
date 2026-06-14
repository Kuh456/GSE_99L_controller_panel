use core::sync::atomic::Ordering;
use embassy_futures::select::{Either3, select3};
use embassy_time::{Duration, Ticker, Timer, with_timeout};
use embedded_can::{Frame, Id};
use esp_hal::{
    Async,
    twai::{self, ErrorKind as TwaiErrorKind, EspTwaiError, EspTwaiFrame, StandardId},
};

use crate::{
    BUTTON_STATE, ButtonFlags, CAN_HEALTH, CAN_LOCAL_ERROR, CAN_MANAGER_HEARTBEAT, CAN_REC,
    CAN_RX_ERROR_COUNT, CAN_RX_EVENT_FLAGS, CAN_RX_EVENT_INPUT_GPIO_STATUS,
    CAN_RX_EVENT_INTERNAL_STATUS, CAN_RX_EVENT_OUTPUT_GPIO_STATUS, CAN_RX_EVENT_PEER,
    CAN_RX_EVENT_VALVE_ANGLE, CAN_TEC, CAN_TX_ERROR_COUNT, CAN_TX_TIMEOUT_ACTIVE,
    CAN_TX_TIMEOUT_MS, CanHealth, CanLocalError, INPUT_GPIO_STATUS, INTERNAL_STATUS_FLAGS,
    INTERNAL_STATUS_PHASE, MAIN_RX_SIGNAL, OUTPUT_GPIO_STATUS, VALVE_ANGLE_X10, VALVE_RX_SIGNAL,
    can::protocol::{CanDecodeError, GseCanMessage},
};

const BUTTON_TX_INTERVAL_MS: u64 = 50;
const CAN_HEALTH_MONITOR_INTERVAL_MS: u64 = 100;
const CAN_ERROR_WARNING_THRESHOLD: u8 = 96;
const CAN_ERROR_PASSIVE_THRESHOLD: u8 = 128;
const BUTTON_RESET_ACK_BIT: u8 = ButtonFlags::RESET_ACK.bits();
const BUTTON_COMMAND_BITS: u8 = BUTTON_RESET_ACK_BIT - 1;
const IB_CAN_PEER_LOST: u8 = 1 << 0;
const IB_CAN_BUS_OFF: u8 = 1 << 1;
const IB_CAN_TX_TIMEOUT: u8 = 1 << 6;
const IB_RECOVERABLE_CAN_FAULTS: u8 = IB_CAN_PEER_LOST | IB_CAN_BUS_OFF | IB_CAN_TX_TIMEOUT;

#[embassy_executor::task]
pub async fn can_manager_task(mut can: twai::Twai<'static, Async>) {
    let mut button_tx_ticker = Ticker::every(Duration::from_millis(BUTTON_TX_INTERVAL_MS));
    let mut health_ticker = Ticker::every(Duration::from_millis(CAN_HEALTH_MONITOR_INTERVAL_MS));
    let mut reset_ack_pending = false;
    let mut reset_ack_gap_pending = false;

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
                    if handle_received_frame(&frame) {
                        reset_ack_pending = true;
                    }
                    update_can_health(&can);
                }
                Err(error) => {
                    record_rx_error(error, &can);
                    if update_can_health(&can) == CanHealth::BusOff {
                        // Prevent an immediately failing receive future from spinning.
                        Timer::after(Duration::from_millis(CAN_HEALTH_MONITOR_INTERVAL_MS)).await;
                    }
                }
            },
            Either3::Second(()) => {
                if update_can_health(&can) == CanHealth::BusOff {
                    continue;
                }

                let button_payload = build_button_payload(BUTTON_STATE.load(Ordering::Relaxed));
                let send_reset_ack =
                    reset_ack_pending && !reset_ack_gap_pending && button_payload == 0;
                let raw = if send_reset_ack {
                    BUTTON_RESET_ACK_BIT
                } else {
                    button_payload
                };
                let message = GseCanMessage::ButtonFromCtrlPanel { raw };
                if let Some(frame) = create_can_frame_to_send(message) {
                    match with_timeout(
                        Duration::from_millis(CAN_TX_TIMEOUT_MS),
                        can.transmit_async(&frame),
                    )
                    .await
                    {
                        Ok(Ok(())) => {
                            CAN_TX_TIMEOUT_ACTIVE.store(false, Ordering::Relaxed);
                            if send_reset_ack {
                                reset_ack_pending = false;
                                reset_ack_gap_pending = true;
                            } else if reset_ack_gap_pending {
                                reset_ack_gap_pending = false;
                            }
                            update_can_health(&can);
                        }
                        Ok(Err(error)) => {
                            record_tx_error(error, &can);
                            update_can_health(&can);
                        }
                        Err(_) => {
                            record_tx_timeout(&can);
                        }
                    }
                }
            }
            Either3::Third(()) => {
                update_can_health(&can);
            }
        }
    }
}

fn build_button_payload(raw_state: u8) -> u8 {
    let state = ButtonFlags::from_bits_truncate(raw_state & BUTTON_COMMAND_BITS);
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

fn has_recoverable_can_fault(flags: u8) -> bool {
    flags & IB_RECOVERABLE_CAN_FAULTS != 0
}

fn create_can_frame_to_send(message: GseCanMessage) -> Option<EspTwaiFrame> {
    let id = StandardId::new(message.id())?;
    let mut payload = [0; 8];
    let dlc = message.encode_payload(&mut payload);
    EspTwaiFrame::new(id, &payload[..dlc])
}

fn mark_peer_frame_received() {
    CAN_RX_EVENT_FLAGS.fetch_or(CAN_RX_EVENT_PEER, Ordering::Release);
}

fn record_invalid_peer_payload() {
    CAN_RX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
}

fn handle_received_frame(frame: &EspTwaiFrame) -> bool {
    let id = match frame.id() {
        Id::Standard(s_id) => s_id.as_raw(),
        Id::Extended(_) => return false,
    };

    match GseCanMessage::decode_standard(id, frame.data()) {
        Ok(GseCanMessage::MainValveAngleToCtrlPanel { angle_x10 }) => {
            mark_peer_frame_received();
            VALVE_ANGLE_X10.store(i32::from(angle_x10), Ordering::Relaxed);
            VALVE_RX_SIGNAL.signal(());
            CAN_RX_EVENT_FLAGS.fetch_or(CAN_RX_EVENT_VALVE_ANGLE, Ordering::Release);
            false
        }
        Ok(GseCanMessage::OutputGpioStatus { output_bits }) => {
            mark_peer_frame_received();
            OUTPUT_GPIO_STATUS.store(output_bits, Ordering::Relaxed);
            VALVE_RX_SIGNAL.signal(());
            CAN_RX_EVENT_FLAGS.fetch_or(CAN_RX_EVENT_OUTPUT_GPIO_STATUS, Ordering::Release);
            false
        }
        Ok(GseCanMessage::InputGpioStatus { input_bits }) => {
            mark_peer_frame_received();
            INPUT_GPIO_STATUS.store(input_bits, Ordering::Relaxed);
            MAIN_RX_SIGNAL.signal(());
            CAN_RX_EVENT_FLAGS.fetch_or(CAN_RX_EVENT_INPUT_GPIO_STATUS, Ordering::Release);
            false
        }
        Ok(GseCanMessage::InternalStatus { phase, flags }) => {
            mark_peer_frame_received();
            INTERNAL_STATUS_PHASE.store(phase, Ordering::Relaxed);
            INTERNAL_STATUS_FLAGS.store(flags, Ordering::Relaxed);
            MAIN_RX_SIGNAL.signal(());
            CAN_RX_EVENT_FLAGS.fetch_or(CAN_RX_EVENT_INTERNAL_STATUS, Ordering::Release);
            has_recoverable_can_fault(flags)
        }
        Ok(GseCanMessage::ButtonFromCtrlPanel { .. }) | Err(CanDecodeError::UnknownId(_)) => false,
        Err(CanDecodeError::InvalidDlc { .. }) => {
            record_invalid_peer_payload();
            false
        }
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
    if health == CanHealth::BusOff {
        CAN_LOCAL_ERROR.store(CanLocalError::BusOff as u8, Ordering::Relaxed);
    }
    health
}

fn record_tx_error(error: EspTwaiError, can: &twai::Twai<'static, Async>) {
    CAN_TX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
    handle_can_error(error, can);
}

fn record_tx_timeout(can: &twai::Twai<'static, Async>) {
    CAN_TX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
    CAN_TX_TIMEOUT_ACTIVE.store(true, Ordering::Relaxed);
    update_can_health(can);
}

fn record_rx_error(error: EspTwaiError, can: &twai::Twai<'static, Async>) {
    CAN_RX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
    handle_can_error(error, can);
}

fn handle_can_error(error: EspTwaiError, can: &twai::Twai<'static, Async>) {
    match error {
        EspTwaiError::BusOff => {
            CAN_HEALTH.store(CanHealth::BusOff as u8, Ordering::Relaxed);
            CAN_LOCAL_ERROR.store(CanLocalError::BusOff as u8, Ordering::Relaxed);
            // Clearing the receive FIFO does not recover bus-off. Recovery requires TWAI
            // stop/start, peripheral reinitialization, or a system reset.
            // TODO: Define the safety reset/recovery policy before implementing recovery.
        }
        EspTwaiError::EmbeddedHAL(TwaiErrorKind::Overrun) => {
            can.clear_receive_fifo();
        }
        EspTwaiError::EmbeddedHAL(_) | EspTwaiError::NonCompliantDlc(_) => {}
    }
}
