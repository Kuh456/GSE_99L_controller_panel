use core::sync::atomic::Ordering;
use embassy_time::{Duration, Timer};
use embedded_can::{Frame, Id};
use esp_hal::{
    Async,
    twai::{self, EspTwaiFrame, StandardId},
};
// use esp_println::println;

use crate::{
    ANGLE_CAN_SIZE, BUTTON_STATE, ButtonFlags, CAN_ID_BUTTON_STATE, CAN_ID_MAIN_STATE,
    CAN_ID_MAIN_VALVE_ANGLE, CAN_ID_MAIN_VALVE_STATE, MAIN_RX_SIGNAL, MAIN_STATE, VALVE_ANGLE_X10,
    VALVE_RX_SIGNAL, VALVE_STATE,
};
#[embassy_executor::task]
pub async fn can_transmit_task(mut tx: twai::TwaiTx<'static, Async>) {
    loop {
        let state = ButtonFlags::from_bits_truncate(BUTTON_STATE.load(Ordering::Relaxed));
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

        if let Some(frame) = create_can_frame_to_send(CAN_ID_BUTTON_STATE, data.bits()) {
            let _ = tx.transmit_async(&frame).await;
        }
        Timer::after(Duration::from_millis(50)).await;
    }
}

#[embassy_executor::task]
pub async fn can_receive_task(mut rx: twai::TwaiRx<'static, Async>) {
    loop {
        if let Ok(payload) = rx.receive_async().await {
            match payload.id() {
                Id::Standard(s_id) if s_id.as_raw() == CAN_ID_MAIN_STATE => {
                    let can_data = payload.data();
                    if !can_data.is_empty() {
                        MAIN_STATE.store(can_data[0], Ordering::Relaxed);
                        MAIN_RX_SIGNAL.signal(());
                    }
                }
                Id::Standard(s_id) if s_id.as_raw() == CAN_ID_MAIN_VALVE_STATE => {
                    let can_data = payload.data();
                    if !can_data.is_empty() {
                        VALVE_STATE.store(can_data[0], Ordering::Relaxed);
                        VALVE_RX_SIGNAL.signal(());
                    }
                }
                Id::Standard(s_id) if s_id.as_raw() == CAN_ID_MAIN_VALVE_ANGLE => {
                    let can_data = payload.data();
                    if can_data.len() >= ANGLE_CAN_SIZE {
                        let angle_x10 = i16::from_le_bytes([can_data[0], can_data[1]]) as i32;
                        VALVE_ANGLE_X10.store(angle_x10, Ordering::Relaxed);
                        VALVE_RX_SIGNAL.signal(());
                    }
                }
                _ => {} // 無関係なIDを無視
            }
        }
    }
}

// --- CAN Helper Functions ---
fn create_can_frame_to_send(can_id: u16, cmd: u8) -> Option<EspTwaiFrame> {
    let id = StandardId::new(can_id)?;
    EspTwaiFrame::new(id, &[cmd])
}
