use core::sync::atomic::Ordering;
use embassy_time::{Duration, Timer};
use embedded_can::{Frame, Id};
use esp_hal::{
    Async,
    twai::{self, EspTwaiFrame, StandardId},
};
// use esp_println::println;

use crate::{
    ANGLE_CAN_SIZE, BUTTON_STATE, CAN_ID_BUTTON_STATE, CAN_ID_MAIN_STATE, CAN_ID_MAIN_VALVE_ANGLE,
    CAN_ID_MAIN_VALVE_STATE, MAIN_RX_FLAG, MAIN_STATE, VALVE_ANGLE_X10, VALVE_RX_FLAG, VALVE_STATE,
};
#[embassy_executor::task]
pub async fn can_transmit_task(mut tx: twai::TwaiTx<'static, Async>) {
    loop {
        let state = BUTTON_STATE.load(Ordering::Relaxed);
        let fire = (state >> 1) & 1;
        let fill = (state >> 2) & 1;
        let separate = (state >> 3) & 1;
        let o2_test = (state >> 5) & 1;
        let mut data = 0;
        data |= state & 1;
        data |= (fire & !fill) << 1;
        data |= fill << 2;
        data |= (separate & !fill) << 3;
        data |= state & (1 << 4);
        data |= (!fire & o2_test) << 5;
        data |= state & (1 << 6);
        if let Some(frame) = create_can_frame_to_send(CAN_ID_BUTTON_STATE, data) {
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
                        MAIN_RX_FLAG.store(true, Ordering::Relaxed);
                        MAIN_STATE.store(can_data[0], Ordering::Relaxed);
                    }
                }
                Id::Standard(s_id) if s_id.as_raw() == CAN_ID_MAIN_VALVE_STATE => {
                    let can_data = payload.data();
                    if !can_data.is_empty() {
                        VALVE_STATE.store(can_data[0], Ordering::Relaxed);
                        VALVE_RX_FLAG.store(true, Ordering::Relaxed);
                    }
                }
                Id::Standard(s_id) if s_id.as_raw() == CAN_ID_MAIN_VALVE_ANGLE => {
                    let can_data = payload.data();
                    if can_data.len() >= ANGLE_CAN_SIZE {
                        let angle_x10 = i16::from_le_bytes([can_data[0], can_data[1]]) as i32;
                        VALVE_RX_FLAG.store(true, Ordering::Relaxed);
                        VALVE_ANGLE_X10.store(angle_x10, Ordering::Relaxed);
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
