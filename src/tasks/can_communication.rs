use core::sync::atomic::Ordering;
use embassy_time::{Duration, Timer, with_timeout};
use embedded_can::{Frame, Id};
use esp_hal::{
    Async,
    twai::{self, EspTwaiFrame, StandardId},
};
use esp_println::println;

use crate::{
    BUTTON_STATE, CAN_ID_BUTTON_STATE, CAN_ID_FROM_MAIN_ACK, CAN_ID_MAIN_VALVE_ANGLE,
    CAN_ID_MAIN_VALVE_STATE, CAN_ID_TO_VALVE_ACK, MAIN_RX_SIGNAL, MAIN_STATE, VALVE_ANGLE,
    VALVE_RX_SIGNAL, VALVE_STATE,
};
#[embassy_executor::task]
pub async fn can_transmit_task(mut tx: twai::TwaiTx<'static, Async>) {
    let mut data: u8 = 0;
    loop {
        let state = BUTTON_STATE.load(Ordering::Relaxed);
        let fire = (state >> 1) & 1; // dataの2bit目は点火(fire).
        let fill = (state >> 2) & 1; // dataの3bit目は充填(fill).
        let separate = (state >> 3) & 1; // dataの4bit目は切り離し(separate).
        let o2_test = (state >> 5) & 1; // dataの6bit目は試験用のo2.
        data = 0;
        data |= state & 1; // dataの1bit目は機体内脱圧(dump).
        data |= (fire & !fill) << 1; // 充填中に点火しないように.
        data |= fill << 2;
        data |= (separate & !fill) << 3; // 充填中に切り離ししないように.
        data |= state & (1 << 4); // dataの5bit目はバルブセット(valve set).
        data |= (!fire & o2_test) << 5; // 点火中はo2_testを無効にする.
        data |= state & (1 << 6); // dataの7bit目はstate_reset.
        println!("send: 0b{:08b}", data);
        send_can_message(&mut tx, CAN_ID_BUTTON_STATE, &[data]).await;
        send_can_message(&mut tx, CAN_ID_TO_VALVE_ACK, &[0]).await;
        Timer::after(Duration::from_millis(50)).await;
    }
}

#[embassy_executor::task]
pub async fn can_receive_task(mut rx: twai::TwaiRx<'static, Async>) {
    loop {
        // ラッパー関数から Option が返ってくるので、Some のときだけ処理
        if let Some(payload) = receive_can_message(&mut rx, 3).await {
            match payload.id() {
                Id::Standard(s_id) if s_id.as_raw() == CAN_ID_FROM_MAIN_ACK => {
                    MAIN_RX_SIGNAL.signal(());
                    MAIN_STATE.store(payload.data()[0], Ordering::Relaxed);
                }
                Id::Standard(s_id) if s_id.as_raw() == CAN_ID_MAIN_VALVE_STATE => {
                    let can_data = payload.data();
                    if can_data.len() > 0 {
                        VALVE_STATE.store(can_data[0], Ordering::Relaxed);
                        VALVE_RX_SIGNAL.signal(());
                    }
                }
                Id::Standard(s_id) if s_id.as_raw() == CAN_ID_MAIN_VALVE_ANGLE => {
                    let can_data = payload.data();
                    if can_data.len() > 0 {
                        let angle = can_data[0] - 130;
                        VALVE_RX_SIGNAL.signal(());
                        VALVE_ANGLE.store(angle, Ordering::Relaxed);
                    }
                }
                _ => {} // 無関係なIDを無視
            }
        }
    }
}

// --- CAN Helper Functions ---
/// CANメッセージを送信するラッパー関数
async fn send_can_message(tx: &mut twai::TwaiTx<'_, Async>, id: u16, data: &[u8]) {
    // 11bitの標準IDとして有効かチェック
    let std_id = match StandardId::new(id) {
        Some(id) => id,
        None => {
            println!("Error: Invalid CAN ID (0x{:x})", id);
            return;
        }
    };

    // フレームの作成
    let frame = EspTwaiFrame::new(std_id, data).unwrap();

    // 送信処理
    let result = tx.transmit_async(&frame).await;

    match result {
        Ok(()) => {} // 送信成功
        Err(e) => {
            println!("CAN Transmit Error (ID: 0x{:x}): {:?}", id, e);
        }
    }
}

/// CANメッセージを受信するラッパー関数
pub async fn receive_can_message(
    rx: &mut twai::TwaiRx<'_, Async>,
    timeout_secs: u64,
) -> Option<EspTwaiFrame> {
    let result = with_timeout(Duration::from_secs(timeout_secs), rx.receive_async()).await;

    match result {
        Ok(Ok(frame)) => Some(frame), // 受信成功
        Ok(Err(e)) => {
            println!("CAN Receive Error: {:?}", e);
            None
        }
        Err(_) => {
            // タイムアウトは通常の待機状態なのでエラーログは出さない
            None
        }
    }
}
