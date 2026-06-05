use core::sync::atomic::Ordering;

use crate::{BUTTON_STATE, ButtonFlags, SAMPLING_RATE_MS};
use embassy_time::{Duration, Timer};
use esp_hal::gpio::Input;

/// sample the raw button states and eliminate the noise
#[embassy_executor::task]
pub async fn button_update_task(
    dump: Input<'static>,
    fire: Input<'static>,
    fill: Input<'static>,
    separate: Input<'static>,
    valve_set: Input<'static>,
    o2: Input<'static>,
    valve_open: Input<'static>,
) {
    // 過去4回分の状態を保存するリングバッファ.
    let mut history = [0u8; 4];
    let mut idx = 0;
    loop {
        let mut state = ButtonFlags::empty();
        if dump.is_high() {
            state.insert(ButtonFlags::DUMP);
        }
        if fire.is_high() {
            state.insert(ButtonFlags::FIRE);
        }
        if fill.is_high() {
            state.insert(ButtonFlags::FILL);
        }
        if separate.is_high() {
            state.insert(ButtonFlags::SEPARATE);
        }
        if valve_set.is_high() {
            state.insert(ButtonFlags::VALVE_SET);
        }
        if o2.is_high() {
            state.insert(ButtonFlags::O2);
        }
        if valve_open.is_high() {
            state.insert(ButtonFlags::VALVE_OPEN);
        }

        // 履歴を更新.
        history[idx] = state.bits();
        idx = (idx + 1) % 4;
        // 4回のサンプリング(8ms x 4 = 32ms間)すべてで1だったボタンだけを採用.
        let all_high = history[0] & history[1] & history[2] & history[3];

        // 4回のサンプリングすべてで0だったボタンのビットだけが1になる.
        let all_low = !history[0] & !history[1] & !history[2] & !history[3];

        let mut current = BUTTON_STATE.load(Ordering::Relaxed);

        // 確実にHighになったボタンを反映.
        current |= all_high;
        // 確実にLowになったボタンを反映 (all_lowのビットを0に落とす).
        current &= !all_low;
        // ※ どちらでもないボタンのビットは、以前の current の状態がそのまま維持される.

        BUTTON_STATE.store(current, Ordering::Relaxed);
        Timer::after(Duration::from_millis(SAMPLING_RATE_MS)).await; // SAMPLING_RATE_MSごとにサンプリングする.
    }
}
