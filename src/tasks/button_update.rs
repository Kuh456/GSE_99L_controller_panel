use core::sync::atomic::Ordering;

use crate::{BUTTON_STATE, SAMPLING_RATE_MS};
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
    main_reset: Input<'static>,
) {
    let mut state = 0;
    // 過去4回分の状態を保存するリングバッファ.
    let mut history = [0u8; 4];
    let mut idx = 0;
    loop {
        state = 0;
        state |= dump.is_high() as u8;
        state |= (fire.is_high() as u8) << 1;
        state |= (fill.is_high() as u8) << 2;
        state |= (separate.is_high() as u8) << 3;
        state |= (valve_set.is_high() as u8) << 4;
        state |= (o2.is_high() as u8) << 5;
        state |= (main_reset.is_high() as u8) << 6;

        // 履歴を更新.
        history[idx] = state;
        idx = (idx + 1) % 4;
        // 4回のサンプリング(40ms間)すべてで1だったボタンのビットだけが1になる.
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
