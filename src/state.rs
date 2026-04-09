use core::sync::atomic::AtomicU8;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};

// ボタン変数(各bitがswのオンオフに対応).
pub static BUTTON_STATE: AtomicU8 = AtomicU8::new(0);
pub static VALVE_STATE: AtomicU8 = AtomicU8::new(0);
pub static VALVE_ANGLE: AtomicU8 = AtomicU8::new(0);
pub static MAIN_STATE: AtomicU8 = AtomicU8::new(0);
pub static MAIN_RX_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static VALVE_RX_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
