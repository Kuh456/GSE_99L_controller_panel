#![no_std]
use core::sync::atomic::{AtomicI32, AtomicU8, AtomicU32};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
pub mod panic;
pub mod tasks;
// --- Time Definitions ---
pub const COMMUNICATION_TIMEOUT_MS: u64 = 3000;
pub const ERROR_COMMUNICATION_TIMEOUT_MS: u64 = 300;
pub const SAMPLING_RATE_MS: u64 = 8;

// --- CAN ID Definitions ---
pub const CAN_ID_BUTTON_STATE: u16 = 0x101;
pub const CAN_ID_MAIN_VALVE_ANGLE: u16 = 0x102;
pub const CAN_ID_MAIN_STATE: u16 = 0x103;
pub const CAN_ID_MAIN_VALVE_STATE: u16 = 0x107;

#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CanHealth {
    Active = 0,
    Warning = 1,
    Passive = 2,
    BusOff = 3,
    Recovering = 4,
}

// --- Valve Angle Definitions ---
pub const OPEN_ANGLE: i32 = -35;
pub const CLOSE_ANGLE: i32 = 55;
pub const ANGLE_CAN_SIZE: usize = 2;
pub const ANGLE_SCALE: i32 = 10;
pub const ANGLE_STATUS_TOLERANCE_DEG: i32 = 20;

bitflags::bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub struct ButtonFlags: u8 {
        const DUMP = 1 << 0;
        const FIRE = 1 << 1;
        const FILL = 1 << 2;
        const SEPARATE = 1 << 3;
        const VALVE_SET = 1 << 4;
        const O2 = 1 << 5;
        const VALVE_OPEN = 1 << 6;
    }
}

// ボタン変数(各bitがswのオンオフに対応).
pub static BUTTON_STATE: AtomicU8 = AtomicU8::new(0);
pub static VALVE_STATE: AtomicU8 = AtomicU8::new(0);
pub static VALVE_ANGLE_X10: AtomicI32 = AtomicI32::new(0);
pub static MAIN_STATE: AtomicU8 = AtomicU8::new(0);
pub static MAIN_RX_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static VALVE_RX_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static CAN_HEALTH: AtomicU8 = AtomicU8::new(CanHealth::Active as u8);
pub static CAN_TEC: AtomicU8 = AtomicU8::new(0);
pub static CAN_REC: AtomicU8 = AtomicU8::new(0);
pub static CAN_TX_ERROR_COUNT: AtomicU32 = AtomicU32::new(0);
pub static CAN_RX_ERROR_COUNT: AtomicU32 = AtomicU32::new(0);
pub static CAN_MANAGER_HEARTBEAT: AtomicU32 = AtomicU32::new(0);
