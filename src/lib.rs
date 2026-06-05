#![no_std]
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU8, AtomicU32};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
pub mod can;
pub mod panic;
pub mod tasks;
// --- Time Definitions ---
pub const COMMUNICATION_TIMEOUT_MS: u64 = 1000;
pub const ERROR_COMMUNICATION_TIMEOUT_MS: u64 = 300;
pub const CAN_MANAGER_HEARTBEAT_TIMEOUT_MS: u64 = 500;
pub const SAMPLING_RATE_MS: u64 = 8;

pub const CAN_RX_EVENT_PEER: u8 = 1 << 0;
pub const CAN_RX_EVENT_INTERNAL_STATUS: u8 = 1 << 1;
pub const CAN_RX_EVENT_OUTPUT_GPIO_STATUS: u8 = 1 << 2;
pub const CAN_RX_EVENT_VALVE_ANGLE: u8 = 1 << 3;
pub const CAN_RX_EVENT_INPUT_GPIO_STATUS: u8 = 1 << 4;

#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CanHealth {
    Active = 0,
    Warning = 1,
    Passive = 2,
    BusOff = 3,
}

#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CanLocalError {
    None = 0,
    BusOff = 1,
    ManagerStalled = 2,
}

// --- Valve Angle Definitions ---
const OPEN_ANGLE: f32 = -34.8;
const CLOSE_ANGLE: f32 = 55.2;
const SERVO_CENTER_POS: u16 = 7500;
const SERVO_POS_PER_DEGREE: f32 = 29.62963;
pub const ANGLE_SCALE: i32 = 10;

const fn angle_to_x10(angle_deg: f32) -> i16 {
    let scaled = angle_deg * ANGLE_SCALE as f32;
    // Const-compatible equivalent of round(), preserving the tenth-degree CAN unit.
    if scaled >= 0.0 {
        (scaled + 0.5) as i16
    } else {
        (scaled - 0.5) as i16
    }
}

pub const MAIN_VALVE_OPEN_ANGLE_X10: i16 = angle_to_x10(OPEN_ANGLE);
pub const MAIN_VALVE_CLOSED_ANGLE_X10: i16 = angle_to_x10(CLOSE_ANGLE);
pub const OPEN_POS: u16 = ((OPEN_ANGLE * SERVO_POS_PER_DEGREE) + SERVO_CENTER_POS as f32) as u16; // 29.6293:  4000 / 135 の近似値
pub const CLOSE_POS: u16 = ((CLOSE_ANGLE * SERVO_POS_PER_DEGREE) + SERVO_CENTER_POS as f32) as u16;
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
// Raw payload state last received from the peer; local CAN errors must not overwrite these.
pub static INTERNAL_STATUS_PHASE: AtomicU8 = AtomicU8::new(0);
pub static INTERNAL_STATUS_FLAGS: AtomicU8 = AtomicU8::new(0);
pub static OUTPUT_GPIO_STATUS: AtomicU8 = AtomicU8::new(0);
pub static INPUT_GPIO_STATUS: AtomicU8 = AtomicU8::new(0);
pub static VALVE_ANGLE_X10: AtomicI32 = AtomicI32::new(0);
pub static MAIN_RX_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static VALVE_RX_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static CAN_HEALTH: AtomicU8 = AtomicU8::new(CanHealth::Active as u8);
pub static CAN_PEER_ALIVE: AtomicBool = AtomicBool::new(false);
pub static CAN_LOCAL_ERROR: AtomicU8 = AtomicU8::new(CanLocalError::None as u8);
pub static CAN_TEC: AtomicU8 = AtomicU8::new(0);
pub static CAN_REC: AtomicU8 = AtomicU8::new(0);
pub static CAN_TX_ERROR_COUNT: AtomicU32 = AtomicU32::new(0);
pub static CAN_RX_ERROR_COUNT: AtomicU32 = AtomicU32::new(0);
pub static CAN_MANAGER_HEARTBEAT: AtomicU32 = AtomicU32::new(0);
pub static CAN_RX_EVENT_FLAGS: AtomicU8 = AtomicU8::new(0);
