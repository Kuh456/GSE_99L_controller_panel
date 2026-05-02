#![no_std]
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU8};
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

// --- Valve Angle Definitions ---
pub const OPEN_ANGLE: i32 = -35;
pub const CLOSE_ANGLE: i32 = 55;
pub const ANGLE_CAN_SIZE: usize = 2;
pub const ANGLE_SCALE: i32 = 10;
pub const ANGLE_STATUS_TOLERANCE_DEG: i32 = 20;

// ボタン変数(各bitがswのオンオフに対応).
pub static BUTTON_STATE: AtomicU8 = AtomicU8::new(0);
pub static VALVE_STATE: AtomicU8 = AtomicU8::new(0);
pub static VALVE_ANGLE_X10: AtomicI32 = AtomicI32::new(0);
pub static MAIN_STATE: AtomicU8 = AtomicU8::new(0);
pub static MAIN_RX_FLAG: AtomicBool = AtomicBool::new(false);
pub static VALVE_RX_FLAG: AtomicBool = AtomicBool::new(false);
