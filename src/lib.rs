#![no_std]
pub mod state;
pub mod tasks;
// --- Time Definitions ---
pub const COMMUNICATION_TIMEOUT_MS: u64 = 3000;
pub const SAMPLING_RATE_MS: u64 = 5;

// --- CAN ID Definitions ---
pub const CAN_ID_BUTTON_STATE: u16 = 0x101;
pub const CAN_ID_MAIN_VALVE_ANGLE: u16 = 0x102;
pub const CAN_ID_FROM_PLC_ACK: u16 = 0x103;
pub const CAN_ID_MAIN_VALVE_STATE: u16 = 0x107;
pub const CAN_ID_TO_VALVE_ACK: u16 = 0x10a;

// --- Valve Angle Definitions ---
pub const OPEN_ANGLE: u8 = 100; // Controller_panelに送るときは100(-35+135)
pub const CLOSE_ANGLE: u8 = 190; // Controller_panelに送るときは190(55+135)
