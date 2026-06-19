use crate::{
    ANGLE_SCALE, ANGLE_STATUS_TOLERANCE_DEG, BUTTON_STATE, ButtonFlags, CAN_HEALTH,
    CAN_LOCAL_ERROR, CAN_PEER_ALIVE, CAN_TX_TIMEOUT_ACTIVE, CanHealth, CanLocalError,
    INPUT_GPIO_STATUS, INTERNAL_STATUS_FLAGS, INTERNAL_STATUS_PHASE, MAIN_VALVE_CLOSED_ANGLE_X10,
    MAIN_VALVE_OPEN_ANGLE_X10, OUTPUT_GPIO_STATUS, VALVE_ANGLE_RECEIVED, VALVE_ANGLE_X10,
};
use core::sync::atomic::Ordering;

const SERVO_ANGLE_LIMIT_X10: i32 = 180 * ANGLE_SCALE;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ButtonDisplayBits {
    pub dump: u8,
    pub fire: u8,
    pub fill: u8,
    pub separate: u8,
    pub valve_set: u8,
    pub o2: u8,
    pub valve_open: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DisplaySnapshot {
    pub output_gpio_status: u8,
    pub input_gpio_status: u8,
    pub internal_status_flags: u8,
    pub angle_x10: i32,
    pub main_sequence_state: u8,
    pub can_peer_alive: bool,
    pub can_local_error: u8,
    pub can_tx_timeout: bool,
    pub can_health: u8,
    pub valve_angle_received: bool,
    pub buttons: ButtonDisplayBits,
}

pub fn bit(flag: bool) -> u8 {
    flag as u8
}

pub fn button_display_bits(buttons: ButtonFlags) -> ButtonDisplayBits {
    ButtonDisplayBits {
        dump: bit(buttons.contains(ButtonFlags::DUMP)),
        fire: bit(buttons.contains(ButtonFlags::FIRE)),
        fill: bit(buttons.contains(ButtonFlags::FILL)),
        separate: bit(buttons.contains(ButtonFlags::SEPARATE)),
        valve_set: bit(buttons.contains(ButtonFlags::VALVE_SET)),
        o2: bit(buttons.contains(ButtonFlags::O2)),
        valve_open: bit(buttons.contains(ButtonFlags::VALVE_OPEN)),
    }
}

pub fn display_snapshot() -> DisplaySnapshot {
    let buttons = ButtonFlags::from_bits_truncate(BUTTON_STATE.load(Ordering::Relaxed));
    DisplaySnapshot {
        output_gpio_status: OUTPUT_GPIO_STATUS.load(Ordering::Relaxed),
        input_gpio_status: INPUT_GPIO_STATUS.load(Ordering::Relaxed),
        internal_status_flags: INTERNAL_STATUS_FLAGS.load(Ordering::Relaxed),
        angle_x10: VALVE_ANGLE_X10.load(Ordering::Relaxed),
        main_sequence_state: INTERNAL_STATUS_PHASE.load(Ordering::Relaxed),
        can_peer_alive: CAN_PEER_ALIVE.load(Ordering::Relaxed),
        can_local_error: CAN_LOCAL_ERROR.load(Ordering::Relaxed),
        can_tx_timeout: CAN_TX_TIMEOUT_ACTIVE.load(Ordering::Relaxed),
        can_health: CAN_HEALTH.load(Ordering::Relaxed),
        valve_angle_received: VALVE_ANGLE_RECEIVED.load(Ordering::Relaxed),
        buttons: button_display_bits(buttons),
    }
}

pub fn main_sequence_state_str(state: u8) -> &'static str {
    match state {
        0 => "Idle",
        1 => "Firing",
        2 => "Timeout",
        3 => "Abort",
        _ => "Unknown",
    }
}

pub fn can_health_str(health: u8) -> &'static str {
    match health {
        x if x == CanHealth::Active as u8 => "Active",
        x if x == CanHealth::Warning as u8 => "Warning",
        x if x == CanHealth::Passive as u8 => "Passive",
        x if x == CanHealth::BusOff as u8 => "BusOff",
        _ => "Unknown",
    }
}

pub fn can_status_str(peer_alive: bool, local_error: u8, tx_timeout: bool) -> &'static str {
    match local_error {
        x if x == CanLocalError::BusOff as u8 => "CAN BUS-OFF",
        x if x == CanLocalError::ManagerStalled as u8 => "CAN TASK STUCK",
        _ if tx_timeout => "CAN TX TIMEOUT",
        _ if peer_alive => "CAN OK",
        _ => "CAN TIMEOUT",
    }
}

pub fn can_is_ok(snapshot: DisplaySnapshot) -> bool {
    snapshot.can_peer_alive
        && snapshot.can_local_error == CanLocalError::None as u8
        && !snapshot.can_tx_timeout
        && snapshot.can_health == CanHealth::Active as u8
}

pub fn angle_status_str(angle_x10: i32) -> &'static str {
    let open_angle_x10 = i32::from(MAIN_VALVE_OPEN_ANGLE_X10);
    let close_angle_x10 = i32::from(MAIN_VALVE_CLOSED_ANGLE_X10);
    let tolerance_x10 = ANGLE_STATUS_TOLERANCE_DEG * ANGLE_SCALE;

    if (angle_x10 - close_angle_x10).abs() <= tolerance_x10 {
        return "Close!";
    }
    if (angle_x10 - open_angle_x10).abs() <= tolerance_x10 {
        return "Open!";
    }
    "Invalid"
}

pub fn servo_angle_is_valid(angle_x10: i32) -> bool {
    angle_x10.abs() <= SERVO_ANGLE_LIMIT_X10
}
