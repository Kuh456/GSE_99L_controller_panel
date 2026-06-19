use crate::{
    ANGLE_SCALE,
    tasks::display_common::{
        DisplaySnapshot, can_is_ok, can_status_str, display_snapshot, main_sequence_state_str,
        servo_angle_is_valid,
    },
};
use core::fmt::Write;
use display_interface_spi::SPIInterface;
use embassy_time::{Duration, Timer};
use embedded_graphics::{
    Drawable,
    mono_font::{MonoTextStyle, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::{DrawTarget, Point, RgbColor, WebColors},
    text::{Baseline, Text},
};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use esp_hal::{Blocking, gpio::Output, spi::master::Spi};
use heapless::String;
use ili9341::Ili9341;
use profont::{PROFONT_14_POINT, PROFONT_18_POINT};

pub type LcdDisplay = Ili9341<
    SPIInterface<
        ExclusiveDevice<Spi<'static, Blocking>, Output<'static>, NoDelay>,
        Output<'static>,
    >,
    Output<'static>,
>;

const FIELD_X: i32 = 84;
const FIELD_CHARS: usize = 23;
const ROW_H: i32 = 23;
const ROW_PHASE: i32 = 30;
const ROW_BURN: i32 = ROW_PHASE + ROW_H;
const ROW_CAN: i32 = ROW_BURN + ROW_H;
const ROW_OUTPUT_1: i32 = ROW_CAN + ROW_H;
const ROW_OUTPUT_2: i32 = ROW_OUTPUT_1 + ROW_H;
const ROW_OUTPUT_3: i32 = ROW_OUTPUT_2 + ROW_H;
const ROW_VALVE: i32 = ROW_OUTPUT_3 + ROW_H;
const ROW_SERVO: i32 = ROW_VALVE + ROW_H;
const OUT_DUMP: u8 = 1 << 0;
const OUT_IGNITER: u8 = 1 << 1;
const OUT_FILL: u8 = 1 << 2;
const OUT_MAIN: u8 = 1 << 4;
const OUT_O2: u8 = 1 << 5;

fn draw_field_color<D>(display: &mut D, y: i32, text: &str, color: Rgb565) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let mut padded: String<48> = String::new();
    let _ = write!(padded, "{}", text);
    while padded.len() < FIELD_CHARS {
        let _ = padded.push(' ');
    }

    Text::with_baseline(
        padded.as_str(),
        Point::new(FIELD_X, y),
        MonoTextStyleBuilder::new()
            .font(&PROFONT_18_POINT)
            .text_color(color)
            .background_color(Rgb565::WHITE)
            .build(),
        Baseline::Top,
    )
    .draw(display)
    .map(|_| ())
}

fn draw_field<D>(display: &mut D, y: i32, text: &str) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    draw_field_color(display, y, text, Rgb565::BLACK)
}

fn draw_static_labels<D>(display: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    Text::with_baseline(
        "GSE CTRL",
        Point::new(6, 4),
        MonoTextStyle::new(&PROFONT_18_POINT, Rgb565::RED),
        Baseline::Top,
    )
    .draw(display)?;

    let label_style = MonoTextStyle::new(&PROFONT_14_POINT, Rgb565::CSS_DIM_GRAY);
    for (label, y) in [
        ("PHASE", ROW_PHASE),
        ("BURN", ROW_BURN),
        ("CAN", ROW_CAN),
        ("OUT1", ROW_OUTPUT_1),
        ("OUT2", ROW_OUTPUT_2),
        ("OUT3", ROW_OUTPUT_3),
        ("VALVE", ROW_VALVE),
        ("SERVO", ROW_SERVO),
    ] {
        Text::with_baseline(label, Point::new(6, y + 2), label_style, Baseline::Top)
            .draw(display)?;
    }

    Ok(())
}

fn burned_value(burned: bool) -> &'static str {
    if burned { "Done" } else { "Yet" }
}

fn draw_burned<D>(display: &mut D, burned: bool) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    draw_field(display, ROW_BURN, burned_value(burned))
}

fn draw_can<D>(display: &mut D, snapshot: DisplaySnapshot) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    if can_is_ok(snapshot) {
        return draw_field(display, ROW_CAN, "OK");
    }

    let mut text: String<32> = String::new();
    let status = can_status_str(
        snapshot.can_peer_alive,
        snapshot.can_local_error,
        snapshot.can_tx_timeout,
    );
    let _ = write!(
        text,
        "ERROR {}",
        status.strip_prefix("CAN ").unwrap_or(status)
    );
    draw_field_color(display, ROW_CAN, text.as_str(), Rgb565::RED)
}

fn draw_phase<D>(display: &mut D, snapshot: DisplaySnapshot) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    draw_field(
        display,
        ROW_PHASE,
        main_sequence_state_str(snapshot.main_sequence_state),
    )
}

fn on_off(active: bool) -> &'static str {
    if active { "ON" } else { "OFF" }
}

fn draw_output_gpio<D>(display: &mut D, snapshot: DisplaySnapshot) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let output = snapshot.output_gpio_status;
    let mut text: String<40> = String::new();
    let _ = write!(
        text,
        "DMP:{} FIL:{}",
        on_off(output & OUT_DUMP != 0),
        on_off(output & OUT_FILL != 0),
    );
    draw_field(display, ROW_OUTPUT_1, text.as_str())?;

    text.clear();
    let _ = write!(
        text,
        "IGN:{} MAIN:{}",
        on_off(output & OUT_IGNITER != 0),
        on_off(output & OUT_MAIN != 0),
    );
    draw_field(display, ROW_OUTPUT_2, text.as_str())?;

    text.clear();
    let _ = write!(
        text,
        "O2:{} SET:{}",
        on_off(output & OUT_O2 != 0),
        on_off(snapshot.buttons.valve_set != 0),
    );
    draw_field(display, ROW_OUTPUT_3, text.as_str())
}

fn draw_valve<D>(display: &mut D, snapshot: DisplaySnapshot) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let error = !snapshot.can_peer_alive
        || !snapshot.valve_angle_received
        || !servo_angle_is_valid(snapshot.angle_x10);
    let text = if error { "VALVE ERROR" } else { "VALVE OK" };
    draw_field_color(
        display,
        ROW_VALVE,
        text,
        if error { Rgb565::RED } else { Rgb565::BLACK },
    )
}

fn draw_servo<D>(display: &mut D, snapshot: DisplaySnapshot) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    if !snapshot.can_peer_alive {
        return draw_field_color(display, ROW_SERVO, "SERVO ERROR", Rgb565::RED);
    }
    if !snapshot.valve_angle_received || !servo_angle_is_valid(snapshot.angle_x10) {
        return draw_field_color(display, ROW_SERVO, "SERVO INVALID", Rgb565::RED);
    }

    let angle_abs_x10 = snapshot.angle_x10.abs();
    let mut text: String<40> = String::new();
    let _ = write!(
        text,
        "SERVO {}{}.{:01}deg",
        if snapshot.angle_x10 < 0 { "-" } else { "" },
        angle_abs_x10 / ANGLE_SCALE,
        angle_abs_x10 % ANGLE_SCALE
    );
    draw_field(display, ROW_SERVO, text.as_str())
}

fn draw_changed_fields<D>(
    display: &mut D,
    burned: bool,
    previous_burned: Option<bool>,
    snapshot: DisplaySnapshot,
    previous: Option<DisplaySnapshot>,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    if previous_burned != Some(burned) {
        draw_burned(display, burned)?;
    }

    if previous.is_none_or(|prev| {
        prev.can_peer_alive != snapshot.can_peer_alive
            || prev.can_local_error != snapshot.can_local_error
            || prev.can_tx_timeout != snapshot.can_tx_timeout
            || prev.can_health != snapshot.can_health
    }) {
        draw_can(display, snapshot)?;
    }

    if previous.is_none_or(|prev| prev.main_sequence_state != snapshot.main_sequence_state) {
        draw_phase(display, snapshot)?;
    }
    if previous.is_none_or(|prev| {
        prev.output_gpio_status != snapshot.output_gpio_status
            || prev.buttons.valve_set != snapshot.buttons.valve_set
    }) {
        draw_output_gpio(display, snapshot)?;
    }
    if previous.is_none_or(|prev| {
        prev.angle_x10 != snapshot.angle_x10
            || prev.valve_angle_received != snapshot.valve_angle_received
            || prev.can_peer_alive != snapshot.can_peer_alive
    }) {
        draw_valve(display, snapshot)?;
        draw_servo(display, snapshot)?;
    }
    Ok(())
}

#[embassy_executor::task]
pub async fn lcd_display_task(mut display: LcdDisplay) {
    display.clear(Rgb565::WHITE).unwrap();
    draw_static_labels(&mut display).unwrap();

    let mut previous_snapshot: Option<DisplaySnapshot> = None;
    let mut previous_burned: Option<bool> = None;
    let mut burned = false;

    loop {
        let snapshot = display_snapshot();
        if snapshot.main_sequence_state == 2 {
            burned = true;
        }

        draw_changed_fields(
            &mut display,
            burned,
            previous_burned,
            snapshot,
            previous_snapshot,
        )
        .unwrap();

        previous_snapshot = Some(snapshot);
        previous_burned = Some(burned);
        Timer::after(Duration::from_millis(1000)).await;
    }
}
