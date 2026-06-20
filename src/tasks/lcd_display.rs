use crate::{
    ANGLE_SCALE,
    tasks::display_common::{
        DisplaySnapshot, can_is_ok, can_status_str, display_snapshot, main_sequence_state_str,
    },
};
use core::fmt::Write;
use display_interface_spi::SPIInterface;
use embassy_time::{Duration, Timer};
use embedded_graphics::{
    Drawable,
    mono_font::{MonoTextStyle, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::{DrawTarget, Point, Primitive, RgbColor, Size, WebColors},
    primitives::{PrimitiveStyle, Rectangle},
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
const FIELD_W: u32 = 236;
const OUT_DUMP: u8 = 1 << 0;
const OUT_FILL: u8 = 1 << 1;
const OUT_SEPARATE: u8 = 1 << 2;
const OUT_O2: u8 = 1 << 3;
const OUT_IGNITER: u8 = 1 << 4;
const FAULT_SERVO_COMM_ERROR: u8 = 1 << 2;

fn draw_field_with_style<D>(
    display: &mut D,
    y: i32,
    text: &str,
    text_color: Rgb565,
    background_color: Rgb565,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    Rectangle::new(Point::new(FIELD_X, y), Size::new(FIELD_W, ROW_H as u32))
        .into_styled(PrimitiveStyle::with_fill(background_color))
        .draw(display)?;

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
            .text_color(text_color)
            .background_color(background_color)
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
    draw_field_with_style(display, y, text, Rgb565::WHITE, Rgb565::BLACK)
}

fn draw_ok_field<D>(display: &mut D, y: i32, text: &str) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    draw_field_with_style(display, y, text, Rgb565::GREEN, Rgb565::BLACK)
}

fn draw_error_field<D>(display: &mut D, y: i32, text: &str) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    draw_field_with_style(display, y, text, Rgb565::WHITE, Rgb565::RED)
}

fn draw_static_labels<D>(display: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    Text::with_baseline(
        "GSE CTRL",
        Point::new(6, 4),
        MonoTextStyle::new(&PROFONT_18_POINT, Rgb565::GREEN),
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
        ("COMM", ROW_VALVE),
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
        return draw_ok_field(display, ROW_CAN, "CAN OK");
    }

    let mut text: String<32> = String::new();
    let status = can_status_str(
        snapshot.can_peer_alive,
        snapshot.can_local_error,
        snapshot.can_tx_timeout,
    );
    let _ = write!(
        text,
        "[ ERROR ] {}",
        status.strip_prefix("CAN ").unwrap_or(status)
    );
    draw_error_field(display, ROW_CAN, text.as_str())
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
        "SEP:{} O2:{}",
        on_off(output & OUT_SEPARATE != 0),
        on_off(output & OUT_O2 != 0),
    );
    draw_field(display, ROW_OUTPUT_2, text.as_str())?;

    text.clear();
    let _ = write!(text, "IGN:{}", on_off(output & OUT_IGNITER != 0));
    draw_field(display, ROW_OUTPUT_3, text.as_str())
}

fn draw_servo_comm<D>(display: &mut D, snapshot: DisplaySnapshot) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let error = snapshot.internal_status_flags & FAULT_SERVO_COMM_ERROR != 0;
    let text = if error {
        "SERVO COMM ERR"
    } else {
        "SERVO COMM OK"
    };
    if error {
        draw_field_with_style(display, ROW_VALVE, text, Rgb565::WHITE, Rgb565::RED)
    } else {
        draw_ok_field(display, ROW_VALVE, text)
    }
}

fn draw_servo<D>(display: &mut D, snapshot: DisplaySnapshot) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    if !snapshot.can_peer_alive {
        return draw_field_with_style(
            display,
            ROW_SERVO,
            "SERVO ERROR",
            Rgb565::WHITE,
            Rgb565::RED,
        );
    }
    if !snapshot.valve_angle_received {
        return draw_field_with_style(
            display,
            ROW_SERVO,
            "SERVO ERROR INVALID",
            Rgb565::WHITE,
            Rgb565::RED,
        );
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
    if previous.is_none_or(|prev| prev.output_gpio_status != snapshot.output_gpio_status) {
        draw_output_gpio(display, snapshot)?;
    }
    if previous.is_none_or(|prev| {
        prev.angle_x10 != snapshot.angle_x10
            || prev.valve_angle_received != snapshot.valve_angle_received
            || prev.can_peer_alive != snapshot.can_peer_alive
            || prev.internal_status_flags != snapshot.internal_status_flags
    }) {
        draw_servo_comm(display, snapshot)?;
        draw_servo(display, snapshot)?;
    }
    Ok(())
}

#[embassy_executor::task]
pub async fn lcd_display_task(mut display: LcdDisplay) {
    display.clear(Rgb565::BLACK).unwrap();
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
