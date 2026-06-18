use crate::{
    ANGLE_SCALE,
    tasks::display_common::{
        ButtonDisplayBits, DisplaySnapshot, angle_status_str, can_health_str, can_status_str,
        display_snapshot, main_sequence_state_str,
    },
};
use core::fmt::Write;
use display_interface_spi::SPIInterface;
use embassy_time::{Duration, Timer};
use embedded_graphics::{
    Drawable,
    mono_font::MonoTextStyle,
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
const FIELD_W: u32 = 230;
const FIELD_H: u32 = 20;
const ROW_H: i32 = 23;
const ROW_BURN: i32 = 30;
const ROW_CAN: i32 = ROW_BURN + ROW_H;
const ROW_PHASE: i32 = ROW_CAN + ROW_H;
const ROW_FLAGS: i32 = ROW_PHASE + ROW_H;
const ROW_GPIO: i32 = ROW_FLAGS + ROW_H;
const ROW_VALVE: i32 = ROW_GPIO + ROW_H;
const ROW_BUTTONS_1: i32 = ROW_VALVE + ROW_H;
const ROW_BUTTONS_2: i32 = ROW_BUTTONS_1 + ROW_H;

fn draw_text<D>(display: &mut D, text: &str, point: Point, color: Rgb565) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    Text::with_baseline(
        text,
        point,
        MonoTextStyle::new(&PROFONT_18_POINT, color),
        Baseline::Top,
    )
    .draw(display)
    .map(|_| ())
}

fn clear_field<D>(display: &mut D, y: i32) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    Rectangle::new(Point::new(FIELD_X, y), Size::new(FIELD_W, FIELD_H))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::WHITE))
        .draw(display)
        .map(|_| ())
}

fn draw_field<D>(display: &mut D, y: i32, text: &str) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    clear_field(display, y)?;
    draw_text(display, text, Point::new(FIELD_X, y), Rgb565::BLACK)
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
        ("BURN", ROW_BURN),
        ("CAN", ROW_CAN),
        ("PHASE", ROW_PHASE),
        ("FLAGS", ROW_FLAGS),
        ("GPIO", ROW_GPIO),
        ("VALVE", ROW_VALVE),
        ("BTN1", ROW_BUTTONS_1),
        ("BTN2", ROW_BUTTONS_2),
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
    let mut text: String<48> = String::new();
    let _ = write!(
        text,
        "{} {}",
        can_status_str(
            snapshot.can_peer_alive,
            snapshot.can_local_error,
            snapshot.can_tx_timeout
        ),
        can_health_str(snapshot.can_health)
    );
    draw_field(display, ROW_CAN, text.as_str())
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

fn draw_flags<D>(display: &mut D, snapshot: DisplaySnapshot) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let mut text: String<16> = String::new();
    let _ = write!(text, "0x{:02X}", snapshot.internal_status_flags);
    draw_field(display, ROW_FLAGS, text.as_str())
}

fn draw_gpio<D>(display: &mut D, snapshot: DisplaySnapshot) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let mut text: String<32> = String::new();
    let _ = write!(
        text,
        "OUT:0x{:02X} IN:0x{:02X}",
        snapshot.output_gpio_status, snapshot.input_gpio_status
    );
    draw_field(display, ROW_GPIO, text.as_str())
}

fn draw_valve<D>(display: &mut D, snapshot: DisplaySnapshot) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let angle_abs_x10 = snapshot.angle_x10.abs();
    let mut text: String<40> = String::new();
    let _ = write!(
        text,
        "{} {}{}.{:01}deg",
        angle_status_str(snapshot.angle_x10),
        if snapshot.angle_x10 < 0 { "-" } else { "" },
        angle_abs_x10 / ANGLE_SCALE,
        angle_abs_x10 % ANGLE_SCALE
    );
    draw_field(display, ROW_VALVE, text.as_str())
}

fn draw_buttons_1<D>(display: &mut D, buttons: ButtonDisplayBits) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let mut text: String<40> = String::new();
    let _ = write!(
        text,
        "DMP:{} FIR:{} FIL:{} SEP:{}",
        buttons.dump, buttons.fire, buttons.fill, buttons.separate
    );
    draw_field(display, ROW_BUTTONS_1, text.as_str())
}

fn draw_buttons_2<D>(display: &mut D, buttons: ButtonDisplayBits) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let mut text: String<32> = String::new();
    let _ = write!(
        text,
        "SET:{} O2:{} VLV:{}",
        buttons.valve_set, buttons.o2, buttons.valve_open
    );
    draw_field(display, ROW_BUTTONS_2, text.as_str())
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
    if previous.is_none_or(|prev| prev.internal_status_flags != snapshot.internal_status_flags) {
        draw_flags(display, snapshot)?;
    }
    if previous.is_none_or(|prev| {
        prev.output_gpio_status != snapshot.output_gpio_status
            || prev.input_gpio_status != snapshot.input_gpio_status
    }) {
        draw_gpio(display, snapshot)?;
    }
    if previous.is_none_or(|prev| prev.angle_x10 != snapshot.angle_x10) {
        draw_valve(display, snapshot)?;
    }
    if previous.is_none_or(|prev| prev.buttons != snapshot.buttons) {
        draw_buttons_1(display, snapshot.buttons)?;
        draw_buttons_2(display, snapshot.buttons)?;
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
