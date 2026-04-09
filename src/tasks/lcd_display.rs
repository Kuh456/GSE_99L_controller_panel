use display_interface_spi::SPIInterface;
use embassy_time::{Duration, Timer};
use embedded_graphics::{
    Drawable,
    mono_font::MonoTextStyle,
    pixelcolor::Rgb565,
    prelude::{DrawTarget, Point, RgbColor, WebColors},
    text::{Baseline, Text},
};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use esp_hal::{Async, gpio::Output, spi::master::Spi};
use ili9341::Ili9341;
use profont::{PROFONT_18_POINT, PROFONT_24_POINT};

#[embassy_executor::task]
pub async fn lcd_display_task(
    mut display: Ili9341<
        SPIInterface<
            ExclusiveDevice<Spi<'static, Async>, Output<'static>, NoDelay>,
            Output<'static>,
        >,
        Output<'static>,
    >,
) {
    display.clear(Rgb565::WHITE).unwrap();
    loop {
        let text_style = MonoTextStyle::new(&PROFONT_24_POINT, Rgb565::RED);
        Text::with_baseline("impl Rust", Point::new(50, 150), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();

        let text_style = MonoTextStyle::new(&PROFONT_18_POINT, Rgb565::CSS_DIM_GRAY);

        Text::with_baseline("for ESP32", Point::new(60, 180), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        Timer::after(Duration::from_millis(1000)).await; // 1秒ごとに状態を送信.
    }
}
