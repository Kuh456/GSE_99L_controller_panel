use esp_hal::system::Cpu;
use esp_println::println;

#[unsafe(no_mangle)]
pub fn custom_pre_backtrace() {
    println!("\n\n\n\n\x1b[31mPanicked!!! Core: {:?}", Cpu::current());
}

#[unsafe(no_mangle)]
pub fn custom_halt() -> ! {
    println!("Rebooting...");
    esp_hal::system::software_reset();
}
