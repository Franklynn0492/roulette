//! # Pico Blinky Example
//!
//! Blinks the LED on a Pico board.
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for
//! the on-board LED.
//!
//! See the `Cargo.toml` file for Copyright and licence details.

#![no_std]
#![no_main]


#[rtic::app(device = rp_pico::hal::pac, peripherals = true)]
mod app{

    use core::convert::Infallible;
    use cortex_m::interrupt::Mutex;
    // The macro for our start-up function
    use cortex_m_rt::entry;

    // GPIO traits
    use embedded_hal::digital::v2::OutputPin;

    // Time handling traits
    use embedded_time::rate::*;

    use heapless::Vec;
    // Ensure we halt the program on panic (if we don't mention this crate it won't
    // be linked)
    use panic_halt as _;

    use rp_pico::hal::gpio::bank0::*;
    use rp_pico::hal::gpio::{Interrupt, PushPull, Output};
    // Pull in any important traits
    use rp_pico::hal::{self, prelude::*, pac, Watchdog, gpio::pin::Pin};

    #[shared]
    struct Shared {
        delay: cortex_m::delay::Delay,
        rolling: Mutex<bool>,
    }


    /// See https://rtic.rs/1/book/en/by-example/resources.html
    /// In my own words: local resources are initialized by the init-task and can only be accessed by a single task
    #[local]
    struct Local {
        led_pins: (
            Pin<Gpio21, Output<PushPull>>, Pin<Gpio22, Output<PushPull>>,
            Pin<Gpio26, Output<PushPull>>, Pin<Gpio27, Output<PushPull>>,
            Pin<Gpio28, Output<PushPull>>, Pin<Gpio16, Output<PushPull>>,
            Pin<Gpio17, Output<PushPull>>, Pin<Gpio18, Output<PushPull>>,
            Pin<Gpio19, Output<PushPull>>, Pin<Gpio20, Output<PushPull>>,
        ),
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut resets = c.device.RESETS;
        // A watchdog is a counter that is incremented internally, and once it overflows it resets the device. This means the watchdog needs to be reset 
        // regularly in order to prohibit the device from resetting. This means a watchdog prohibits the device from getting stuck or in an infinite loop  
        let mut watchdog = Watchdog::new(c.device.WATCHDOG);
        // Configure the clocks
        let clocks = hal::clocks::init_clocks_and_plls(
            rp_pico::XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();



        // The delay object lets us wait for specified amounts of time (in
        // milliseconds)
        let mut delay = cortex_m::delay::Delay::new(c.core.SYST, clocks.system_clock.freq().integer());

        // The single-cycle I/O block controls our GPIO pins
        let sio = hal::Sio::new(c.device.SIO);

        // Set the pins up according to their function on this particular board
        let pins = rp_pico::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let led_pins = (
            pins.gpio21.into_push_pull_output(), pins.gpio22.into_push_pull_output(),
            pins.gpio26.into_push_pull_output(), pins.gpio27.into_push_pull_output(),
            pins.gpio28.into_push_pull_output(), pins.gpio16.into_push_pull_output(),
            pins.gpio17.into_push_pull_output(), pins.gpio18.into_push_pull_output(),
            pins.gpio19.into_push_pull_output(), pins.gpio20.into_push_pull_output(),
        );        

        pins.gpio15.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);

        (Shared { rolling: Mutex::new(true), delay }, Local {led_pins}, init::Monotonics())
    }

    /// Executed after the init-task. Replaces the entry task
    #[idle(local = [led_pins], shared = [delay, rolling])]
    fn idle(context: idle::Context) -> ! {
        let led_pins = context.local.led_pins;
        let mut led_pins_arr: [&mut dyn OutputPin<Error = Infallible>; 10] = [
            &mut led_pins.0, &mut led_pins.1, &mut led_pins.2, &mut led_pins.3, &mut led_pins.4,
            &mut led_pins.5, &mut led_pins.6, &mut led_pins.7, &mut led_pins.8, &mut led_pins.9];

        
        let mut delay_mutex = context.shared.delay;
        
        let led_count = led_pins_arr.len();

        // Blink the LED at 1 Hz
        let mut index = 0;
        loop {
            delay_mutex.lock(|delay| {
                index = (index + 1) % led_count; 
                let led_pin = &mut led_pins_arr[index];
                led_pin.set_high().unwrap();
                delay.delay_ms(30);
                led_pin.set_low().unwrap();
            }
            )
        }
    }
}
// End of file
