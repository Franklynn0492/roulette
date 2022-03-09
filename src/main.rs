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

    use rp_pico::hal::gpio::Interrupt;
    // Pull in any important traits
    use rp_pico::hal::prelude::*;

    // A shorter alias for the Peripheral Access Crate, which provides low-level
    // register access
    use rp_pico::hal::pac;

    // A shorter alias for the Hardware Abstraction Layer, which provides
    // higher-level drivers.
    use rp_pico::hal;


    #[shared]
    struct Shared {
        delay: cortex_m::delay::Delay,
        rolling: Mutex<bool>,
    }


    /// See https://rtic.rs/1/book/en/by-example/resources.html
    /// In my own words: local resources are initialized by the init-task and can only be accessed by a single task
    #[local]
    struct Local {
        led_pins: Vec<&'static (dyn OutputPin<Error = Infallible> + Send + Sync), 10>,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        // Grab our singleton objects
        let mut pac = pac::Peripherals::take().unwrap();
        let core = pac::CorePeripherals::take().unwrap();

        // Set up the watchdog driver - needed by the clock setup code
        let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

        // Configure the clocks
        //
        // The default is to generate a 125 MHz system clock
        let clocks = hal::clocks::init_clocks_and_plls(
            rp_pico::XOSC_CRYSTAL_FREQ,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        // The delay object lets us wait for specified amounts of time (in
        // milliseconds)
        let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

        // The single-cycle I/O block controls our GPIO pins
        let sio = hal::Sio::new(pac.SIO);

        // Set the pins up according to their function on this particular board
        let pins = rp_pico::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        let mut led_pins: Vec<&'static (dyn OutputPin<Error = Infallible> + Send + Sync), 10> = Vec::new();
        led_pins.push(&pins.gpio21.into_push_pull_output());
        led_pins.push(&pins.gpio22.into_push_pull_output());
        led_pins.push(&pins.gpio26.into_push_pull_output());
        led_pins.push(&pins.gpio27.into_push_pull_output());
        led_pins.push(&pins.gpio28.into_push_pull_output());
        led_pins.push(&pins.gpio16.into_push_pull_output());
        led_pins.push(&pins.gpio17.into_push_pull_output());
        led_pins.push(&pins.gpio18.into_push_pull_output());
        led_pins.push(&pins.gpio19.into_push_pull_output());
        led_pins.push(&pins.gpio20.into_push_pull_output());
/*
        [Box::new(pins.gpio21.into_push_pull_output()), &mut pins.gpio22.into_push_pull_output(),
            &mut pins.gpio26.into_push_pull_output(), &mut pins.gpio27.into_push_pull_output(),
            &mut pins.gpio28.into_push_pull_output(), &mut pins.gpio16.into_push_pull_output(),
            &mut pins.gpio17.into_push_pull_output(), &mut pins.gpio18.into_push_pull_output(),
            &mut pins.gpio19.into_push_pull_output(), &mut pins.gpio20.into_push_pull_output(),
        ];*/
        
        pins.gpio15.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);

        (Shared { rolling: Mutex::new(true), delay }, Local {led_pins}, init::Monotonics())
    }

    /// Executed after the init-task. Replaces the entry task
    #[idle(local = [led_pins], shared = [delay, rolling])]
    fn idle(context: idle::Context) -> ! {
        let mut led_pins = context.local.led_pins;
        let mut delay_mutex = context.shared.delay;
        
        let led_count = led_pins.len();

        // Blink the LED at 1 Hz
        let mut index = 0;
        loop {
            delay_mutex.lock(|delay| {
                index = (index + 1) % led_count; 
                let led_pin = &mut led_pins[index];
                led_pin.set_high().unwrap();
                delay.delay_ms(50);
                led_pin.set_low().unwrap();
            }
            )
        }
    }
}
// End of file
