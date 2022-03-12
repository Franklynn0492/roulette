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
    use core::sync::atomic::{AtomicBool, AtomicU32};

    // GPIO traits
    use embedded_hal::digital::v2::{OutputPin, PinState};

    // Time handling traits
    use embedded_time::rate::*;

    // Ensure we halt the program on panic (if we don't mention this crate it won't
    // be linked)
    use panic_halt as _;

    use rp_pico::hal::gpio::{bank0::*, Interrupt};
    use rp_pico::hal::gpio::{PushPull, PullDownInput, Output};
    use rp_pico::hal::{self, prelude::*, Watchdog, gpio::pin::Pin};
    use rtic::{Mutex};

    const RUN_DELAY_DEFAULT_MS: u32 = 15;
    const RUN_DELAY_MAX_MS: u32 = 20000;

    #[shared]
    struct Shared {
        rolling: AtomicBool,
        display_toggle_pin: Pin<Gpio2, Output<PushPull>>,
        button_pin: Pin<Gpio15, PullDownInput>,
        roulette_delay_ms: AtomicU32,
        delay: cortex_m::delay::Delay,
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
        let delay = cortex_m::delay::Delay::new(c.core.SYST, clocks.system_clock.freq().integer());

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

        let button_pin = pins.gpio15.into_pull_down_input();
        button_pin.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);
        let display_toggle_pin = pins.gpio2.into_push_pull_output();

        (Shared { rolling: AtomicBool::new(false), display_toggle_pin, button_pin, roulette_delay_ms: AtomicU32::new(RUN_DELAY_DEFAULT_MS), delay }, Local {led_pins}, init::Monotonics())
    }

    /// Executed after the init-task. Replaces the entry task
    #[idle(local = [led_pins], shared = [rolling, display_toggle_pin, roulette_delay_ms, delay])]
    fn idle(context: idle::Context) -> ! {
        let shared = context.shared;
        let local = context.local;

        let led_pins = local.led_pins;
        let mut led_pins_arr: [&mut dyn OutputPin<Error = Infallible>; 10] = [
            &mut led_pins.0, &mut led_pins.1, &mut led_pins.2, &mut led_pins.3, &mut led_pins.4,
            &mut led_pins.5, &mut led_pins.6, &mut led_pins.7, &mut led_pins.8, &mut led_pins.9];

        
        let mut delay_mutex = shared.delay;
        let mut rolling_mutex = shared.rolling;
        let mut display_toggle_pin_mutex = shared.display_toggle_pin;
        let mut roulette_delay_ms_mutex = shared.roulette_delay_ms;
        
        let led_count = led_pins_arr.len();

        let mut index = 0;
        let mut out_state = PinState::Low;
        loop {
            
            rolling_mutex.lock(|rolling| {
                if *rolling.get_mut() {
                    let mut current_active_pin = &mut led_pins_arr[index];
                    current_active_pin.set_low().unwrap();
                    index = (index + 1) % led_count; 
                    current_active_pin = &mut led_pins_arr[index];
                    current_active_pin.set_high().unwrap();
                }
            });
            let delay_ms = roulette_delay_ms_mutex.lock(|roulette_delay_ms| {
                *roulette_delay_ms.get_mut()
            });

            delay_mutex.lock(|delay| {
                delay.delay_ms(delay_ms);
            });

            // TODO: remove this once you have proper control over the display
            if index % 10 == 0 {
                display_toggle_pin_mutex.lock(|toggle_pin| toggle_pin.set_state(out_state)).unwrap();
                out_state = if out_state == PinState::Low { PinState::High } else { PinState::Low };
            }
        }
    }

    #[task(binds = IO_IRQ_BANK0, shared = [rolling, button_pin, roulette_delay_ms, delay])]
    fn button_irq_handler(mut context: button_irq_handler::Context) {
        
        let button_pin_mutex = &mut context.shared.button_pin;
        let rolling_mutex = &mut context.shared.rolling;
        button_pin_mutex.lock(|pin| pin.clear_interrupt(Interrupt::EdgeHigh));
        
        let rolling = rolling_mutex.lock(|rolling| {
            *rolling.get_mut()
        });

        // this breaks the atomic processing of rolling. But that's ok here
        if rolling {
            set_roulette_halting(context);
        } else {
            set_roulette_rolling(context);
        }
    }

    fn set_roulette_rolling(context: button_irq_handler::Context) {
        let mut roulette_delay_ms_mutex = context.shared.roulette_delay_ms;
        let mut rolling_mutex = context.shared.rolling;

        roulette_delay_ms_mutex.lock(|roulette_delay_ms| {
            roulette_delay_ms.store(RUN_DELAY_DEFAULT_MS, core::sync::atomic::Ordering::Relaxed);
        });

        rolling_mutex.lock(|rolling| {
            rolling.store(true, core::sync::atomic::Ordering::Relaxed);
        });
    }

    fn set_roulette_halting(context: button_irq_handler::Context) {
        let mut roulette_delay_ms_mutex = context.shared.roulette_delay_ms;
        let mut rolling_mutex = context.shared.rolling;
        let mut delay_mutex = context.shared.delay;

        let roulette_delay_ms_start = roulette_delay_ms_mutex.lock(|roulette_delay_ms| {
            *roulette_delay_ms.get_mut()
        });
        for next_delay_ms in (roulette_delay_ms_start..RUN_DELAY_MAX_MS).step_by(100) {
            roulette_delay_ms_mutex.lock(|roulette_delay_ms| {
                roulette_delay_ms.store(next_delay_ms, core::sync::atomic::Ordering::Relaxed);
            });

            delay_mutex.lock(|delay| {
                delay.delay_ms(100);
            })
        }
    
        rolling_mutex.lock(|rolling|{
            rolling.store(false, core::sync::atomic::Ordering::Relaxed);
        });
    }

}
// End of file
