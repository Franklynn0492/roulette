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
    const RUN_DELAY_MAX_MS: u32 = 1500;

    type RoulettePins = (
        Pin<Gpio21, Output<PushPull>>, Pin<Gpio22, Output<PushPull>>,
        Pin<Gpio26, Output<PushPull>>, Pin<Gpio27, Output<PushPull>>,
        Pin<Gpio28, Output<PushPull>>, Pin<Gpio16, Output<PushPull>>,
        Pin<Gpio17, Output<PushPull>>, Pin<Gpio18, Output<PushPull>>,
        Pin<Gpio19, Output<PushPull>>, Pin<Gpio20, Output<PushPull>>,
    );

    pub struct Roulette {
        current_index: u8,
        led_pins: RoulettePins,
    }

    impl Roulette {
        pub fn new(mut led_pins: RoulettePins) -> Roulette {

            let mut roulette = Roulette{current_index: 0, led_pins};

            roulette.set_pin_state(0, true);
            for i in 1..9 {
                roulette.set_pin_state(i, false);
            }


            roulette
        }

        pub fn shift(&mut self) {
            let led_count = 10;
            self.set_pin_state(self.current_index, false);
            self.current_index = (self.current_index + 1) % led_count;
            self.set_pin_state(self.current_index, true);
        }

        fn set_pin_state(&mut self, index: u8, state: bool) {
/*
            let mut led_pins_arr: [&mut dyn OutputPin<Error = Infallible>; 10] = [
                &mut led_pins.0, &mut led_pins.1, &mut led_pins.2, &mut led_pins.3, &mut led_pins.4,
                &mut led_pins.5, &mut led_pins.6, &mut led_pins.7, &mut led_pins.8, &mut led_pins.9];*/
            let state = if state { PinState::High} else {PinState::Low};
            match index % 10 {
                0 => self.led_pins.0.set_state(state).unwrap(),
                1 => self.led_pins.1.set_state(state).unwrap(),
                2 => self.led_pins.2.set_state(state).unwrap(),
                3 => self.led_pins.3.set_state(state).unwrap(),
                4 => self.led_pins.4.set_state(state).unwrap(),
                5 => self.led_pins.5.set_state(state).unwrap(),
                6 => self.led_pins.6.set_state(state).unwrap(),
                7 => self.led_pins.7.set_state(state).unwrap(),
                8 => self.led_pins.8.set_state(state).unwrap(),
                9 => self.led_pins.9.set_state(state).unwrap(),
                _ => self.led_pins.0.set_state(state).unwrap()
            };
        }
    }

    #[shared]
    struct Shared {
        rolling: AtomicBool,
        display_toggle_pin: Pin<Gpio2, Output<PushPull>>,
        button_pin: Pin<Gpio15, PullDownInput>,
        delay: cortex_m::delay::Delay,
        roulette: Roulette,
    }


    /// See https://rtic.rs/1/book/en/by-example/resources.html
    /// In my own words: local resources are initialized by the init-task and can only be accessed by a single task
    #[local]
    struct Local {
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

        let led_pins: RoulettePins = (
            pins.gpio21.into_push_pull_output(), pins.gpio22.into_push_pull_output(),
            pins.gpio26.into_push_pull_output(), pins.gpio27.into_push_pull_output(),
            pins.gpio28.into_push_pull_output(), pins.gpio16.into_push_pull_output(),
            pins.gpio17.into_push_pull_output(), pins.gpio18.into_push_pull_output(),
            pins.gpio19.into_push_pull_output(), pins.gpio20.into_push_pull_output(),
        );        

        let button_pin = pins.gpio15.into_pull_down_input();
        button_pin.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);
        let display_toggle_pin = pins.gpio2.into_push_pull_output();

        (Shared { rolling: AtomicBool::new(false), display_toggle_pin, button_pin, delay, roulette: Roulette::new(led_pins) }, Local {}, init::Monotonics())
    }

    /// Executed after the init-task. Replaces the entry task
    #[idle(local = [], shared = [rolling, display_toggle_pin, delay, roulette])]
    fn idle(context: idle::Context) -> ! {
        let shared = context.shared;
        
        let mut delay_mutex = shared.delay;
        let mut rolling_mutex = shared.rolling;
        let mut roulette_mutex = shared.roulette;
        let mut display_toggle_pin_mutex = shared.display_toggle_pin;
        

        loop {
            
            let rolling = rolling_mutex.lock(|rolling| { *rolling.get_mut()});
            if rolling {
                roulette_mutex.lock(|roulette| {
                    roulette.shift();
                });
            }

            delay_mutex.lock(|delay| {
                delay.delay_ms(RUN_DELAY_DEFAULT_MS);
            });

            // TODO: remove this once you have proper control over the display
            /* 
            if index % 10 == 0 {
                display_toggle_pin_mutex.lock(|toggle_pin| toggle_pin.set_state(out_state)).unwrap();
                out_state = if out_state == PinState::Low { PinState::High } else { PinState::Low };
            }*/
        }
    }

    #[task(binds = IO_IRQ_BANK0, shared = [rolling, button_pin, delay, roulette])]
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
        let mut rolling_mutex = context.shared.rolling;

        rolling_mutex.lock(|rolling| {
            rolling.store(true, core::sync::atomic::Ordering::Relaxed);
        });
    }

    fn set_roulette_halting(context: button_irq_handler::Context) {
        let mut rolling_mutex = context.shared.rolling;
        let mut delay_mutex = context.shared.delay;
        let mut roulette_mutex = context.shared.roulette;
        roulette_mutex.lock(|roulette| {

            let mut next_delay_ms = RUN_DELAY_DEFAULT_MS;
            while next_delay_ms < RUN_DELAY_MAX_MS {

                roulette.shift();
                delay_mutex.lock(|delay| {
                    delay.delay_ms(next_delay_ms);
                });
                next_delay_ms = (next_delay_ms as f32 * 1.3) as u32;
            }
        });
    
        rolling_mutex.lock(|rolling|{
            rolling.store(false, core::sync::atomic::Ordering::Relaxed);
        });
    }

}
// End of file
