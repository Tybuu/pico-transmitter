//! This example test the RP Pico on board LED.
//!
//! It does not work with the RP Pico W board. See wifi_blinky.rs.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::info;
use embassy_executor::Spawner;
use embassy_futures::join::{join, join4};
use embassy_futures::yield_now;
use embassy_rp::adc::{self, Adc, Channel, Config as AdcConfig};
use embassy_rp::gpio::{Input, Pull};
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{Config, Direction, Pio};
use embassy_rp::pwm::{Pwm, SetDutyCycle};
use embassy_rp::{bind_interrupts, gpio, peripherals, pwm, usb, Peripheral};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Ticker, Timer};
use gpio::{Level, Output};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
});
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Device Started!");
    let p = embassy_rp::init(Default::default());

    let mut led = Output::new(p.PIN_25, Level::High);
    let mut c_high = pwm::Config::default();
    c_high.top = 1249;
    c_high.compare_b = 625;

    let mut c_low = pwm::Config::default();
    c_low.top = 2500;
    c_low.compare_b = 1250;

    let prog = pio_asm!(
        "start:",
        "  pull block", // Pull 32 bits from TX FIFO, block if empty
        "  set x, 6",   // Set X to 6 (bit count)
        "cond:",
        "  jmp x--, loop", // If X != 0, jump to loop and decrement X
        "  jmp start",     // Else restart
        "loop:",
        "  out y, 1",    // Output 1 bit to Y
        "  jmp !y low", // If Y == 0, jump to low
        "  set y, 19", // Y = 19 (high frequency loop count)
        "high_loop:",
        "  set pins, 1 [31]", // Pin high with 31 cycle delay
        "  nop [31]",
        "  set pins, 0 [31]", // Pin low with 31 cycle delay
        "  nop [31]",
        "  jmp y-- high_loop", // Loop Y times
        "  nop [31]",
        "  jmp cond [26]", // Go back to cond with delay
        "low:"
        "  set y, 13",   // Else set Y = 13 (low frequency loop count)
        "low_loop:",
        "  set pins, 1 [31]", // Pin high with 31 cycle delay
        "  nop [31]",
        "  nop [31]",
        "  set pins, 0 [31]", // Pin low with 31 cycle delay
        "  nop [31]",
        "  nop [31]",
        "  jmp y-- low_loop", // Loop Y times
        "  jmp cond",         // Go back to cond
    );
    let mut pio = Pio::new(p.PIO0, Irqs);
    let load = pio.common.load_program(&prog.program);
    let pin = pio.common.make_pio_pin(p.PIN_15);
    pio.sm0.set_pin_dirs(Direction::Out, &[&pin]);
    let mut cfg = Config::default();
    cfg.use_program(&load, &[]);
    cfg.set_set_pins(&[&pin]);
    pio.sm0.set_config(&cfg);
    pio.sm0.set_enable(true);
    // let mut pwm = Pwm::new_output_b(p.PWM_SLICE7, p.PIN_15, c_low.clone());

    let bit_sequecne = [true, false, true, true, false, true, false, true];
    let bits = 0b0000000u32;
    loop {
        // let mut pin = unsafe { p.PIN_15.clone_unchecked() };
        // let res = Input::new(&mut pin, Pull::None);
        // Timer::after_micros(5).await;
        // let res = Output::new(pin, Level::Low);
        // Timer::after_micros(5).await;
        pio.sm0.tx().wait_push(bits).await;
        // for bit in bit_sequecne {
        //     if bit {
        //         pwm.set_config(&c_high);
        //     } else {
        //         pwm.set_config(&c_low);
        //     }
        //     tick.next().await;
        // }
    }
}
