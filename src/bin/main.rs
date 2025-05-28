//! This example test the RP Pico on board LED.
//!
//! It does not work with the RP Pico W board. See wifi_blinky.rs.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::info;
use embassy_executor::Spawner;
use embassy_futures::join::{join, join4};
use embassy_futures::select::select;
use embassy_futures::yield_now;
use embassy_rp::adc::{self, Adc, Channel, Config as AdcConfig};
use embassy_rp::gpio::{Input, Pin, Pull};
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{Config, Direction, Pio};
use embassy_rp::pwm::{Pwm, SetDutyCycle};
use embassy_rp::{bind_interrupts, gpio, peripherals, pwm, usb, Peripheral};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Ticker, Timer};
use gpio::{Level, Output};
use keyboard::debounce::Debouncer;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
});
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Device Started!");
    let p = embassy_rp::init(Default::default());

    let mut led = Output::new(p.PIN_25, Level::High);
    // let mut c_high = pwm::Config::default();
    // c_high.top = 1249;
    // c_high.compare_b = 625;
    //
    // let mut c_low = pwm::Config::default();
    // c_low.top = 2500;
    // c_low.compare_b = 1250;

    let prog = pio_asm!(
        "start:",
        "  pull block", // Pull 32 bits from TX FIFO, block if empty
        "  set x, 13",   // Set X to 6 (bit count)
        "cond:",
        "  jmp x--, loop", // If X != 0, jump to loop and decrement X
        "  jmp start",     // Else restart
        "loop:",
        "  out y, 1",    // Output 1 bit to Y
        "  jmp !y low", // If Y == 0, jump to low
        "  set y, 10", // Y = 19 (high frequency loop count)
        "high_loop:",
        "  set pins, 1 [31]", // Pin high with 31 cycle delay
        "  nop [31]",
        "  set pins, 0 [31]", // Pin low with 31 cycle delay
        "  nop [31]",
        "  jmp y-- high_loop", // Loop Y times
        "  jmp cond", // Go back to cond with delay
        "low:"
        "  set y, 6",   // Else set Y = 13 (low frequency loop count)
        "low_loop:",
        "  set pins, 1 [31]", // Pin high with 31 cycle delay
        "  nop [31]",
        "  nop [31]",
        "  nop [11]",
        "  set pins, 0 [31]", // Pin low with 31 cycle delay
        "  nop [31]",
        "  nop [31]",
        "  nop [11]",
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

    let mut inputs = [
        Input::new(p.PIN_17.degrade(), Pull::Down),
        Input::new(p.PIN_16.degrade(), Pull::Down),
    ];
    let mut states = [Debouncer::default(); 2];
    let mut bits: u32 = 0b0000000100111;
    loop {
        let (i0, i1) = inputs.split_at_mut(1);
        // select(i0[0].wait_for_high(), i1[0].wait_for_high()).await;
        loop {
            let mut current_state = bits;
            states[0].update_buf(inputs[0].get_level().into());
            states[1].update_buf(inputs[1].get_level().into());
            set_bit(&mut current_state, 7, states[0].is_pressed());
            set_bit(&mut current_state, 8, states[0].is_pressed());
            set_bit(&mut current_state, 9, states[0].is_pressed());
            set_bit(&mut current_state, 10, states[1].is_pressed());
            set_bit(&mut current_state, 11, states[1].is_pressed());
            set_bit(&mut current_state, 12, states[1].is_pressed());
            if current_state != bits {
                info!("State: {:013b}", current_state);
                bits = current_state;
                pio.sm0.tx().wait_push(bits).await;
            }
            // CHANNEL.send([true, true]).await;
            // if states[0].is_pressed() {
            //     CHANNEL.send([true, true]).await;
            // }
            // if states[1].is_pressed() {
            //     CHANNEL.send([false, false]).await;
            // }
            if !states[0].is_pressed() && !states[1].is_pressed() {
                break;
            }
        }
    }
}

fn set_bit(num: &mut u32, pos: usize, state: bool) {
    if state {
        *num |= 1 << pos;
    } else {
        *num &= !(1 << pos);
    }
}
