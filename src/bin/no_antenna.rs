//! It does not work with the RP Pico W board. See wifi_blinky.rs.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::info;
use embassy_executor::{Executor, Spawner};
use embassy_futures::join::{join, join4};
use embassy_futures::select::select;
use embassy_futures::yield_now;
use embassy_rp::gpio::{Input, Level, Output, Pin, Pull};
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_rp::pac::PWM;
use embassy_rp::peripherals::{PIN_15, PIN_16, PIN_17, PIO0, PWM_SLICE7};
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{Config, Direction, Pio};
use embassy_rp::pwm::{self, Slice};
use embassy_rp::pwm::{Pwm, SetDutyCycle};
use embassy_rp::{pac, PeripheralRef};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Ticker, Timer};
use keyboard::debounce::Debouncer;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();
static CHANNEL: Channel<CriticalSectionRawMutex, [bool; 2], 5> = Channel::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Device Started!");

    let mut c_high = pwm::Config::default();
    c_high.top = 1249;
    c_high.compare_b = 625;
    let p = embassy_rp::init(Default::default());
    let _ = Output::new(p.PIN_25, Level::High);
    //
    let mut c_high = pwm::Config::default();
    c_high.top = 1249;
    c_high.compare_b = 625;

    let mut c_low = pwm::Config::default();
    c_low.top = 2500;
    c_low.compare_b = 1250;

    let mut pwm = Pwm::new_output_b(p.PWM_SLICE7, p.PIN_15, c_high);
    let mut pwm2 = Pwm::new_output_b(p.PWM_SLICE6, p.PIN_13, c_low);

    let en = PWM.en().as_ptr() as *mut u32;
    loop {
        unsafe { en.write_volatile(1 << 6) };
        Timer::after_millis(500).await;
        unsafe { en.write_volatile(1 << 7) };
        Timer::after_millis(500).await;
    }
}
