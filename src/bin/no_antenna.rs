//! It does not work with the RP Pico W board. See wifi_blinky.rs.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::info;
use embassy_executor::{Executor, Spawner};
use embassy_futures::join::{join, join4};
use embassy_futures::select::select;
use embassy_futures::yield_now;
use embassy_rp::clocks::{clk_sys_freq, ClockConfig};
use embassy_rp::config::Config;
use embassy_rp::gpio::{Input, Level, Output, Pin, Pull};
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_rp::pac::{PADS_BANK0, PWM};
use embassy_rp::peripherals::{DMA_CH0, PIN_15, PIN_16, PIN_17, PIO0, PWM_SLICE7};
use embassy_rp::pwm::{self, Slice};
use embassy_rp::pwm::{Pwm, SetDutyCycle};
use embassy_rp::{clocks, dma, pac, Peri};
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

const HIGH: u32 = 1 << 4;
const LOW: u32 = 2 << 4;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Device Started!");
    // let clk_cfg = ClockConfig::system_freq(200_000_000).unwrap();
    // let p = embassy_rp::init(Config::new(clk_cfg));
    let p = embassy_rp::init(Default::default());
    let _ = Output::new(p.PIN_25, Level::High);

    let dma0 = p.DMA_CH0;
    let mut c_high = pwm::Config::default();
    c_high.top = 207;
    c_high.compare_b = 104;

    // Make the transitions between low and high sharper to have stronger signals
    // PADS_BANK0.gpio(15).modify(|x| {
    //     x.set_drive(pac::pads::vals::Drive::_12M_A);
    // });

    let high_pwm = Pwm::new_output_b(p.PWM_SLICE7, p.PIN_15, c_high);
    let reg = PWM.ch(7).div().as_ptr() as *mut u32;
    loop {
        loop {
            unsafe {
                reg.write(HIGH);
            }
            Timer::after_millis(500).await;
            unsafe {
                reg.write(LOW);
            }
            Timer::after_millis(500).await;
        }
    }
}

struct Transmitter {
    dma: Peri<'static, DMA_CH0>,
    reg: *mut u32,
}

impl Transmitter {
    fn new(dma: Peri<'static, DMA_CH0>, reg: *mut u32) -> Self {
        // Disable all pwm slices
        unsafe {
            reg.write_volatile(0);
        }
        // Set the DMA timer to trigger every 100 us
        pac::DMA.timer(0).modify(|w| {
            w.set_x(1);
            w.set_y(12500);
        });
        Self { dma, reg }
    }
    async fn transmit_data(&mut self, data: &[u32]) {
        // Transfer data to the PWM enable register to change which slices are active
        // to have certain frequencies at a bit period
        unsafe {
            dma::write(
                self.dma.reborrow(),
                data,
                self.reg,
                pac::dma::vals::TreqSel::TIMER0,
            )
            .await;
        }

        // Disable all pwm slices
        unsafe {
            self.reg.write_volatile(0);
        }
    }

    async fn transmit_data_timed(&mut self, data: &[u32], dur: Duration) {
        for bit in data {
            unsafe {
                self.reg.write_volatile(*bit);
            }
            Timer::after(dur).await;
        }
        unsafe {
            self.reg.write_volatile(0);
        }
    }
}
