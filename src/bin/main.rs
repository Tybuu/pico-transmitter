#![no_std]
#![no_main]

use core::ops::Deref;
use core::slice;
use core::sync::atomic::{AtomicBool, Ordering};

use byte_slice_cast::AsByteSlice;
use defmt::info;
use embassy_executor::Spawner;
use embassy_futures::join::{join, join4};
use embassy_futures::select::select;
use embassy_futures::yield_now;
use embassy_rp::adc::{self, Adc, Config as AdcConfig};
use embassy_rp::dma::Channel;
use embassy_rp::gpio::{Input, Pin, Pull};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{Config, Direction, Pio};
use embassy_rp::pwm::{Pwm, SetDutyCycle};
use embassy_rp::{
    bind_interrupts, dma, gpio, into_ref, pac, peripherals, pwm, usb, Peripheral, PeripheralRef,
};
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
const HIGH: u32 = 0xF0F0F0F0;
const LOW: u32 = 0xFF00FF00;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Device Started!");
    let p = embassy_rp::init(Default::default());

    let mut led = Output::new(p.PIN_25, Level::High);
    let gpio_ref = pac::SIO.gpio_out(15).value_xor().as_ptr();
    let mut dma0 = PeripheralRef::new(p.DMA_CH0);

    let mut inputs = [
        Input::new(p.PIN_17.degrade(), Pull::Down),
        Input::new(p.PIN_16.degrade(), Pull::Down),
    ];
    let mut states = [Debouncer::default(); 2];

    let mut report = [false, false];
    // let mut bits: [u32; 13] = [
    //     HIGH, HIGH, HIGH, LOW, LOW, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, LOW,
    // ];
    let mut bits = [HIGH; 256];
    loop {
        let timer = pac::DMA.timer(0).modify(|w| {
            w.set_x(8);
            w.set_y(1250);
        });
        // unsafe {
        //     dma::write(
        //         dma0.reborrow(),
        //         &bits,
        //         gpio_ref,
        //         pac::dma::vals::TreqSel::TIMER0,
        //     );
        // }
    }
}
