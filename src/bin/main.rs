#![no_std]
#![no_main]

use core::slice;
use core::sync::atomic::{AtomicBool, Ordering};

use byte_slice_cast::AsByteSlice;
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
use embassy_rp::{bind_interrupts, gpio, peripherals, pwm, usb, Peripheral, PeripheralRef};
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

    let mut dma0 = PeripheralRef::new(p.DMA_CH0);

    let prog = pio_asm!(".wrap_target", "out pins, 1 ", ".wrap");
    let mut pio = Pio::new(p.PIO0, Irqs);
    let mut load = pio.common.load_program(&prog.program);
    // load.wrap.target = 1;
    // load.wrap.source = 1;
    let pin = pio.common.make_pio_pin(p.PIN_15);
    pio.sm0.set_pin_dirs(Direction::Out, &[&pin]);
    pio.sm0.set_pins(Level::High, &[&pin]);
    let mut cfg = Config::default();
    cfg.use_program(&load, &[]);
    cfg.set_out_pins(&[&pin]);
    cfg.shift_out.auto_fill = true;
    cfg.shift_out.threshold = 0;
    pio.sm0.set_config(&cfg);
    pio.sm0.set_enable(true);

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
        // pio.sm0.tx().wait_push().await;
        pio.sm0.tx().dma_push(dma0.reborrow(), &bits, true).await;
        // let (i0, i1) = inputs.split_at_mut(1);
        // select(i0[0].wait_for_high(), i1[0].wait_for_high()).await;
        // loop {
        //     let mut current_state = report;
        //     states[0].update_buf(inputs[0].get_level().into());
        //     states[1].update_buf(inputs[1].get_level().into());
        //     current_state[0] = states[0].is_pressed();
        //     current_state[1] = states[1].is_pressed();
        //
        //     if current_state != report {
        //         info!("State: {:013b}", current_state);
        //         report = current_state;
        //         bits[7] = if report[0] { HIGH } else { LOW };
        //         bits[8] = if report[0] { HIGH } else { LOW };
        //         bits[9] = if report[0] { HIGH } else { LOW };
        //         bits[10] = if report[1] { HIGH } else { LOW };
        //         bits[11] = if report[1] { HIGH } else { LOW };
        //         bits[12] = if report[1] { HIGH } else { LOW };
        //         pio.sm0.tx().dma_push(dma0.reborrow(), &bits, false).await;
        //     }
        //
        //     if !states[0].is_pressed() && !states[1].is_pressed() {
        //         break;
        //     }
        // }
    }
}
