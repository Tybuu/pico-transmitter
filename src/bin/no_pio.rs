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

#[cortex_m_rt::entry]
fn main() -> ! {
    info!("Device Started!");

    let mut c_high = pwm::Config::default();
    c_high.top = 1249;
    c_high.compare_b = 625;
    let p = embassy_rp::init(Default::default());
    let _ = Output::new(p.PIN_25, Level::High);
    // let mut pwm = Pwm::new_output_b(p.PWM_SLICE7, p.PIN_15, c_high);
    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| {
                (spawner.spawn(core1_task(
                    PeripheralRef::new(p.PWM_SLICE7),
                    PeripheralRef::new(p.PIN_15),
                )))
                .unwrap()
            });
        },
    );

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        (spawner.spawn(core0_task(
            PeripheralRef::new(p.PIN_17),
            PeripheralRef::new(p.PIN_16),
        )))
        .unwrap()
    });
}
#[embassy_executor::task]
async fn core0_task(p0: PeripheralRef<'static, PIN_17>, p1: PeripheralRef<'static, PIN_16>) {
    info!("Hello from core 0");

    let mut inputs = [Input::new(p0, Pull::Down), Input::new(p1, Pull::Down)];
    let mut states = [Debouncer::default(); 2];
    let mut bool_states = [false, false];
    loop {
        let (i0, i1) = inputs.split_at_mut(1);
        // select(i0[0].wait_for_high(), i1[0].wait_for_high()).await;
        loop {
            let mut current_state = bool_states;
            states[0].update_buf(inputs[0].get_level().into());
            states[1].update_buf(inputs[1].get_level().into());
            current_state[0] = states[0].is_pressed();
            current_state[1] = states[1].is_pressed();
            if current_state != bool_states {
                info!("State: {}", current_state);
                bool_states = current_state;
                CHANNEL.send(bool_states).await;
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

#[embassy_executor::task]
async fn core1_task(
    slice: PeripheralRef<'static, PWM_SLICE7>,
    pin: PeripheralRef<'static, PIN_15>,
) {
    info!("Hello from core 1");
    let mut c_high = pwm::Config::default();
    c_high.top = 1249;
    c_high.compare_b = 625;

    let p = pac::PWM.ch(slice.number());
    let mut c_low = pwm::Config::default();
    c_low.top = 2500;
    c_low.compare_b = 1250;
    let mut pwm = Pwm::new_output_b(slice, pin, c_low.clone());
    pwm.set_duty_cycle_fully_off().unwrap();
    let mut bits: [bool; 13] = [
        true, true, true, false, false, true, false, false, false, false, false, false, false,
    ];
    const BIT_PERIOD: u64 = 250;
    let mut ticker = Ticker::every(Duration::from_micros(BIT_PERIOD));
    let mut last_pressed = Instant::now();
    loop {
        if last_pressed.elapsed() > Duration::from_micros(BIT_PERIOD * 26) {
            let res = CHANNEL.receive().await;
            bits[7] = res[0];
            bits[8] = res[0];
            bits[9] = res[0];
            bits[10] = res[1];
            bits[11] = res[1];
            bits[12] = res[1];
            info!("Recevied: {}", res);
            info!("transmitting {}", bits);
            ticker.reset();
            // if res[0] {
            //     bits = [true; 10];
            // } else {
            //     bits = [false; 10];
            // }
            for bit in bits {
                if bit {
                    p.cc().write(|w| {
                        w.set_b(625);
                    });
                    p.top().write(|w| w.set_top(1249));
                    // pwm.set_config(&c_high);
                } else {
                    p.cc().write(|w| {
                        w.set_b(1249);
                    });
                    p.top().write(|w| w.set_top(2500));
                    // pwm.set_config(&c_low);
                }
                p.ctr().write(|w| w.set_ctr(0));
                ticker.next().await;
            }
            pwm.set_duty_cycle_fully_off().unwrap();
            last_pressed = Instant::now();
        }
    }
}
