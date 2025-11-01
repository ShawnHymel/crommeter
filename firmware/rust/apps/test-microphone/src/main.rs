//! Simplest solution - just signal when buffer is ready

#![no_std]
#![no_main]

use core::mem;

use embassy_executor::Spawner;

use embassy_sync::signal::Signal;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use embassy_rp::bind_interrupts;
use embassy_rp::gpio;
use embassy_rp::peripherals::PIO0;
use embassy_rp::peripherals::USB;
use embassy_rp::pio::Pio;
use embassy_rp::pio::InterruptHandler as PioInterruptHandler;
use embassy_rp::usb::Driver;
use embassy_rp::usb::InterruptHandler as UsbInterruptHandler;

use pio_i2s::{PioI2sIn, PioI2sInProgram};

use static_cell::StaticCell;
use panic_probe as _;

// Settings
const LOG_LEVEL: log::LevelFilter = log::LevelFilter::Debug;
const SAMPLE_RATE: u32 = 48_000;
const CHANNELS: u32 = 2;
const USE_ONBOARD_PULLDOWN: bool = false;

// Circular buffer - MUST be power of 2!
const CIRCULAR_BUFFER_SIZE: usize = 1024;
static CIRCULAR_BUFFER: StaticCell<[u32; CIRCULAR_BUFFER_SIZE]> = StaticCell::new();

// Bind interrupts
bind_interrupts!(struct PioIrqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

bind_interrupts!(struct UsbIrqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

// Task: handle USB logging
#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, LOG_LEVEL, driver);
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Initialize USB driver and task
    let usb_driver = Driver::new(p.USB, UsbIrqs);
    spawner.spawn(logger_task(usb_driver).expect("Failed to spawn logger task"));

    // Initialize LED
    let mut led_pin = gpio::Output::new(p.PIN_15, gpio::Level::Low);

    // Setup pio state machine for i2s input
    let Pio { mut common, sm0, .. } = Pio::new(p.PIO0, PioIrqs);

    // Configure I2S pins
    let bit_clock_pin = p.PIN_18;
    let left_right_clock_pin = p.PIN_19;
    let data_pin = p.PIN_20;

    // Initialize PIO I2S program
    let program = PioI2sInProgram::new(&mut common);
    let mut i2s = PioI2sIn::new(
        &mut common,
        sm0,
        p.DMA_CH0,
        USE_ONBOARD_PULLDOWN,
        data_pin,
        bit_clock_pin,
        left_right_clock_pin,
        SAMPLE_RATE,
        // BIT_DEPTH,
        CHANNELS,
        &program,
    );

    // TEST: let serial connect
    embassy_time::Timer::after_secs(5).await;

    // Test: Get DMA channel
    let ch = i2s.dma_channel();
    log::info!("DMA channel: {}", ch);

    // Test: Initialize circular buffer
    let circular_buf = CIRCULAR_BUFFER.init([0u32; CIRCULAR_BUFFER_SIZE]);
    
    // Test: Call start_circular (should just log messages)
    log::info!("Calling start_circular...");
    unsafe {
        i2s.start_circular(circular_buf);
    }
    log::info!("start_circular completed!");

    // Done
    loop {
        embassy_time::Timer::after_secs(10).await;
    }
}