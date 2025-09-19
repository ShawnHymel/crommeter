//! This example shows receiving audio from a connected I2S microphone (or other audio source)
//! using the PIO module of the RP235x.
//!
//!
//! Connect the i2s microphone as follows:
//!   bclk : GPIO 18
//!   lrc  : GPIO 19
//!   din  : GPIO 20
//! Then hold down the boot select button to begin receiving audio. Received I2S words will be written to
//! buffers for the left and right channels for use in your application, whether that's storage or
//! further processing
//!
//!  Note the const USE_ONBOARD_PULLDOWN is by default set to false, meaning an external
//!  pull-down resistor is being used on the data pin if required by the mic being used.
//! 
//! From: https://github.com/embassy-rs/embassy/blob/main/examples/rp235x/src/bin/pio_i2s_rx.rs   

#![no_std]
#![no_main]

use core::mem;

use embassy_executor::Spawner;

use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::PIO0;
use embassy_rp::peripherals::USB;
use embassy_rp::pio::InterruptHandler as PioInterruptHandler;
use embassy_rp::pio::Pio;
use embassy_rp::pio_programs::i2s::{PioI2sIn, PioI2sInProgram};
use embassy_rp::usb::Driver;
use embassy_rp::usb::InterruptHandler as UsbInterruptHandler;

use static_cell::StaticCell;

use panic_probe as _;

// Settings
const LOG_LEVEL: log::LevelFilter = log::LevelFilter::Debug;
const SAMPLE_RATE: u32 = 48_000;
const BIT_DEPTH: u32 = 16;
const CHANNELS: u32 = 2;
const USE_ONBOARD_PULLDOWN: bool = false;
const BUFFER_SIZE: usize = 1024;

// Globals
static DMA_BUFFER: StaticCell<[u32; BUFFER_SIZE * 2]> = StaticCell::new();

// Bind PIO interrupt handler
bind_interrupts!(struct PioIrqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

// Bind USB interrupt handler
bind_interrupts!(struct UsbIrqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

// Task: handle USB logging
#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, LOG_LEVEL, driver);
}

// Async function to handle audio buffer half when ready
async fn handle_audio_buffer(buf: &[u32]) {
    log::debug!("{:?}", &buf[0..100]);
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize embassy HAL
    let p = embassy_rp::init(Default::default());

    // Initialize USB driver and task
    let usb_driver = Driver::new(p.USB, UsbIrqs);
    let _ = spawner.spawn(logger_task(usb_driver).unwrap());

    // Initialize the audio buffer
    let dma_buffer = DMA_BUFFER.init_with(|| [0u32; BUFFER_SIZE * 2]);
    let (mut buffer_a, mut buffer_b) = dma_buffer.split_at_mut(BUFFER_SIZE);

    // Setup pio state machine for i2s input
    let Pio { mut common, sm0, .. } = Pio::new(p.PIO0, PioIrqs);

    // Configure I2S pins
    let bit_clock_pin = p.PIN_18;
    let left_right_clock_pin = p.PIN_19; // AKA "WS (word select)"
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
        BIT_DEPTH,
        CHANNELS,
        &program,
    );

    loop {
        // Start DMA read into buffer_a
        let dma_future = i2s.read(buffer_a);
        
        // Process buffer_b using raw pointer to avoid borrow checker
        let buffer_b_ptr = buffer_b.as_ptr();
        let buffer_b_len = buffer_b.len();
        let process_future = async {
            unsafe {
                let slice = core::slice::from_raw_parts(buffer_b_ptr, buffer_b_len);
                handle_audio_buffer(slice).await;
            }
        };
        
        // Use join to wait for both - this consumes both futures
        embassy_futures::join::join(dma_future, process_future).await;
        
        // Swap buffers for next iteration
        mem::swap(&mut buffer_a, &mut buffer_b);
    }
}