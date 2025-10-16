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
const BIT_DEPTH: u32 = 32;
const CHANNELS: u32 = 2;
const USE_ONBOARD_PULLDOWN: bool = false;
const BUFFER_SIZE: usize = 2048;

// Globals
static DMA_BUFFER: StaticCell<[u32; BUFFER_SIZE * 2]> = StaticCell::new();
static BUFFER_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();

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

// Task: handle audio processing - just blink LED when buffer ready
#[embassy_executor::task]
async fn audio_processor_task(mut led_pin: gpio::Output<'static>) {
    loop {
        BUFFER_READY.wait().await;
        
        // Toggle LED to show a buffer was processed
        led_pin.toggle();
        
        // Add any other processing here
        // The actual audio data is in the global DMA_BUFFER if you need it
        
        // TODO: figure out how to get the global buffer
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Initialize USB driver and task
    let usb_driver = Driver::new(p.USB, UsbIrqs);
    spawner.spawn(logger_task(usb_driver).expect("Failed to spawn logger task"));

    // Initialize LED
    let led_pin = gpio::Output::new(p.PIN_15, gpio::Level::Low);

    // Spawn audio processing task
    spawner.spawn(audio_processor_task(led_pin).expect("Failed to spawn audio processor task"));

    // Initialize the audio buffer
    let dma_buffer = DMA_BUFFER.init_with(|| [0u32; BUFFER_SIZE * 2]);
    let (mut buffer_a, mut buffer_b) = dma_buffer.split_at_mut(BUFFER_SIZE);

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

    loop {
        // Minimal I2S loop - just read and signal
        i2s.read(buffer_a).await;
        
        // Signal that a buffer is ready (very fast - no data copying)
        BUFFER_READY.signal(());
        
        // Swap buffers for next iteration
        mem::swap(&mut buffer_a, &mut buffer_b);
    }
}