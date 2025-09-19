#![no_std]
#![no_main]

// Embassy: HAL imports
use embassy_rp::bind_interrupts;
use embassy_rp::gpio;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};

// Embassy: main executor
use embassy_executor::Spawner;

// Embassy: timer
use embassy_time::Timer;

// Let panic_probe handle our panic routine
use panic_probe as _;

// Macro to bind USB interrupt handler
bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

// Task: handle USB logging
#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Debug, driver);
}

// Task: blink the LED if the button is pressed
#[embassy_executor::task]
async fn blink_led_task(mut pin: gpio::Output<'static>) {
    loop {
        // Turn on LED
        pin.set_high();
        log::info!("LED is on");
        Timer::after_millis(250).await;

        // Turn off LED
        pin.set_low();
        log::info!("LED is off");
        Timer::after_millis(250).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize embassy HAL
    let p = embassy_rp::init(Default::default());

    // Initialize USB driver and task
    let usb_driver = Driver::new(p.USB, Irqs);
    let _ = spawner.spawn(logger_task(usb_driver));

    // Create a new output pin
    let led_pin = gpio::Output::new(p.PIN_15, gpio::Level::Low);

    // Spawn blink task
    spawner.spawn(blink_led_task(led_pin)).unwrap();

    // Keep main alive
    loop {
        Timer::after_secs(1).await;
    }
}