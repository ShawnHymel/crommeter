//! Pio backed I2s output and output drivers
//! 
//! Originally from: https://github.com/embassy-rs/embassy/blob/4b5707b2563c725ce9bcd5537e2637d9bcaae7c6/embassy-rp/src/pio_programs/i2s.rs
//! Modified to work with 64-bit input frames (instead of the default 32-bit frames) and removed
//! output functions.

use fixed::traits::ToFixed;

use embassy_rp::dma::{AnyChannel, Channel};
use embassy_rp::gpio::Pull;
use embassy_rp::pio::{
    Common, Config, Direction, FifoJoin, Instance, LoadedProgram, PioPin, program, ShiftConfig,
    ShiftDirection, StateMachine,
};
use embassy_rp::Peri;
use embassy_rp::dma::Transfer;

/// This struct represents a 32-bit i2s receiver & controller driver program
pub struct PioI2sInProgram<'d, PIO: Instance> {
    prg: LoadedProgram<'d, PIO>,
}

impl<'d, PIO: Instance> PioI2sInProgram<'d, PIO> {
    /// Load the 32-bit input program into the given pio
    pub fn new(common: &mut Common<'d, PIO>) -> Self {
        let prg = program::pio_asm! {
            ".side_set 2",
            "   set x, 30               side 0b01",  // 31 iterations (30 down to 0) = 31 bits
            "left_data:",
            "   in pins, 1              side 0b00",  // read one left-channel bit from SD
            "   jmp x-- left_data       side 0b01",
            "   in pins, 1              side 0b10",  // 32nd bit, ws changes 1 clock before next channel
            "   set x, 30               side 0b11",  // 31 iterations for right channel
            "right_data:",
            "   in pins, 1             side 0b10",   // read one right-channel bit from SD
            "   jmp x-- right_data     side 0b11",
            "   in pins, 1             side 0b00"    // 32nd bit, ws changes 1 clock before next frame
        };
        let prg = common.load_program(&prg.program);
        Self { prg }
    }
}

/// Pio backed 32-bit I2s input driver
pub struct PioI2sIn<'d, P: Instance, const S: usize> {
    dma: Peri<'d, AnyChannel>,
    sm: StateMachine<'d, P, S>,
}

impl<'d, P: Instance, const S: usize> PioI2sIn<'d, P, S> {
    /// Configure a state machine for 32-bit I2S input
    pub fn new(
        common: &mut Common<'d, P>,
        mut sm: StateMachine<'d, P, S>,
        dma: Peri<'d, impl Channel>,
        data_pulldown: bool,
        data_pin: Peri<'d, impl PioPin>,
        bit_clock_pin: Peri<'d, impl PioPin>,
        lr_clock_pin: Peri<'d, impl PioPin>,
        sample_rate: u32,
        _bit_depth: u32,
        channels: u32,
        program: &PioI2sInProgram<'d, P>,
    ) -> Self {
        let mut data_pin = common.make_pio_pin(data_pin);
        if data_pulldown {
            data_pin.set_pull(Pull::Down);
        }
        let bit_clock_pin = common.make_pio_pin(bit_clock_pin);
        let left_right_clock_pin = common.make_pio_pin(lr_clock_pin);

        let cfg = {
            let mut cfg = Config::default();
            cfg.use_program(&program.prg, &[&bit_clock_pin, &left_right_clock_pin]);
            cfg.set_in_pins(&[&data_pin]);
            
            // Force 32-bit operation regardless of bit_depth parameter
            let clock_frequency = sample_rate * 32 * channels;
            cfg.clock_divider = (embassy_rp::clocks::clk_sys_freq() as f64 / clock_frequency as f64 / 2.0).to_fixed();
            
            cfg.shift_in = ShiftConfig {
                threshold: 32,
                direction: ShiftDirection::Left,
                auto_fill: true,
            };
            
            // Join fifos to have more time for DMA transfers
            cfg.fifo_join = FifoJoin::RxOnly;
            cfg
        };

        sm.set_config(&cfg);
        sm.set_pin_dirs(Direction::In, &[&data_pin]);
        sm.set_pin_dirs(Direction::Out, &[&left_right_clock_pin, &bit_clock_pin]);
        sm.set_enable(true);

        Self { 
            dma: dma.into(), 
            sm 
        }
    }

    /// Return a DMA transfer future for reading I2S data
    pub fn read<'b>(&'b mut self, buff: &'b mut [u32]) -> Transfer<'b, AnyChannel> {
        self.sm.rx().dma_pull(self.dma.reborrow(), buff, false)
    }
}