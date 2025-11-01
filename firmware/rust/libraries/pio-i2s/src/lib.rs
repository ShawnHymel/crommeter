#![no_std]

use fixed::traits::ToFixed;

use embassy_rp::Peri;
use embassy_rp::dma::{AnyChannel, Channel, Transfer};
use embassy_rp::gpio::Pull;
use embassy_rp::pio::{
    Common, Config, Direction, FifoJoin, Instance, LoadedProgram, PioPin, ShiftConfig, ShiftDirection, StateMachine,
};

use rp235x_pac as pac;

// ============================================================================
// I2S Input Program
// ============================================================================

/// I2S receiver program with configurable bit depth
pub struct PioI2sInProgram<'d, PIO: Instance> {
    prg: LoadedProgram<'d, PIO>,
}

// ***CHANGED: hardcoded to 32 bit depth
impl<'d, PIO: Instance> PioI2sInProgram<'d, PIO> {
    pub fn new(common: &mut Common<'d, PIO>) -> Self {
        let prg = pio::pio_asm! {
            ".side_set 2",
            "   set x, 30               side 0b01",
            "left_data:",
            "   in pins, 1              side 0b00", // read one left-channel bit from SD
            "   jmp x-- left_data       side 0b01",
            "   in pins, 1              side 0b10", // ws changes 1 clock before MSB
            "   set x, 30               side 0b11",
            "right_data:",
            "   in pins, 1             side 0b10",
            "   jmp x-- right_data     side 0b11",
            "   in pins, 1             side 0b00" // ws changes 1 clock before ms
        };
        let prg = common.load_program(&prg.program);
        Self { prg }
    }
}

/// Pio backed I2s input driver
pub struct PioI2sIn<'d, P: Instance, const S: usize> {
    dma: Peri<'d, AnyChannel>,
    sm: StateMachine<'d, P, S>,
    dma_channel: u8,
}

impl<'d, P: Instance + 'static, const S: usize> PioI2sIn<'d, P, S> {
    /// Configure a state machine to act as both the controller (provider of SCK and WS) and receiver (of SD) for an I2S signal
    /// ***CHANGED: hardcoded 32 bit depth
    pub fn new(
        common: &mut Common<'d, P>,
        mut sm: StateMachine<'d, P, S>,
        dma: Peri<'d, impl Channel>,
        // Whether or not to use the MCU's internal pull-down resistor, as the
        // Pico 2 is known to have problems with the inbuilt pulldowns, many
        // opt to just use an external pull down resistor to meet requirements of common
        // i2s microphones such as the INMP441
        data_pulldown: bool,
        data_pin: Peri<'d, impl PioPin>,
        bit_clock_pin: Peri<'d, impl PioPin>,
        lr_clock_pin: Peri<'d, impl PioPin>,
        sample_rate: u32,
        channels: u32,
        program: &PioI2sInProgram<'d, P>,
    ) -> Self {
        // Get channel number
        let dma_channel = dma.number();

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
            let clock_frequency = sample_rate * 32 * channels;
            cfg.clock_divider = (embassy_rp::clocks::clk_sys_freq() as f64 / clock_frequency as f64 / 2.).to_fixed();
            cfg.shift_in = ShiftConfig {
                threshold: 32,
                direction: ShiftDirection::Left,
                auto_fill: true,
            };
            // join fifos to have twice the time to start the next dma transfer
            cfg.fifo_join = FifoJoin::RxOnly; // both control signals are sent via side-setting
            cfg
        };
        sm.set_config(&cfg);
        sm.set_pin_dirs(Direction::In, &[&data_pin]);
        sm.set_pin_dirs(Direction::Out, &[&left_right_clock_pin, &bit_clock_pin]);
        sm.set_enable(true);

        Self { dma: dma.into(), sm, dma_channel, }
    }

    /// Return an in-prograss dma transfer future. Awaiting it will guarentee a complete transfer.
    pub fn read<'b>(&'b mut self, buff: &'b mut [u32]) -> Transfer<'b, AnyChannel> {
        self.sm.rx().dma_pull(self.dma.reborrow(), buff, false)
    }

    /// Get the DMA channel number for advanced usage
    pub fn dma_channel(&self) -> u8 {
        self.dma_channel
    }
    
    /// Get mutable access to the state machine for advanced configuration
    pub fn state_machine_mut(&mut self) -> &mut StateMachine<'d, P, S> {
        &mut self.sm
    }

    /// Get the current DMA transfer count
    /// This decrements as the DMA transfer progresses
    pub fn get_dma_count(&self) -> u32 {
        let ch = self.dma_channel as usize;
        unsafe {
            let dma = pac::Peripherals::steal().DMA;
            dma.ch(ch).ch_trans_count().read().bits()
        }
    }

    /// Start continuous circular buffer
    pub unsafe fn start_circular(&mut self, buffer: &'static mut [u32]) {
        let buffer_len = buffer.len();
        let buffer_ptr = buffer.as_mut_ptr();
        
        assert!(buffer_len.is_power_of_two(), "Buffer length must be power of 2");
        
        let ch = self.dma_channel as usize;
        
        log::info!("Step 3c: Getting DMA peripheral...");
        let dma = pac::Peripherals::steal().DMA;
        
        // Calculate PIO FIFO address
        let pio_base = if core::any::TypeId::of::<P>() == core::any::TypeId::of::<embassy_rp::peripherals::PIO0>() {
            0x50200000u32
        } else {
            0x50300000u32
        };
        let read_addr = pio_base + 0x020 + (S as u32 * 4);
        
        // Calculate DREQ
        let dreq_base = if core::any::TypeId::of::<P>() == core::any::TypeId::of::<embassy_rp::peripherals::PIO0>() {
            4u8  // PIO0_RX0
        } else {
            8u8  // PIO1_RX0
        };
        let dreq = dreq_base + (S as u8);
        
        // Calculate ring size (in bytes, as power of 2)
        let ring_size = (buffer_len * 4).trailing_zeros() as u8;
        
        log::info!("Step 3c: Config - buf_len={}, ring_size={}, dreq={}", buffer_len, ring_size, dreq);
        
        // Set addresses and count
        dma.ch(ch).ch_read_addr().write(|w| unsafe { w.bits(read_addr) });
        dma.ch(ch).ch_write_addr().write(|w| unsafe { w.bits(buffer_ptr as u32) });
        dma.ch(ch).ch_trans_count().write(|w| unsafe { w.bits(buffer_len as u32) });
        
        log::info!("Step 3c: Addresses configured");
        
        // Configure and start DMA
        log::info!("Step 3c: Configuring control register...");
        dma.ch(ch).ch_ctrl_trig().write(|w| {
            w.data_size().size_word();   // 32-bit transfers
            w.incr_read().clear_bit();   // Don't increment read (FIFO)
            w.incr_write().set_bit();    // Increment write (buffer)
            w.ring_sel().set_bit();      // Apply ring to write address
            unsafe { w.ring_size().bits(ring_size) };
            unsafe { w.treq_sel().bits(dreq) };
            unsafe { w.chain_to().bits(ch as u8) };  // Chain to self
            w.en().set_bit()             // Enable!
        });
        
        log::info!("Step 3c: Circular DMA STARTED!");
    }
}

// ============================================================================
// Configurable I2S Output Program
// ============================================================================

/// This struct represents an i2s output driver program
///
/// The sample bit-depth is set through scratch register `Y`.
/// `Y` has to be set to sample bit-depth - 2.
/// (14 = 16bit, 22 = 24bit, 30 = 32bit)
pub struct PioI2sOutProgram<'d, PIO: Instance> {
    prg: LoadedProgram<'d, PIO>,
}

impl<'d, PIO: Instance> PioI2sOutProgram<'d, PIO> {
    /// Load the program into the given pio
    pub fn new(common: &mut Common<'d, PIO>) -> Self {
        let prg = pio::pio_asm!(
            ".side_set 2",                      // side 0bWB - W = Word Clock, B = Bit Clock
            "    mov x, y           side 0b01", // y stores sample depth - 2 (14 = 16bit, 22 = 24bit, 30 = 32bit)
            "left_data:",
            "    out pins, 1        side 0b00",
            "    jmp x-- left_data  side 0b01",
            "    out pins, 1        side 0b10",
            "    mov x, y           side 0b11",
            "right_data:",
            "    out pins, 1         side 0b10",
            "    jmp x-- right_data side 0b11",
            "    out pins, 1         side 0b00",
        );

        let prg = common.load_program(&prg.program);

        Self { prg }
    }
}

/// Pio backed I2s output driver
pub struct PioI2sOut<'d, P: Instance, const S: usize> {
    dma: Peri<'d, AnyChannel>,
    sm: StateMachine<'d, P, S>,
}

impl<'d, P: Instance, const S: usize> PioI2sOut<'d, P, S> {
    /// Configure a state machine to output I2s
    pub fn new(
        common: &mut Common<'d, P>,
        mut sm: StateMachine<'d, P, S>,
        dma: Peri<'d, impl Channel>,
        data_pin: Peri<'d, impl PioPin>,
        bit_clock_pin: Peri<'d, impl PioPin>,
        lr_clock_pin: Peri<'d, impl PioPin>,
        sample_rate: u32,
        bit_depth: u32,
        program: &PioI2sOutProgram<'d, P>,
    ) -> Self {
        let data_pin = common.make_pio_pin(data_pin);
        let bit_clock_pin = common.make_pio_pin(bit_clock_pin);
        let left_right_clock_pin = common.make_pio_pin(lr_clock_pin);

        let cfg = {
            let mut cfg = Config::default();
            cfg.use_program(&program.prg, &[&bit_clock_pin, &left_right_clock_pin]);
            cfg.set_out_pins(&[&data_pin]);
            let clock_frequency = sample_rate * bit_depth * 2;
            cfg.clock_divider = (embassy_rp::clocks::clk_sys_freq() as f64 / clock_frequency as f64 / 2.).to_fixed();
            cfg.shift_out = ShiftConfig {
                threshold: 32,
                direction: ShiftDirection::Left,
                auto_fill: true,
            };
            // join fifos to have twice the time to start the next dma transfer
            cfg.fifo_join = FifoJoin::TxOnly;
            cfg
        };
        sm.set_config(&cfg);
        sm.set_pin_dirs(Direction::Out, &[&data_pin, &left_right_clock_pin, &bit_clock_pin]);

        // Set the `y` register up to configure the sample depth
        // The SM counts down to 0 and uses one clock cycle to set up the counter,
        // which results in bit_depth - 2 as register value.
        unsafe { sm.set_y(bit_depth - 2) };

        sm.set_enable(true);

        Self { dma: dma.into(), sm }
    }

    /// Return an in-prograss dma transfer future. Awaiting it will guarentee a complete transfer.
    pub fn write<'b>(&'b mut self, buff: &'b [u32]) -> Transfer<'b, AnyChannel> {
        self.sm.tx().dma_push(self.dma.reborrow(), buff, false)
    }
}