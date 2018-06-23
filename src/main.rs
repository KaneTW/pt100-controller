extern crate rppal;

use rppal::spi;
use rppal::gpio;
use std::mem;
use std::time;
use std::thread;


struct State {
    gpio: gpio::Gpio,
    spi: spi::Spi
}

enum ADCCommand {
  Wakeup,
  Sleep,
  Sync,
  Reset,
  Rdata,
  Rdatac,
  Sdatac,
  Rreg { reg: u8, extra_count: u8 },
  Wreg { reg: u8, data: Vec<u8> },
  Sysocal,
  Sysgcal,
  Selfocal,
  Nop
}

fn serialize_command(cmd: &ADCCommand) -> Vec<u8> {
    use ADCCommand::*;
    match cmd {
        Wakeup => vec![0x0],
        Sleep => vec![0x2],
        Sync => vec![0x4],
        Reset => vec![0x6],
        Rdata => vec![0x12],
        Rdatac => vec![0x14],
        Sdatac => vec![0x16],
        Rreg { reg, extra_count } => vec![0x20 | reg, *extra_count],
        Wreg { reg, data } => { 
            let mut out = vec![0x40 | reg, data.len() as u8 - 1];
            out.extend_from_slice(&data);
            out },
        Sysocal => vec![0x60],
        Sysgcal => vec![0x61],
        Selfocal => vec![0x62],
        Nop => vec![0xff]
    }
}

#[derive(Copy, Clone)]
enum BurnoutCurrentSource {
    BcsOff = 0,
    Bcs0R5 = 1,
    Bcs2 = 2,
    Bcs10 = 3
}

#[derive(Copy, Clone)]
enum InputChannel {
    Ain0 = 0,
    Ain1 = 1,
    Ain2 = 2,
    Ain3 = 3,
    Ain4 = 4,
    Ain5 = 5,
    Ain6 = 6,
    Ain7 = 7
}

#[derive(Copy, Clone)]
enum InternalReferenceControl {
    Off = 0,
    On = 1,
    OnInUse = 2
}

#[derive(Copy, Clone)]
enum ReferenceSelectControl {
    Ref0 = 0,
    Ref1 = 1,
    Internal = 2,
    InternalConnectedRef0 = 3
}

#[derive(Copy, Clone)]
enum SystemMonitorControl {
    NormalOp = 0,
    OffsetCal = 1,
    GainCal = 2,
    Temperature = 3,
    Ref1 = 4,
    Ref0 = 5,
    Avdd = 6,
    Dvdd = 7
}

#[derive(Copy, Clone)]
enum PgaGain {
    Gain1 = 0,
    Gain2 = 1,
    Gain4 = 2,
    Gain8 = 3,
    Gain16 = 4,
    Gain32 = 5,
    Gain64 = 6,
    Gain128 = 7
}

#[derive(Copy, Clone)]
enum DataRate {
    Sps5 = 0,
    Sps10 = 1,
    Sps20 = 2,
    Sps40 = 3,
    Sps80 = 4,
    Sps160 = 5,
    Sps320 = 6,
    Sps640 = 7,
    Sps1000 = 8,
    Sps2000 = 9
}

#[derive(Copy, Clone)]
enum ExcCurrentMagnitude {
    ExcOff = 0,
    Exc50 = 1,
    Exc100 = 2,
    Exc250 = 3,
    Exc500 = 4,
    Exc750 = 5,
    Exc1000 = 6,
    Exc1500 = 7
}

#[derive(Copy, Clone)]
enum ExcCurrentOutput {
    Ain0 = 0,
    Ain1 = 1,
    Ain2 = 2,
    Ain3 = 3,
    Ain4 = 4,
    Ain5 = 5,
    Ain6 = 6,
    Ain7 = 7,
    Iexc1 = 8,
    Iexc2 = 9
}

#[derive(Copy, Clone)]
enum Register {
    Mux0 { bcs: BurnoutCurrentSource, mux_sp: InputChannel, mux_sn: InputChannel },
    Vbias { vbias: [bool; 8] },
    Mux1 { vrefcon: InternalReferenceControl, refselt: ReferenceSelectControl, muxcal: SystemMonitorControl },
    Sys0 { pga: PgaGain, dr: DataRate },
    Ofc { ofc: u32 },
    Fsc { fsc: u32 },
    Idac0 { drdy_mode: bool, imag: ExcCurrentMagnitude },
    Idac1 { i1dir: ExcCurrentOutput, i2dir: ExcCurrentOutput },
    Gpiocfg { iocfg: [bool; 8] },
    Gpiodir { iodir: [bool; 8] },
    Gpiodat { iodat: [bool; 8] } 
}

fn fold_byte(array: &[bool; 8]) -> u8 {
    array.iter().rev().fold(0, |acc, &b| acc << 1 | b as u8)
}

// register order
fn serialize_24bit(n: u32) -> Vec<u8> {
    vec![(n & 0xff) as u8, ((n & 0xff00) >> 8) as u8, ((n & 0xff0000) >> 16) as u8]
}

fn serialize_register(reg: Register) -> Vec<u8> {
    use Register::*;
    match reg {
        Mux0 { bcs, mux_sp, mux_sn } => vec![(bcs as u8) << 6 | (mux_sp as u8) << 3 | (mux_sn as u8)],
        Vbias { vbias } => vec![fold_byte(&vbias)],
        Mux1 { vrefcon, refselt, muxcal } => vec![(vrefcon as u8) << 5 | (refselt as u8) << 3 | (muxcal as u8)],
        Sys0 { pga, dr } => vec![(pga as u8) << 4 | (dr as u8)],
        Ofc { ofc } => serialize_24bit(ofc),
        Fsc { fsc } => serialize_24bit(fsc),
        Idac0 { drdy_mode, imag } => vec![(drdy_mode as u8) << 3 | (imag as u8)],
        Idac1 { i1dir, i2dir } => vec![(i1dir as u8) << 4 | (i2dir as u8)],
        Gpiocfg { iocfg } => vec![fold_byte(&iocfg)],
        Gpiodir { iodir } => vec![fold_byte(&iodir)],
        Gpiodat { iodat } => vec![fold_byte(&iodat)]
    }
 }

fn register_index(reg: Register) -> u8 {
    use Register::*;
    match reg {
        Mux0 { bcs, mux_sp, mux_sn } => 0,
        Vbias { vbias } => 1,
        Mux1 { vrefcon, refselt, muxcal } => 2,
        Sys0 { pga, dr } => 3,
        Ofc { ofc } => 4,
        Fsc { fsc } => 7,
        Idac0 { drdy_mode, imag } => 0xa,
        Idac1 { i1dir, i2dir } => 0xb,
        Gpiocfg { iocfg } => 0xc,
        Gpiodir { iodir } => 0xd,
        Gpiodat { iodat } => 0xe
    }
}

fn send_command(state: &mut State, cmd: &ADCCommand) {
    let data = serialize_command(cmd);
    state.spi.write(&data).unwrap();
}

fn write_register(state: &mut State, reg: Register) {
    let index = register_index(reg);
    let data = serialize_register(reg);
    let cmd = ADCCommand::Wreg { reg: index, data: data };
    send_command(state, &cmd);
}

const GPIO_RESET: u8 = 17;
const GPIO_START: u8 = 27;
const GPIO_DRDY: u8 = 25;
const GPIO_MUX_A: u8 = 22;
const GPIO_MUX_B: u8 = 23;
const GPIO_MUX_INH: u8 = 24;
const SPI_FREQ: u32 = 500*1000;

fn setup() -> State {
  let mut gpio = gpio::Gpio::new().unwrap();
  gpio.set_mode(GPIO_RESET, gpio::Mode::Output);
  gpio.set_mode(GPIO_START, gpio::Mode::Output);
  
  gpio.set_mode(GPIO_DRDY, gpio::Mode::Input);
  gpio.set_pullupdown(GPIO_DRDY, gpio::PullUpDown::PullUp);

  gpio.set_mode(GPIO_MUX_A, gpio::Mode::Output);
  gpio.set_mode(GPIO_MUX_B, gpio::Mode::Output);
  gpio.set_mode(GPIO_MUX_INH, gpio::Mode::Output);

  gpio.write(GPIO_RESET, gpio::Level::High);
  gpio.write(GPIO_START, gpio::Level::High);
  gpio.write(GPIO_MUX_A, gpio::Level::Low);
  gpio.write(GPIO_MUX_B, gpio::Level::Low);
  gpio.write(GPIO_MUX_INH, gpio::Level::Low);
  
  gpio.set_interrupt(GPIO_DRDY, gpio::Trigger::FallingEdge).unwrap();

  let mut spi = spi::Spi::new(spi::Bus::Spi0, spi::SlaveSelect::Ss0, SPI_FREQ, spi::Mode::Mode1).unwrap();

  State { gpio: gpio, spi: spi }

}

fn post_reset(state: &mut State) {
  send_command(state, &ADCCommand::Reset);
  thread::sleep(time::Duration::from_micros(600));
  send_command(state, &ADCCommand::Sdatac);
  write_register(state, Register::Mux1 { vrefcon: InternalReferenceControl::On, refselt: ReferenceSelectControl::Ref0, muxcal: SystemMonitorControl::NormalOp});
  write_register(state, Register::Sys0 { pga: PgaGain::Gain4, dr: DataRate::Sps20});
  write_register(state, Register::Idac0 { drdy_mode: false, imag: ExcCurrentMagnitude::Exc1000});
  write_register(state, Register::Idac1 { i1dir: ExcCurrentOutput::Iexc1, i2dir: ExcCurrentOutput::Iexc2})
}

#[derive(Copy, Clone, Debug)]
enum Channel {
    Ch1,
    Ch2,
    Ch3,
    Ch4
}

fn select_output(state: &mut State, ch: Channel) {
    use Channel::*;
    match ch {
        Ch1 => { 
            state.gpio.write(GPIO_MUX_A, gpio::Level::Low);
            state.gpio.write(GPIO_MUX_B, gpio::Level::Low);
            write_register(state, Register::Mux0 { bcs : BurnoutCurrentSource::BcsOff, mux_sp: InputChannel::Ain0, mux_sn: InputChannel::Ain5})
        },
        Ch2 => { 
            state.gpio.write(GPIO_MUX_A, gpio::Level::High);
            state.gpio.write(GPIO_MUX_B, gpio::Level::Low);
            write_register(state, Register::Mux0 { bcs : BurnoutCurrentSource::BcsOff, mux_sp: InputChannel::Ain4, mux_sn: InputChannel::Ain1})
        },
        Ch3 => { 
            state.gpio.write(GPIO_MUX_A, gpio::Level::Low);
            state.gpio.write(GPIO_MUX_B, gpio::Level::High);
            write_register(state, Register::Mux0 { bcs : BurnoutCurrentSource::BcsOff, mux_sp: InputChannel::Ain2, mux_sn: InputChannel::Ain3})
        },
        Ch4 => { 
            state.gpio.write(GPIO_MUX_A, gpio::Level::High);
            state.gpio.write(GPIO_MUX_B, gpio::Level::High);
            write_register(state, Register::Mux0 { bcs : BurnoutCurrentSource::BcsOff, mux_sp: InputChannel::Ain6, mux_sn: InputChannel::Ain7})
        }
    }
}

fn read_last_measurement(state: &mut State) -> u32 {
    let mut read_buffer: [u8; 4] = [0; 4];
    let write_buffer: Vec<u8> = [ADCCommand::Rdata, ADCCommand::Nop, ADCCommand::Nop, ADCCommand::Nop].iter().flat_map(|cmd| serialize_command(cmd)).collect();
    state.spi.transfer(&mut read_buffer, &write_buffer).unwrap();
    (read_buffer[1] as u32) << 16 | (read_buffer[2] as u32) << 8 | (read_buffer[3] as u32)
}

fn chop(state: &mut State, chop: bool) {
    if chop {
         write_register(state, Register::Idac1 { i1dir: ExcCurrentOutput::Iexc2, i2dir: ExcCurrentOutput::Iexc1});
    } else {
         write_register(state, Register::Idac1 { i1dir: ExcCurrentOutput::Iexc1, i2dir: ExcCurrentOutput::Iexc2});
    }
}

fn main() {
    let mut state = setup();
    post_reset(&mut state);
    let chs = [Channel::Ch1, Channel::Ch2, Channel::Ch3, Channel::Ch4];
    let infinite_channels = chs.iter().cycle();
    
    for ch in infinite_channels {
        select_output(&mut state, *ch);
        chop(&mut state, false);
        state.gpio.poll_interrupt(GPIO_DRDY, true, None).unwrap();
        let code = read_last_measurement(&mut state);
        println!("{:?}: {}", ch, code);
        chop(&mut state, true);
        state.gpio.poll_interrupt(GPIO_DRDY, true, None).unwrap();
        let code = read_last_measurement(&mut state);
        println!("{:?}: {}", ch, code);
    }
}
