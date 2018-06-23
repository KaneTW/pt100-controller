extern crate rppal;

use rppal::spi;
use rppal::gpio;
use std::time;
use std::thread;

struct State {
    gpio: gpio::Gpio,
    spi: spi::Spi
}

#[derive(Copy, Clone)]
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

fn serializeCommand(cmd: &ADCCommand) -> Vec<u8> {
    use ADCCommand::*;
    match cmd {
        Wakeup => vec![0x0],
        Sleep => vec![0x2],
        Sync => vec![0x4],
        Reset => vec![0x6],
        Rdata => vec![0x12],
        Rdatac => vec![0x14],
        Sdatac => vec![0x16],
        Rreg { reg, extra_count } => vec![0x20 | reg, extra_count],
        Wreg { reg, data } => { 
            let out = vec![0x40 | reg, data.len() as u8 - 1];
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
    InternalReferenceOff = 0,
    InternalReferenceOn = 1,
    InternalReferenceOnInUse = 2
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
    Iexc1Output = 8,
    Iexc2Output = 9
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

fn foldByte(array: &[bool; 8]) -> u8 {
    array.iter().rev().fold(0, |acc, &b| acc << 1 | b as u8)
}

fn serializeRegister(reg: Register) -> Vec<u8> {
    use Register::*;
    match reg {
        Mux0 { bcs, mux_sp, mux_sn } => vec![(bcs as u8) << 6 | (mux_sp as u8) << 3 | (mux_sn as u8)],
        Vbias { vbias } => vec![foldByte(&vbias)],
        Mux1 { vrefcon, refselt, muxcal } => vec![(vrefcon as u8) << 5 | (refselt as u8) << 3 | (muxcal as u8)],
        Sys0 { pga, dr } => vec![(pga as u8) << 4 | (dr as u8)],
        Ofc { ofc } => vec![((ofc & 0xff0000) >> 16) as u8, ((ofc & 0xff00) >> 8) as u8, (ofc & 0xff) as u8],
        Fsc { fsc } => vec![((fsc & 0xff0000) >> 16) as u8, ((fsc & 0xff00) >> 8) as u8, (fsc & 0xff) as u8],
        Idac0 { drdy_mode, imag } => vec![(drdy_mode as u8) << 3 | (imag as u8)],
        Idac1 { i1dir, i2dir } => vec![(i1dir as u8) << 4 | (i2dir as u8)],
        Gpiocfg { iocfg } => vec![foldByte(&iocfg)],
        Gpiodir { iodir } => vec![foldByte(&iodir)],
        Gpiodat { iodat } => vec![foldByte(&iodat)]
    }
 }

fn registerIndex(reg: Register) -> u8 {
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

fn sendCommand(state: &mut State, cmd: ADCCommand) {
    state.spi.write(&serializeCommand(cmd));
}

fn writeRegister(state: &mut State, reg: Register) {
    let index = registerIndex(reg);
    let data = serializeRegister(reg);
    let cmd = ADCCommand::Wreg { reg: index, data: data };
    sendCommand(state, cmd);
}

const GPIO_RESET: u8 = 17;
const GPIO_START: u8 = 27;
const GPIO_DRDY: u8 = 25;
const GPIO_MUX_A: u8 = 22;
const GPIO_MUX_B: u8 = 23;
const GPIO_MUX_INH: u8 = 24;
const SPI_FREQ: u32 = 1000*1000;

fn setup() -> State {
  let gpio = gpio::Gpio::new().unwrap();
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

  let spi = spi::Spi::new(spi::Bus::Spi0, spi::SlaveSelect::Ss0, SPI_FREQ, spi::Mode::Mode1).unwrap();

  State { gpio: gpio, spi: spi }

}

fn post_reset(state: State) {
  sendCommand(&state, ADCCommand::Sdatac);
}

fn main() {

    println!("Hello, world!");
}
