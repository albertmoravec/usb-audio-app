#![no_std]
#![no_main]

use cortex_m::{asm, interrupt::free, peripheral::NVIC};
use panic_rtt_target as _;
use stm32f4xx_hal as hal;

use cortex_m_rt::entry;
use hal::{
    dma::{config::DmaConfig, MemoryToPeripheral, Stream4, StreamsTuple},
    dma::{Channel0, Transfer},
    gpio::{gpiob, gpioc},
    interrupt,
    pac::{DMA1, SPI2},
    prelude::*,
    stm32::{self, Interrupt},
};
use rtt_target::rprintln;

static mut TOOTH: &'static mut [u16; 64] = &mut [
    0x000, 0x000, 0x100, 0x100, 0x200, 0x200, 0x300, 0x300, 0x400, 0x400, 0x500, 0x500, 0x600,
    0x600, 0x700, 0x700, 0x800, 0x800, 0x900, 0x900, 0xA00, 0xA00, 0xB00, 0xB00, 0xC00, 0xC00,
    0xD00, 0xD00, 0xE00, 0xE00, 0xF00, 0xF00, 0xFFF, 0xFFF, 0xEFF, 0xEFF, 0xDFF, 0xDFF, 0xCFF,
    0xCFF, 0xBFF, 0xBFF, 0xAFF, 0xAFF, 0x9FF, 0x9FF, 0x8FF, 0x8FF, 0x7FF, 0x7FF, 0x6FF, 0x6FF,
    0x5FF, 0x5FF, 0x4FF, 0x4FF, 0x3FF, 0x3FF, 0x2FF, 0x2FF, 0x1FF, 0x1FF, 0x0FF, 0x0FF,
];

// static mut TOOTH: &'static mut [u8; 128] = &mut [
//     0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x0, 0x1, 0x0, 0x2, 0x0, 0x2, 0x0, 0x3, 0x0, 0x3, 0x0, 0x4, 0x0,
//     0x4, 0x0, 0x5, 0x0, 0x5, 0x0, 0x6, 0x0, 0x6, 0x0, 0x7, 0x0, 0x7, 0x0, 0x8, 0x0, 0x8, 0x0, 0x9,
//     0x0, 0x9, 0x0, 0xA, 0x0, 0xA, 0x0, 0xB, 0x0, 0xB, 0x0, 0xC, 0x0, 0xC, 0x0, 0xD, 0x0, 0xD, 0x0,
//     0xE, 0x0, 0xE, 0x0, 0xF, 0x0, 0xF, 0xFF, 0xF, 0xFF, 0xF, 0xFF, 0xE, 0xFF, 0xE, 0xFF, 0xD, 0xFF,
//     0xD, 0xFF, 0xC, 0xFF, 0xC, 0xFF, 0xB, 0xFF, 0xB, 0xFF, 0xA, 0xFF, 0xA, 0xFF, 0x9, 0xFF, 0x9,
//     0xFF, 0x8, 0xFF, 0x8, 0xFF, 0x7, 0xFF, 0x7, 0xFF, 0x6, 0xFF, 0x6, 0xFF, 0x5, 0xFF, 0x5, 0xFF,
//     0x4, 0xFF, 0x4, 0xFF, 0x3, 0xFF, 0x3, 0xFF, 0x2, 0xFF, 0x2, 0xFF, 0x1, 0xFF, 0x1, 0xFF, 0x0,
//     0xFF, 0x0,
// ];

type DmaTransfer = Transfer<
    Stream4<DMA1>,
    Channel0,
    SPI2,
    MemoryToPeripheral,
    &'static mut [u16; 64],
    u16, /* transfer size */
>;

#[entry]
fn main() -> ! {
    let mut dp = stm32::Peripherals::take().unwrap();
    let mut cortex = cortex_m::peripheral::Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();

    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(48.mhz())
        .pclk1(24.mhz())
        .require_pll48clk()
        .freeze();

    rtt_target::rtt_init_print!();

    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    let gpiod = dp.GPIOD.split();

    init_spi(&mut dp.SPI2, gpioc.pc3, gpiob.pb10, gpiob.pb12);

    let stream_4 = StreamsTuple::new(dp.DMA1).4;

    let config = DmaConfig::default()
        .transfer_complete_interrupt(true)
        .memory_increment(true)
        .double_buffer(true);

    let i2s = dp.SPI2;

    let mut transfer = DmaTransfer::init(stream_4, i2s, unsafe { TOOTH }, Some(unsafe {TOOTH}), config);

    // unsafe { stm32::Peripherals::steal().DMA1.st[4].cr.modify(|_, w| w.circ().enabled()); }

    free(|_cs| unsafe {
        NVIC::unpend(Interrupt::SPI2);
        NVIC::unmask(Interrupt::SPI2);

        NVIC::unpend(Interrupt::DMA1_STREAM4);
        NVIC::unmask(Interrupt::DMA1_STREAM4);
    });

    transfer.start(|_| {});

    loop {
        // rprintln!("Loop");
        asm::nop();
    }
}

#[interrupt]
fn DMA1_STREAM0() {
    free(|_cs| unsafe {
        rprintln!("DMA interrupt");

        NVIC::unpend(Interrupt::DMA1_STREAM4);
    });
}

#[interrupt]
fn SPI2() {
    free(|_cs| unsafe {
        rprintln!("SPI interrupt");

        NVIC::unpend(Interrupt::SPI2);
    });
}

fn init_spi<X, Y, Z>(
    i2s: &mut stm32::SPI2,
    pc3: gpioc::PC3<X>,
    pb10: gpiob::PB10<Y>,
    pb12: gpiob::PB12<Z>,
) {
    let rcc = unsafe { &*stm32::RCC::ptr() };

    rcc.ahb1enr
        .modify(|_, w| w.gpiocen().enabled().gpioben().enabled());
    rcc.apb1enr.modify(|_, w| w.spi2en().enabled());

    i2s.cr2.write(
        |w| w.txdmaen().enabled(), // .txeie().not_masked()
    );

    i2s.i2spr
        .write(|w| unsafe { w.i2sdiv().bits(12).odd().odd().mckoe().disabled() });

    i2s.i2scfgr.write(|w| {
        w.chlen()
            .sixteen_bit()
            .datlen()
            .sixteen_bit()
            .ckpol()
            .idle_low()
            .i2sstd()
            .philips()
            .i2scfg()
            .master_tx()
            .i2smod()
            .i2smode()
            .i2se()
            .enabled()
    });

    pc3.into_alternate_af5();
    pb10.into_alternate_af5();
    pb12.into_alternate_af5();
}
