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
    otg_hs::{UsbBus, USB},
    pac::{DMA1, SPI2},
    prelude::*,
    stm32::{self, Interrupt},
    time::Hertz,
};
use rtt_target::rprintln;
use usb_audio::class::AudioClass;
use usb_device::device::{UsbDeviceBuilder, UsbVidPid};

static mut EP_MEMORY: [u32; 1024] = [0; 1024];

static mut TOOTH: &'static mut [u16; 64] = &mut [
    0x000, 0x000, 0x100, 0x100, 0x200, 0x200, 0x300, 0x300, 0x400, 0x400, 0x500, 0x500, 0x600,
    0x600, 0x700, 0x700, 0x800, 0x800, 0x900, 0x900, 0xA00, 0xA00, 0xB00, 0xB00, 0xC00, 0xC00,
    0xD00, 0xD00, 0xE00, 0xE00, 0xF00, 0xF00, 0xFFF, 0xFFF, 0xEFF, 0xEFF, 0xDFF, 0xDFF, 0xCFF,
    0xCFF, 0xBFF, 0xBFF, 0xAFF, 0xAFF, 0x9FF, 0x9FF, 0x8FF, 0x8FF, 0x7FF, 0x7FF, 0x6FF, 0x6FF,
    0x5FF, 0x5FF, 0x4FF, 0x4FF, 0x3FF, 0x3FF, 0x2FF, 0x2FF, 0x1FF, 0x1FF, 0x0FF, 0x0FF,
];

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
        .sysclk(96.mhz())
        .pclk1(24.mhz())
        .require_pll48clk()
        .freeze();

    rtt_target::rtt_init_print!();

    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    let gpiod = dp.GPIOD.split();

    init_spi(&mut dp.SPI2, gpioc.pc3, gpiob.pb10, gpiob.pb12);
    let usb = init_usb(
        dp.OTG_HS_GLOBAL,
        dp.OTG_HS_DEVICE,
        dp.OTG_HS_PWRCLK,
        gpiob.pb14,
        gpiob.pb15,
        clocks.hclk(),
    );

    let stream_4 = StreamsTuple::new(dp.DMA1).4;

    let config = DmaConfig::default()
        .transfer_complete_interrupt(true)
        .memory_increment(true)
        .double_buffer(true);

    let mut transfer = DmaTransfer::init(
        stream_4,
        dp.SPI2,
        unsafe { TOOTH },
        Some(unsafe { TOOTH }),
        config,
    );

    let bus = UsbBus::new(usb, unsafe { &mut EP_MEMORY });
    let mut usb_audio = AudioClass::new(&bus);
    let mut usb_dev = UsbDeviceBuilder::new(&bus, UsbVidPid(0x5824, 0x27dd))
        .manufacturer("Albru")
        .product("SuperHighTech audio device")
        .serial_number("TEST")
        .device_class(0)
        .max_packet_size_0(64)
        .build();

    // free(|_cs| unsafe {
    //     NVIC::unpend(Interrupt::SPI2);
    //     NVIC::unmask(Interrupt::SPI2);

    //     NVIC::unpend(Interrupt::DMA1_STREAM4);
    //     NVIC::unmask(Interrupt::DMA1_STREAM4);
    // });

    transfer.start(|_| {});

    loop {
        if usb_dev.poll(&mut [&mut usb_audio]) {}
    }
}

// #[interrupt]
// fn DMA1_STREAM0() {
//     free(|_cs| unsafe {
//         rprintln!("DMA interrupt");

//         NVIC::unpend(Interrupt::DMA1_STREAM4);
//     });
// }

// #[interrupt]
// fn SPI2() {
//     free(|_cs| unsafe {
//         rprintln!("SPI interrupt");

//         NVIC::unpend(Interrupt::SPI2);
//     });
// }

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

fn init_usb<X, Y>(
    hs_global: stm32::OTG_HS_GLOBAL,
    hs_device: stm32::OTG_HS_DEVICE,
    hs_pwrclk: stm32::OTG_HS_PWRCLK,
    pb14: gpiob::PB14<X>,
    pb15: gpiob::PB15<Y>,
    hclk: Hertz,
) -> USB {
    USB {
        usb_global: hs_global,
        usb_device: hs_device,
        usb_pwrclk: hs_pwrclk,
        pin_dm: pb14.into_alternate_af12(),
        pin_dp: pb15.into_alternate_af12(),
        hclk: hclk,
    }
}
