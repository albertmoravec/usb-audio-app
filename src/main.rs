#![no_std]
#![no_main]

use core::{ops::Deref, sync::atomic::{Ordering, compiler_fence}};

use cortex_m::{
    asm,
    interrupt::{free, Mutex},
    peripheral::NVIC,
    singleton,
};
use panic_rtt_target as _;
use rtic::app;
use stm32f4xx_hal as hal;

use cortex_m_rt::entry;
use hal::{dma::{config::DmaConfig, MemoryToPeripheral, Stream4, StreamsTuple}, dma::{Channel0, Transfer, traits::{DMASet, PeriAddress}}, gpio::{gpiob, gpioc}, interrupt, otg_hs::{UsbBus, USB}, pac::{DMA1, SPI2, spi1}, prelude::*, stm32::{self, Interrupt}, time::Hertz};
use rtt_target::rprintln;
use usb_audio::class::AudioClass;
use usb_device::{
    class_prelude::UsbBusAllocator,
    device::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
    UsbError,
};

static mut EP_MEMORY: [u32; 1024] = [0; 1024];
static mut BUS: Option<UsbBusAllocator<UsbBus<USB>>> = None;

pub struct MyDMA;

// impl Deref for MyDMA {
//     type Target = spi1::RegisterBlock;
//     #[inline(always)]
//     fn deref(&self) -> &Self::Target {
//         unsafe { &*SPI2::ptr() }
//     }
// }

unsafe impl PeriAddress for MyDMA {
    type MemSize = u16;

    fn address(&self) -> u32 {
        //&self.dr as *const _ as u32
        unsafe { &(&*SPI2::ptr()).dr as *const _ as u32 }
    }
}

unsafe impl DMASet<Stream4<DMA1>, Channel0, MemoryToPeripheral> for MyDMA {}

type Buffer = &'static mut [u16; 96];
type DmaTransfer = Transfer<
    Stream4<DMA1>,
    Channel0,
    MyDMA,
    MemoryToPeripheral,
    Buffer,
>;

enum DmaState {
    Disabled,
    Enabled
}

#[app(device = stm32f4xx_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        audio_class: AudioClass<'static, UsbBus<USB>>,
        usb_device: UsbDevice<'static, UsbBus<USB>>,
        transfer: DmaTransfer,
        third_buffer: Option<Buffer>,
        #[init(DmaState::Disabled)]
        dma_state: DmaState,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        let core: cortex_m::Peripherals = cx.core;
        let mut device: hal::pac::Peripherals = cx.device;

        let clocks = device
            .RCC
            .constrain()
            .cfgr
            .use_hse(8.mhz())
            .sysclk(96.mhz())
            .pclk1(24.mhz())
            .require_pll48clk()
            .freeze();

        rtt_target::rtt_init_print!();

        let gpiob = device.GPIOB.split();
        let gpioc = device.GPIOC.split();

        init_spi(&mut device.SPI2, gpioc.pc3, gpiob.pb10, gpiob.pb12);

        let usb = init_usb_peripheral(
            device.OTG_HS_GLOBAL,
            device.OTG_HS_DEVICE,
            device.OTG_HS_PWRCLK,
            gpiob.pb14,
            gpiob.pb15,
            clocks.hclk(),
        );

        unsafe {
            BUS = Some(UsbBus::new(usb, &mut EP_MEMORY));
        }

        let first_buffer = singleton!(: [u16; 96] = [0; 96]).unwrap();
        let second_buffer = singleton!(: [u16; 96] = [0; 96]).unwrap();
        let third_buffer = singleton!(: [u16; 96] = [0; 96]).unwrap();

        let (audio_class, usb_device) = init_usb(unsafe { BUS.as_ref().unwrap() });
        let transfer = init_dma(device.DMA1, device.SPI2, first_buffer, second_buffer);

        rprintln!("initialized");

        init::LateResources {
            audio_class,
            usb_device,
            transfer,
            third_buffer: Some(third_buffer),
        }
    }

    #[idle(resources = [transfer])]
    fn idle(mut cx: idle::Context) -> ! {
        rprintln!("idle");

        cx.resources.transfer.lock(|transfer| transfer.start(|_| {}));

        rprintln!("transfer started");

        loop {
            compiler_fence(Ordering::SeqCst);
        }
    }

    #[task(binds = OTG_HS, resources = [usb_device, audio_class, third_buffer])]
    fn otg_hs(cx: otg_hs::Context) {
        if !cx.resources.usb_device.poll(&mut [cx.resources.audio_class]) {
            return;
        }

        let buf = cx.resources.third_buffer.as_deref_mut().unwrap();        

        let (_, aligned_buf, _) = unsafe { buf.align_to_mut::<u8>() };

        match cx.resources.audio_class.read_data(aligned_buf) {
            Ok(_) => (), //rprintln!("data read"),
            Err(UsbError::WouldBlock) => (),
            Err(_) => panic!("usb error"),
        };
    }

    #[task(binds = DMA1_STREAM4, resources = [third_buffer, transfer])]
    fn dma1_stream4(cx: dma1_stream4::Context) {
        let triple: Buffer = cx.resources.third_buffer.take().unwrap();
        let buf = cx
            .resources
            .transfer
            .next_transfer(triple)
            .map_err(|_| {})
            .unwrap()
            .0;
        
        *cx.resources.third_buffer = Some(buf);
    }
};

fn init_usb(
    bus: &'static UsbBusAllocator<UsbBus<USB>>,
) -> (
    AudioClass<'static, UsbBus<USB>>,
    UsbDevice<'static, UsbBus<USB>>,
) {
    let usb_audio = AudioClass::new(bus);
    let usb_dev = UsbDeviceBuilder::new(bus, UsbVidPid(0x5824, 0x27dd))
        .manufacturer("Albru")
        .product("SuperHighTech audio device")
        .serial_number("TEST")
        .device_class(0)
        .max_packet_size_0(64)
        .build();

    (usb_audio, usb_dev)
}

fn init_dma(
    dma1: hal::pac::DMA1,
    spi2: hal::pac::SPI2,
    first_buffer: Buffer,
    second_buffer: Buffer,
) -> DmaTransfer {
    let stream_4 = StreamsTuple::new(dma1).4;

    let config = DmaConfig::default()
        .transfer_complete_interrupt(true)
        .memory_increment(true)
        .double_buffer(true);

    DmaTransfer::init(stream_4, MyDMA{}, first_buffer, Some(second_buffer), config)
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

fn init_usb_peripheral<X, Y>(
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
