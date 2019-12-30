#![no_std]
#![no_main]

extern crate panic_halt;
use cortex_m_semihosting::{debug, hprintln};

pub mod bus;
pub mod write_to;

use bme280::BME280;
use rtfm::app;
use rtfm::cyccnt::{Instant, U32Ext as _};

const PERIOD: u32 = 4_000_000;
use embedded_graphics::fonts::Font6x8;
use embedded_graphics::text_6x8;

use asm_delay::bitrate::*;
use asm_delay::AsmDelay;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_hal::digital::v2::OutputPin;
use ssd1306::prelude::*;
use ssd1306::Builder;
use stm32f1::stm32f103::I2C1;
use stm32f1xx_hal::{
    delay::Delay,
    gpio::{
        gpiob::{PB6, PB7},
        Alternate, OpenDrain,
    },
    i2c::{BlockingI2c, DutyCycle, Mode},
    prelude::*,
    timer::Timer,
};

type I2c<SCL, SDA> = BlockingI2c<I2C1, (SCL, SDA)>;

#[app(device = stm32f1xx_hal::pac, monotonic = rtfm::cyccnt::CYCCNT, peripherals = true)]
const APP: () = {
    struct Resources {
        BUS: bus::Bus,
    }

    #[init(spawn = [task1])]
    fn init(ctx: init::Context) -> init::LateResources {
        let p = ctx.device;
        let core = ctx.core;

        let mut rcc = p.RCC.constrain();
        let mut flash = p.FLASH.constrain();

        let mut gpiob = p.GPIOB.split(&mut rcc.apb2);

        let clocks = rcc.cfgr.freeze(&mut flash.acr);
        let mut afio = p.AFIO.constrain(&mut rcc.apb2);

        let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
        let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);

        let i2c = BlockingI2c::i2c1(
            p.I2C1,
            (scl, sda),
            &mut afio.mapr,
            Mode::Fast {
                frequency: stm32f1xx_hal::time::U32Ext::hz(400),
                duty_cycle: DutyCycle::Ratio2to1,
            },
            clocks,
            &mut rcc.apb1,
            1000,
            10,
            1000,
            1000,
        );

        let bus = bus::Bus::new(i2c);

        // let manager = MyBusManager::new(i2c);

        // let mut disp: = Builder::new()
        //     .size(DisplaySize::Display128x32)
        //     .connect_i2c(manager.acquire())
        //     .into();
        // disp.init().unwrap();

        // let mut bme280 = BME280::new_primary(manager.acquire(), delay);
        // bme280.init().unwrap();
        ctx.spawn.task1().unwrap();

        // let measurements = bme280.measure().unwrap();

        init::LateResources {
            // PERIPHERALS: p,
            // I2C: i2c,
            BUS: bus,
        }
    }

    #[task(schedule = [task1], resources = [BUS])]
    fn task1(ctx: task1::Context) {
        let now = Instant::now();
        // let mut bus = &mut ctx.resources.BUS;
        let (disp, bme) = ctx.resources.BUS.devices_mut();
        // let disp = ctx.resources.BUS.disp_mut();
        let measurements = bme.measure().unwrap();

        let mut buf = [0 as u8; 20];

        let s: &str = write_to::show(
            &mut buf,
            format_args!("temperature: {} deg C", measurements.temperature),
        )
        .unwrap();

        disp.draw(
            Font6x8::render_str(&s)
                .stroke(Some(BinaryColor::On))
                .into_iter(),
        );
        ctx.schedule.task1(now + (PERIOD * 2).cycles()).unwrap()
    }

    extern "C" {
        fn TIM2();
    }
};
