#![no_std]
#![no_main]

use panic_semihosting as _;

pub mod bus;
pub mod write_to;

use bme280::BME280;
use rtfm::app;
use rtfm::cyccnt::{Instant, U32Ext as _};

const PERIOD: u32 = 1000;
use asm_delay::bitrate::*;
use asm_delay::AsmDelay;
use cortex_m_semihosting::hprintln;
use embedded_graphics::fonts::Font6x8;

use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Line;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::text_6x8;
use embedded_hal::digital::v2::OutputPin;
use heapless::consts::{U1024, U32};
use heapless::Vec;
use ssd1306::prelude::*;
use ssd1306::Builder;
use stm32f1::stm32f103::I2C1;
use stm32f1xx_hal::{
    delay::Delay,
    gpio::{
        gpiob::{PB6, PB7},
        Alternate, Edge, ExtiPin, Floating, Input, OpenDrain, PullDown,
    },
    i2c::{BlockingI2c, DutyCycle, Mode},
    prelude::*,
    timer::Timer,
};

type I2c<SCL, SDA> = BlockingI2c<I2C1, (SCL, SDA)>;

pub struct MeasurementEntry {
    pub temperature: u16,
}

#[app(device = stm32f1xx_hal::pac, monotonic = rtfm::cyccnt::CYCCNT, peripherals = true)]
const APP: () = {
    struct Resources {
        BUS: bus::Bus,
        // MEASUREMENTS: Vec<MeasurementEntry, U1024>,
        MEASUREMENTS: Vec<u16, U1024>,
        REFRESH_DISPLAY_SPAWNED: bool,
        EXTI: stm32f1xx_hal::gpio::gpioa::PA4<Input<PullDown>>,
        CURRENT_SCREEN: u8,
    }

    #[init(spawn = [measure_environment])]
    fn init(ctx: init::Context) -> init::LateResources {
        let p = ctx.device;
        let mut core = ctx.core;

        core.DWT.enable_cycle_counter();

        let mut rcc = p.RCC.constrain();
        let mut flash = p.FLASH.constrain();

        let mut gpiob = p.GPIOB.split(&mut rcc.apb2);
        let mut gpioa = p.GPIOA.split(&mut rcc.apb2);

        let clocks = rcc.cfgr.freeze(&mut flash.acr);
        let mut afio = p.AFIO.constrain(&mut rcc.apb2);

        let mut exti = gpioa.pa4.into_pull_down_input(&mut gpioa.crl);
        exti.make_interrupt_source(&mut afio);
        exti.trigger_on_edge(&p.EXTI, Edge::FALLING);
        exti.enable_interrupt(&p.EXTI);

        let mut nvic = core.NVIC;
        nvic.enable(stm32f1xx_hal::pac::Interrupt::EXTI1);

        let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
        let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);

        let i2c = BlockingI2c::i2c1(
            p.I2C1,
            (scl, sda),
            &mut afio.mapr,
            Mode::Fast {
                frequency: stm32f1xx_hal::time::U32Ext::hz(100_000),
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

        ctx.spawn.measure_environment().unwrap();

        init::LateResources {
            BUS: bus,
            MEASUREMENTS: Vec::new(),
            REFRESH_DISPLAY_SPAWNED: false,
            EXTI: exti,
            CURRENT_SCREEN: 0,
        }
    }
    #[task(binds = EXTI4,  spawn = [refresh_display], resources = [EXTI, CURRENT_SCREEN])]
    fn button_pressed(c: button_pressed::Context) {
        let exti = c.resources.EXTI;

        if exti.check_interrupt() {
            *c.resources.CURRENT_SCREEN = (*c.resources.CURRENT_SCREEN + 1) % 2;
            exti.clear_interrupt_pending_bit();
        }
    }

    #[task(schedule = [refresh_display], capacity = 2, resources = [BUS, MEASUREMENTS, CURRENT_SCREEN])]
    fn refresh_display(ctx: refresh_display::Context) {
        let current_screen = ctx.resources.CURRENT_SCREEN;
        let devices = ctx.resources.BUS.devices_mut();
        let disp = &mut devices.disp;

        let measurements = ctx.resources.MEASUREMENTS;
        disp.clear();
        if current_screen == &0 {
            if let Some(recent_measurement) = measurements.last() {
                let mut buf = [0 as u8; 32];

                let mut s: &str = write_to::show(
                    &mut buf,
                    format_args!("{} deg C", ((*recent_measurement) as f32 / 100.0)),
                )
                .unwrap();

                disp.draw(
                    Font6x8::render_str(&s)
                        .stroke(Some(BinaryColor::On))
                        .into_iter(),
                );
            }

            disp.draw(
                Line::new(Point::new(0, 31), Point::new(127, 31)).stroke(Some(BinaryColor::On)),
            );
            disp.draw(
                Line::new(Point::new(0, 9), Point::new(127, 9)).stroke(Some(BinaryColor::On)),
            );

            for (i, temperatures_chunk) in measurements.chunks(32).enumerate() {
                let mut sum: u32 = 0;
                for temp in temperatures_chunk.iter() {
                    sum += *temp as u32;
                }

                let average_temperature = sum / temperatures_chunk.len() as u32;
                let temperature = average_temperature / 100;
                let scaled_temperature = 12 - (34 - temperature) as i32;

                if temperatures_chunk.len() == 32 {
                    disp.draw(
                        Rectangle::new(
                            Point::new((0 + i * 4) as i32, 31 - (scaled_temperature * 22 / 12)),
                            Point::new((2 + i * 4) as i32, 31),
                        )
                        .fill(Some(BinaryColor::On)),
                    );
                } else {
                    disp.draw(
                        Rectangle::new(
                            Point::new((0 + i * 4) as i32, 31 - (scaled_temperature * 22 / 12)),
                            Point::new((2 + i * 4) as i32, 31),
                        )
                        .stroke_width(1)
                        .stroke(Some(BinaryColor::On)),
                    );
                }
            }
        } else {
            let bme = &mut devices.bme;

            let measure = bme.measure().unwrap();
            let mut buf = [0 as u8; 32];

            let mut s: &str = write_to::show(
                &mut buf,
                format_args!(
                    "{} deg C",
                    (((measure.temperature * 100.0) as u32) as f32 / 100.0)
                ),
            )
            .unwrap();
            disp.draw(
                Font6x8::render_str(&s)
                    .stroke(Some(BinaryColor::On))
                    .into_iter(),
            );

            s = write_to::show(&mut buf, format_args!("{} pascals", measure.pressure)).unwrap();

            disp.draw(
                Font6x8::render_str(&s)
                    .stroke(Some(BinaryColor::On))
                    .translate(Point::new(0, 10))
                    .into_iter(),
            );

            s = write_to::show(&mut buf, format_args!("{}%", measure.humidity)).unwrap();

            disp.draw(
                Font6x8::render_str(&s)
                    .stroke(Some(BinaryColor::On))
                    .translate(Point::new(0, 20))
                    .into_iter(),
            );
        }
        disp.flush().unwrap();

        ctx.schedule
            .refresh_display(Instant::now() + 1_000_000.cycles())
            .unwrap();
    }

    #[task(schedule = [measure_environment], spawn = [refresh_display], resources = [BUS, MEASUREMENTS, REFRESH_DISPLAY_SPAWNED])]
    fn measure_environment(ctx: measure_environment::Context) {
        let devices = ctx.resources.BUS.devices_mut();
        let bme = &mut devices.bme;

        let measure = bme.measure().unwrap();
        let mut measurements = ctx.resources.MEASUREMENTS;

        if measurements.len() + 1 == measurements.capacity() {
            measurements.clear();
        }
        measurements.push(((measure.temperature * 100.0) as u16));

        if !*ctx.resources.REFRESH_DISPLAY_SPAWNED {
            *ctx.resources.REFRESH_DISPLAY_SPAWNED = true;
            ctx.spawn.refresh_display().unwrap();
        }

        ctx.schedule
            .measure_environment(Instant::now() + 100.cycles())
            .unwrap();
    }

    extern "C" {
        fn TIM2();
    }
};
