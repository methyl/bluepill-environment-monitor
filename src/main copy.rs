#![no_std]
#![no_main]

// pick a panicking behavior
extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
                         // extern crate panic_abort; // requires nightly
                         // extern crate panic_itm; // logs messages over ITM; requires ITM support
                         // extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger

// extern crate cortex_m;
// extern crate rtfm;
// // extern crate stm32f1xx_hal;
// // use cortex_m::peripheral::syst::SystClkSource;
// use rtfm::app;
// // use cortex_m::asm;
// // use cortex_m_rt::entry;
use cortex_m_semihosting::{debug, hprintln};

// #[app(device = stm32f1::stm32f103)]
// const APP: () = {
//     #[init(spawn = [foo])]
//     fn init(c: init::Context) {
//         hprintln!("foo - start").unwrap();
//         c.spawn.foo().unwrap();
//     }

//     #[idle]
//     fn idle(c: idle::Context) -> ! {
//         hprintln!("foo - start").unwrap();
//         loop {}
//     }

//     #[task(spawn = [bar, baz])]
//     fn foo(c: foo::Context) {
//         hprintln!("foo - start").unwrap();

//         // spawns `bar` onto the task scheduler
//         // `foo` and `bar` have the same priority so `bar` will not run until
//         // after `foo` terminates
//         c.spawn.bar().unwrap();

//         hprintln!("foo - middle").unwrap();

//         // spawns `baz` onto the task scheduler
//         // `baz` has higher priority than `foo` so it immediately preempts `foo`
//         c.spawn.baz().unwrap();

//         hprintln!("foo - end").unwrap();
//     }

//     #[task]
//     fn bar(_: bar::Context) {
//         hprintln!("bar").unwrap();
//     }

//     #[task(priority = 2)]
//     fn baz(_: baz::Context) {
//         hprintln!("baz").unwrap();
//     }

//     // Interrupt handlers used to dispatch software tasks
//     extern "C" {
//         fn UART4();
//         fn UART5();
//     }

//     // #[init(schedule = [foo, bar])]
//     // fn init(mut cx: init::Context) {
//     //     hprintln!("Hello, world!").unwrap();
//     //     hprintln!("Hello, world2!").unwrap();
//     //     // static mut X: u32 = 0;

//     //     // Cortex-M peripherals
//     //     // let core: rtfm::Peripherals = cx.core;

//     //     // // Device specific peripherals
//     //     // // let device: stm32f1xx_hal::pac::Peripherals = cx.device;
//     //     // let device = stm32f1xx_hal::pac::Peripherals::take().unwrap();
//     //     // let mut pwr = device.PWR;
//     //     // let mut rcc = device.RCC.constrain();
//     //     // let mut gpioc = device.GPIOC.split(&mut rcc.apb2);
//     //     // let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
//     //     // // spawn.periodic().unwrap();
//     //     // let mut syst = core.SYST;

//     //     // syst.set_clock_source(SystClkSource::Core);
//     //     // syst.set_reload(8_000_000); // Period = 1s
//     //     // syst.enable_interrupt();
//     //     // syst.enable_counter();
//     //     cx.core.DCB.enable_trace();
//     //     unsafe { cx.core.DWT.lar.write(0xC5ACCE55) }
//     //     cx.core.DWT.enable_cycle_counter();
//     //     // cx.schedule.foo(Instant::now() + PERIOD.cycles()).unwrap();
//     //     let now = cx.start; // the start time of the system

//     //     hprintln!("init @ {:?}", now).unwrap();

//     //     // Schedule `foo` to run 8e6 cycles (clock cycles) in the future
//     //     // cx.schedule.foo(now + 1_000_000.cycles()).unwrap();
//     //     // cx.schedule.foo(now + 2_000_000.cycles()).unwrap();

//     //     // Schedule `bar` to run 4e6 cycles in the future
//     //     cx.schedule.bar(now + 100.cycles()).unwrap();
//     // }

//     // #[task]
//     // fn foo(_: foo::Context) {
//     //     hprintln!("foo  @ {:?}", Instant::now()).unwrap();
//     // }

//     // #[task]
//     // fn bar(_: bar::Context) {
//     //     hprintln!("bar  @ {:?}", Instant::now()).unwrap();
//     // }

//     // extern "C" {
//     //     fn UART5();
//     // }
// };

pub mod write_to {
    use core::cmp::min;
    use core::fmt;

    pub struct WriteTo<'a> {
        buffer: &'a mut [u8],
        // on write error (i.e. not enough space in buffer) this grows beyond
        // `buffer.len()`.
        used: usize,
    }

    impl<'a> WriteTo<'a> {
        pub fn new(buffer: &'a mut [u8]) -> Self {
            WriteTo { buffer, used: 0 }
        }

        pub fn as_str(self) -> Option<&'a str> {
            if self.used <= self.buffer.len() {
                // only successful concats of str - must be a valid str.
                use core::str::from_utf8_unchecked;
                Some(unsafe { from_utf8_unchecked(&self.buffer[..self.used]) })
            } else {
                None
            }
        }
    }

    impl<'a> fmt::Write for WriteTo<'a> {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            if self.used > self.buffer.len() {
                return Err(fmt::Error);
            }
            let remaining_buf = &mut self.buffer[self.used..];
            let raw_s = s.as_bytes();
            let write_num = min(raw_s.len(), remaining_buf.len());
            remaining_buf[..write_num].copy_from_slice(&raw_s[..write_num]);
            self.used += raw_s.len();
            if write_num < raw_s.len() {
                Err(fmt::Error)
            } else {
                Ok(())
            }
        }
    }

    pub fn show<'a>(buffer: &'a mut [u8], args: fmt::Arguments) -> Result<&'a str, fmt::Error> {
        let mut w = WriteTo::new(buffer);
        fmt::write(&mut w, args)?;
        w.as_str().ok_or(fmt::Error)
    }
}

use rtfm::app;
use rtfm::cyccnt::{Instant, U32Ext as _};
// use stm32f1::stm32f103 as target;
use bme280::BME280;

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
struct MyMutex<T>(cortex_m::interrupt::Mutex<T>);

impl<T> shared_bus::BusMutex<T> for MyMutex<T> {
    fn create(v: T) -> MyMutex<T> {
        MyMutex(cortex_m::interrupt::Mutex::new(v))
    }

    fn lock<R, F: FnOnce(&T) -> R>(&self, f: F) -> R {
        cortex_m::interrupt::free(|cs| {
            let v = self.0.borrow(cs);
            f(v)
        })
    }
}
type MyBusManager<L, P> = shared_bus::BusManager<MyMutex<L>, P>;

type I2c<SCL, SDA> = BlockingI2c<I2C1, (SCL, SDA)>;

#[app(device = stm32f1xx_hal::pac, monotonic = rtfm::cyccnt::CYCCNT, peripherals = true)]
const APP: () = {
    struct Resources {
        BME: bme280::BME280<
            shared_bus::proxy::BusProxy<
                '_,
                MyMutex<
                    core::cell::RefCell<I2c<PB6<Alternate<OpenDrain>>, PB7<Alternate<OpenDrain>>>>,
                >,
            >,
            asm_delay::AsmDelay,
        >,
        // PERIPHERALS: stm32f1xx_hal::pac::Peripherals,
        // I2C: stm32f1xx_hal::i2c::BlockingI2c<
        //     stm32f1::stm32f103::I2C1,
        //     (
        //         stm32f1xx_hal::gpio::gpiob::PB6<
        //             stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>,
        //         >,
        //         stm32f1xx_hal::gpio::gpiob::PB7<
        //             stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>,
        //         >,
        //     ),
        // >,
    }

    #[init(spawn = [task1])]
    fn init(ctx: init::Context) -> init::LateResources {
        let p = ctx.device;
        let core = ctx.core;

        let mut rcc = p.RCC.constrain();
        let mut flash = p.FLASH.constrain();

        // let gpioc = &p.GPIOC;
        let mut gpiob = p.GPIOB.split(&mut rcc.apb2);

        // rcc.apb2enr.write(|w| w.iopcen().set_bit());
        // gpioc
        //     .crh
        //     .write(|w| w.mode13().bits(0b11).cnf13().bits(0b00));

        // ctx.spawn.task1().unwrap();
        let clocks = rcc.cfgr.freeze(&mut flash.acr);
        let mut afio = p.AFIO.constrain(&mut rcc.apb2);

        // let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
        let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);

        let delay = AsmDelay::new(bitrate::U32BitrateExt::mhz(72));

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

        let manager = MyBusManager::new(i2c);

        // let mut disp: GraphicsMode<_> = Builder::new()
        //     .size(DisplaySize::Display128x32)
        //     .connect_i2c(manager.acquire())
        //     .into();
        // disp.init().unwrap();

        let mut bme280 = BME280::new_primary(manager.acquire(), delay);
        bme280.init().unwrap();
        ctx.spawn.task1().unwrap();

        // hprintln!("{}", measurements.temperature);
        // let mut buf = String::<u8>::new();
        // let mut buf = [0 as u8; 20];

        // write!(&mut buf[..], "Count: {}", measurements.temperature);

        // let mut buf = [0u8; 64];
        // let s: &str = write_to::show(
        //     &mut buf,
        //     format_args!("temperature: {} deg C", measurements.temperature),
        // )
        // .unwrap();

        // disp.draw(
        //     Font6x8::render_str(&s)
        //         .stroke(Some(BinaryColor::On))
        //         .into_iter(),
        // );
        // disp.draw(
        //     Font6x8::render_str(measurements.temperature.to_string())
        //         .stroke(Some(BinaryColor::On))
        //         .into_iter(),
        // );
        // disp.draw(
        //     Font6x8::render_str(measurements.pressure.to_string())
        //         .stroke(Some(BinaryColor::On))
        //         .into_iter(),
        // );
        // disp.flush().unwrap();

        init::LateResources {
            // PERIPHERALS: p,
            // I2C: i2c,
            BME: bme280,
        }
    }

    // #[task(schedule = [task2], resources = [LED])]
    //     let now = Instant::now();

    //     // let gpioc = &ctx.resources.PERIPHERALS.GPIOC;

    //     // gpioc.bsrr.write(|w| w.bs13().set_bit());
    //     &ctx.resources.LED.set_high().unwrap();

    //     ctx.schedule.task2(now + PERIOD.cycles()).unwrap()
    // }

    #[task(schedule = [task1], resources = [BME])]
    fn task1(ctx: task1::Context) {
        let now = Instant::now();
        // &ctx.resources.LED.set_low().unwrap();
        let measurements = &ctx.resources.BME.measure().unwrap();

        ctx.schedule.task1(now + (PERIOD * 2).cycles()).unwrap()
    }

    extern "C" {
        fn TIM2();
    }
};
