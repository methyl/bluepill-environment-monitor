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

use rtfm::app;
use rtfm::cyccnt::{Instant, U32Ext as _};
// use stm32f1::stm32f103 as target;

const PERIOD: u32 = 4_000_000;
use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::{prelude::*, timer::Timer};

#[app(device = stm32f1xx_hal::pac, monotonic = rtfm::cyccnt::CYCCNT, peripherals = true)]
const APP: () = {
  struct Resources {
    // PERIPHERALS: stm32f1xx_hal::pac::Peripherals,
    LED:
      stm32f1xx_hal::gpio::gpioc::PC13<stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::PushPull>>,
  }

  #[init(spawn = [task1])]
  fn init(ctx: init::Context) -> init::LateResources {
    let p = ctx.device;

    let mut rcc = p.RCC.constrain();
    // let gpioc = &p.GPIOC;
    let mut gpioc = p.GPIOC.split(&mut rcc.apb2);

    // rcc.apb2enr.write(|w| w.iopcen().set_bit());
    // gpioc
    //     .crh
    //     .write(|w| w.mode13().bits(0b11).cnf13().bits(0b00));

    ctx.spawn.task1().unwrap();
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    init::LateResources {
      // PERIPHERALS: p,
      LED: led,
    }
  }

  #[task(schedule = [task2], resources = [LED])]
  fn task1(ctx: task1::Context) {
    let now = Instant::now();

    // let gpioc = &ctx.resources.PERIPHERALS.GPIOC;

    // gpioc.bsrr.write(|w| w.bs13().set_bit());
    &ctx.resources.LED.set_high().unwrap();

    ctx.schedule.task2(now + PERIOD.cycles()).unwrap()
  }

  #[task(schedule = [task1], resources = [LED])]
  fn task2(ctx: task2::Context) {
    let now = Instant::now();
    &ctx.resources.LED.set_low().unwrap();

    ctx.schedule.task1(now + PERIOD.cycles()).unwrap()
  }

  extern "C" {
    fn TIM2();
  }
};
