// Trying to solve https://github.com/Rahix/shared-bus/issues/4 by
// using a single struct with BusManager and all devices that share
// the bus + a bit of lifetime hacking. The struct is RTFM shared
// resource, so RTFM manages locks on the whole bus and the task can
// access the bus safely without additional locking.
use bme280::BME280;
use ssd1306::prelude::*;
use ssd1306::Builder;
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

use asm_delay::bitrate::*;
use asm_delay::AsmDelay;
use core::cell::RefCell;
use core::mem::transmute;
use core::ops::Deref;
use shared_bus::{BusManager, BusProxy};
use stm32f1::stm32f103::I2C1;

/// NOP mutex. It does nothing. We manage all I²C devices together
/// with the manager in one struct. They are one RTFM resource, so
/// that the whole bus is ceiling-analyzed and locked together by
/// RTFM, lock is not needed.
///
/// We use shared_bus only because external libraries consume the
/// whole bus, and we want to use more devices on one bus and keep
/// using existing libraries instead of writing new ones.
pub struct NOPMutex<T>(T);

impl<T> shared_bus::BusMutex<T> for NOPMutex<T> {
    fn create(v: T) -> Self {
        Self(v)
    }

    fn lock<R, F: FnOnce(&T) -> R>(&self, f: F) -> R {
        f(&self.0)
    }
}

// bme280::BME280<
//             shared_bus::proxy::BusProxy<
//                 '_,
//                 MyMutex<
//                     core::cell::RefCell<>,
//                 >,
//             >,
//             asm_delay::AsmDelay,
//         >

// Newtype from HAL crate that implements I²C traits
// type Device<> = lpi2c::LPI2C<T>;

type I2c<SCL, SDA> = BlockingI2c<I2C1, (SCL, SDA)>;
type Device = I2c<PB6<Alternate<OpenDrain>>, PB7<Alternate<OpenDrain>>>;

pub type Proxy<'a> = BusProxy<'a, NOPMutex<RefCell<Device>>, Device>;
pub type Manager = BusManager<NOPMutex<RefCell<Device>>, Device>;

// Device types
type BME<'a> = bme280::BME280<Proxy<'a>, AsmDelay>;
type DISP<'a> = GraphicsMode<I2cInterface<Proxy<'a>>>;

// fn init_manager<T>(i2c: T) -> Manager
// where
//     T: Deref<Target = lpi2c::RegisterBlock>,
// {
// Initializing the peripheral - skipped for brevity
// Manager::new(i2c.into())
// }

/// Structure that holds bus and all its devices together. Manager is
/// inside the struct as an attempt to keep the lifetime synchronized.
pub struct Bus
// where
//     T: Deref<Target = lpi2c::RegisterBlock>,
{
    _manager: Manager,

    // We use 'static private struct members and then downcast them to
    // shorter lifetimes in public accessor methods.  Trick described
    // in https://stackoverflow.com/a/33203128/16390
    bme: BME<'static>,
    disp: DISP<'static>,
}

// I'm not 100% confident in what I'm doing here, but Bus owns the
// Manager (which has consumed the Device, which in turn has consumed
// the Peripheral) and all the devices, and doesn't allow safe access
// to the peripheral or manager, so it seems safe to send across
// threads, right?
unsafe impl Send for Bus
// where T: Deref<Target = lpi2c::RegisterBlock>
{
}

impl Bus
// where
//     T: Deref<Target = lpi2c::RegisterBlock>,
{
    /// Returns new I²C bus with all the devices used by project.
    pub fn new(i2c: Device) -> Self {
        let manager = Manager::new(i2c);
        let delay = AsmDelay::new(bitrate::U32BitrateExt::mhz(72));
        let bme: BME<'static> = unsafe { transmute(BME280::new_primary(manager.acquire(), delay)) };
        let disp: DISP<'static> = unsafe {
            let disp: GraphicsMode<_> = Builder::new()
                .size(DisplaySize::Display128x32)
                .connect_i2c(manager.acquire())
                .into();

            transmute(disp)
        };

        Self {
            _manager: manager,
            bme: bme,
            disp: disp,
        }
    }

    // pub fn bme<'a>(&'a self) -> &'a BME<'a> {
    //     &self.bme
    // }

    // pub fn bme_mut<'a>(&'a mut self) -> &'a mut BME<'a> {
    //     unsafe { transmute(&mut self.bme) }
    // }

    // pub fn disp<'a>(&'a self) -> &'a DISP<'a> {
    //     &self.disp
    // }

    pub fn devices_mut<'a>(&'a mut self) -> (&'a mut DISP<'a>, &'a mut BME<'a>) {
        (unsafe { transmute(&mut self.disp) }, unsafe {
            transmute(&mut self.bme)
        })
    }
}
