use cortex_m_semihosting::hprintln;
use embedded_hal::serial::{Read, Write};
use heapless::consts::U16;
use heapless::Vec;
use nb::block;

pub struct Mhz19<TX: Write<u8>, RX: Read<u8>> {
    tx: TX,
    rx: RX,
}

impl<TX: Write<u8>, RX: Read<u8>> Mhz19<TX, RX> {
    pub fn new(tx: TX, rx: RX) -> Self {
        Mhz19 { tx: tx, rx: rx }
    }

    pub fn send_cmd(&mut self, bytes: &[u8; 9], response_length: usize) -> [u8; 9] {
        let mut tx = &mut self.tx;
        let mut rx = &mut self.rx;
        for byte in bytes.iter() {
            block!(tx.write(*byte));
        }
        let mut buf: [u8; 9] = [0; 9];
        for i in 0..(response_length) {
            if let Ok(byte) = block!(rx.read()) {
                buf[i] = byte;
            }
        }
        buf
    }

    pub fn set_automatic_baseline_correction(&mut self, enabled: bool) {
        self.send_cmd(&mh_z19::set_automatic_baseline_correction(1, enabled), 9);
    }

    pub fn read_gas_concentration(&mut self) -> Result<u32, mh_z19::MHZ19Error> {
        let result = self.send_cmd(&mh_z19::read_gas_concentration(1), 9);

        mh_z19::parse_gas_contentration_ppm(&result[..])
    }
}
