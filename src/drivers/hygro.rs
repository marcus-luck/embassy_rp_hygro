
#[allow(dead_code)]

use embassy_time::{Duration, Timer};
use embassy_rp::i2c::{self};
use embassy_rp::peripherals::I2C1;
use embedded_hal_async::i2c::I2c;

const ADDR: u8 = 0x40; // default addr
const TMP: u8 = 0x00;
const HUM: u8 = 0x01;
const CONFIG: u8 = 0x02;
const DEVICEID: u8 = 0xFF;

pub struct Hygro<'a> {
    i2c: i2c::I2c<'a, I2C1, i2c::Async>,
    buff: [u8; 2],
}
impl<'a> Hygro<'a> {

    pub fn new(i2c: i2c::I2c<'a, I2C1, i2c::Async>) -> Self {
        Self {
            i2c: i2c,
            buff: [0; 2],
        }
    }

    pub async fn init(&mut self) {
        self.i2c.write(ADDR, &[CONFIG, 0x00]).await.unwrap();
    }

    async fn read_internal(&mut self, reg: u8) {
        let addr = u16::from_le_bytes([ADDR, 0x00]);
        self.i2c.write_async(addr, [reg]).await.unwrap();
        Timer::after(Duration::from_millis(7)).await;
        self.i2c.read_async(addr, &mut self.buff).await.unwrap();
    }

    /// Read the temperature from the Hygro
    /// 
    /// The temperature is read out over i2c from the temperature address.
    /// The data is read out as two bytes, the return is big endian.
    /// The temperature is caluclated: temp_c = (raw_t / 2^16) * 165 - 40
    pub async fn temperature(&mut self) -> f32 {
        self.read_internal(TMP).await;
        let mut deg_c = (u16::from_be_bytes(self.buff) as f32 / 0x10000 as f32) as f32;
        deg_c *= 165.0;
        deg_c -= 40.0;
        deg_c
    }

    pub async fn humidity(&mut self) -> f32 {
        self.read_internal(HUM).await;
        let mut hum = (u16::from_be_bytes(self.buff) as f32 / 0x10000 as f32) as f32;
        hum *= 100.0;
        hum
    }
}