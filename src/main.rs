#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
use embassy_rp::gpio::AnyPin;
use embassy_rp::peripherals::{
    PIN_12,
    PIN_13,
    PIN_18,
    PIN_19,
    DMA_CH0,
    DMA_CH1,
};
use embassy_rp::peripherals::UART0;
use embassy_rp::usb::Out;
use embassy_usb::descriptor::capability_type;
use static_cell::StaticCell;
use cortex_m::peripheral;
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::Peripheral;
use embassy_rp::gpio;
use embassy_rp::uart;
use embassy_rp::bind_interrupts;
use embassy_rp::i2c::{self, Config, InterruptHandler};
use embassy_rp::peripherals::I2C1;
use embassy_rp::peripherals::{PWM_CH1, PIN_3, PWM_CH2, PIN_4};
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c;
use gpio::{Level, Output};
mod drivers;
use drivers::hygro::Hygro;
use embassy_rp::pwm;
use fixed::traits::ToFixed;
use libm::{atanf, powf};

// Concurrency crap
use embassy_sync::mutex;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;

use core::fmt::Write;
use heapless::String;

use {defmt_rtt as _, panic_probe as _};

macro_rules! singleton {
    ($val:expr) => {{
        type T = impl Sized;
        static STATIC_CELL: StaticCell<T> = StaticCell::new();
        STATIC_CELL.init_with(move || $val)
    }};
}

bind_interrupts!(struct Irqs {
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
    UART0_IRQ => uart::InterruptHandler<UART0>;
});

// struct Registers<T> {
//     registers: mutex::Mutex<ThreadModeRawMutex, [T]>,
// }
    
// impl<T> Registers<T> {
//     pub fn with_capacity(cap: usize) -> Self {
//         registers: mutex::Mutex::new([T; cap])
//     }
// }

static mut REGISTERS: mutex::Mutex<ThreadModeRawMutex, [f32; 2]> = mutex::Mutex::new([0f32; 2]);


#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Hello, world!");
    let p = embassy_rp::init(Default::default());

    // let _led_h = spawner.spawn(pulse_led_2(pw2, pn2)).unwrap();   
    // let mut led_on = Output::new(p.PIN_2, Level::Low);
    info!("Time to spawn a temp reader");
    let sda = p.PIN_18;
    let scl = p.PIN_19;
    let i2c1 = p.I2C1;
    spawner.spawn(read_temp(i2c1, sda, scl)).unwrap();


    let pw = p.PWM_CH1;
    let pn = p.PIN_3;
    spawner.spawn(pulse_led_1(pw, pn)).unwrap();

    // Spawn a uart task to hadnle register requests.
    let tx = p.PIN_12;
    let rx = p.PIN_13;
    let uart0 = p.UART0;
    let dma0 = p.DMA_CH0;
    let dma1 = p.DMA_CH1;
    spawner.spawn(handle_requests(uart0, tx, rx, dma0, dma1)).unwrap();

}

/// get register value
/// 
/// takes an address number and returns the value of the register.
fn get_register(addr: usize) -> f32 {
    // validate the address
    match addr {
        0..=1 => {
            unsafe {
                let r = REGISTERS.get_mut();
                return r[addr].clone()
            }
        },
        _ => return 0f32,
    }

}

/// set register value
/// 
/// takes an address number and a value and sets the register to that value.
fn set_register(addr: usize, val: f32) {
    // validate the address
    match addr {
        0..=1 => {
            unsafe {
                let r = REGISTERS.get_mut();
                r[addr] = val;
            }
        },
        _ => (),
    }
}

#[embassy_executor::task]
async fn read_temp(i2c1: I2C1, sda: PIN_18, scl: PIN_19) {

    info!("set up i2c ");
    let i2c = i2c::I2c::new_async(i2c1, scl, sda, Irqs, Config::default());


    // Setup i2c:
    let mut sensor = Hygro::new(i2c);
    sensor.init().await;

    loop {

        let t = sensor.temperature().await;
        let rh = sensor.humidity().await;
        set_register(0, t);
        set_register(1, rh);

        Timer::after(Duration::from_millis(1000)).await;
    }

}

#[embassy_executor::task]
async fn handle_requests(uart0: UART0, tx: PIN_12, rx: PIN_13, tx_dma: DMA_CH0, rx_dma: DMA_CH1) {
    let mut config = uart::Config::default();
    config.baudrate = 9600;
    config.data_bits = uart::DataBits::DataBits8;
    config.stop_bits = uart::StopBits::STOP1;
    config.parity = uart::Parity::ParityNone;
    let mut uart = uart::Uart::new(uart0, tx, rx, Irqs, tx_dma, rx_dma, config);

    let mut data: String<64> = String::new(); // 32 byte string buffer
    let mut t: f32 = 0f32;
    let mut rh: f32 = 0f32;

    loop {
        let buffer: &mut [u8] = &mut [0u8; 8];
        let n = uart.read(buffer).await.unwrap();


        let b: [u8; 2] = buffer[0..2].try_into().unwrap();

        let addr: u16 = u16::from_le_bytes(b);
        match addr {
            0x00 => {
                t = get_register(0);
                core::writeln!(&mut data, "{:02.03}", t).unwrap();
            },
            0x01 => {
                rh = get_register(1);
                core::writeln!(&mut data, "{:02.03}", rh).unwrap();
            },
            0x02 => {
                t = get_register(0);
                rh = get_register(1);
                let wb = t * atanf(0.151977f32 * powf((rh + 8.313659f32), 0.5f32)) + atanf(t + rh) - atanf(rh - 1.676331f32) + 0.00391838f32*(powf(rh, 1.5f32)) * atanf(0.023101f32 * rh) - 4.686035f32;
                core::writeln!(&mut data, "{:02.03}", wb).unwrap();
            },
            _ => {
                core::writeln!(&mut data, "Invalid address").unwrap();

            }
        }
        uart.write(&data.as_bytes()).await.unwrap();
        data.clear();

        Timer::after(Duration::from_millis(200)).await;
    }


}


/// LED pulse task 1.
/// 
/// This function is designed to not be generic to make it easier to adapt.
/// It takes a specific `PWM` channel and `PIN` combination, see Pico documentation.
/// 
#[embassy_executor::task]
async fn pulse_led_1(pwm_ch1: PWM_CH1, pin3: PIN_3) {
// async fn blink_led(pwm: &'static mut pwm::Pwm<'static, PWM_CH1>, pin: AnyPin) {

    let mut c: pwm::Config = Default::default();
    let mut pw = pwm::Pwm::new_output_b(pwm_ch1, pin3, c.clone());

    c.top = 12000; //0x8000;
    c.divider = 240u16.to_fixed(); // Seems like this is implmented with a u8 as the largest divider
    c.compare_b = 8;

    info!("Done setting up, running loop");
    let mut going_up: bool = true;
    let mut fade: u16 = 0;
    loop {
        // LEd PWM
        if going_up {
            fade+=1;
            if fade == 255 {
                going_up = false;
            }
        } else {
            fade-=1;
            if fade == 0 {
                going_up = true;
            }
        }

        c.compare_b = (fade as f32 * fade as f32 * 12000.0f32 / 65535.0f32) as u16;

        pw.set_config(&c);
        Timer::after(Duration::from_millis(10)).await;
    }
}