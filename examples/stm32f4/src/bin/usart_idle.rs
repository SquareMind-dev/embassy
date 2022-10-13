#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::dma::NoDma;
use embassy_stm32::interrupt;
use embassy_stm32::peripherals::{DMA2_CH7, USART1};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{Config, Uart, UartWithIdle};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

const INCOMMING_DATA_SIZE: usize = 17;
const BUFFER_SIZE: usize = 16;
const MAIN_BUFFER_SIZE: usize = 32;
const FIRST_VALUE: u8 = 0;

pub fn config() -> embassy_stm32::Config {
    let mut config = embassy_stm32::Config::default();

    config.rcc.hse = Some(Hertz(8_000_000));
    config.rcc.bypass_hse = false;
    config.rcc.pll48 = false;
    config.rcc.sys_ck = Some(Hertz(180_000_000));
    config.rcc.hclk = Some(Hertz(180_000_000));
    config.rcc.pclk1 = Some(Hertz(45_000_000));
    config.rcc.pclk2 = Some(Hertz(90_000_000));

    config
}

fn fill_ref_buffer(buffer: &mut [u8]) {
    let mut ch = FIRST_VALUE;

    for i in 0..buffer.len() {
        buffer[i] = ch;
        ch += 1;
    }
}

#[embassy_executor::task]
async fn emitter_task(mut uart: Uart<'static, USART1, DMA2_CH7, NoDma>) {
    let mut buffer = [0; INCOMMING_DATA_SIZE];

    fill_ref_buffer(&mut buffer);

    loop {
        Timer::after(Duration::from_millis(500)).await;

        defmt::info!("Starting emitting data...");

        uart.write(&buffer).await.unwrap();
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = embassy_stm32::init(config());
    info!("Hello World!");

    let mut config = Config::default();
    config.baudrate = 115200;

    let detect_previous_overrun = true;

    let irq = interrupt::take!(USART3);
    let mut usart = UartWithIdle::new(
        p.USART3,
        irq,
        p.PC5,
        p.PB10,
        NoDma,
        p.DMA1_CH1,
        config,
        detect_previous_overrun,
    );

    let emitter = Uart::new(p.USART1, p.PA10, p.PA9, p.DMA2_CH7, NoDma, config);

    let mut buffer: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];

    let mut main_buffer: [u8; MAIN_BUFFER_SIZE] = [0; MAIN_BUFFER_SIZE];

    defmt::assert!(BUFFER_SIZE <= MAIN_BUFFER_SIZE);

    spawner.spawn(emitter_task(emitter)).unwrap();

    let mut first_value = FIRST_VALUE;

    let mut new_pos = 0;

    loop {
        let received_bytes = usart.read_until_idle(&mut buffer).await.unwrap();

        // info!("Received {} bytes: {}", received_bytes, buffer[..received_bytes]);

        if received_bytes > 0 {
            if buffer[0] != first_value || buffer[received_bytes - 1] != first_value + (received_bytes - 1) as u8 {
                defmt::panic!("missing data: received {}", buffer[..received_bytes]);
            }
        } else {
            defmt::panic!("Received 0 bytes");
        }

        first_value += received_bytes as u8;

        first_value %= INCOMMING_DATA_SIZE as u8;

        // copy data to larger main ring buffer
        let old_pos = new_pos;
        if old_pos + received_bytes > MAIN_BUFFER_SIZE {
            let data_to_copy = MAIN_BUFFER_SIZE - old_pos;
            main_buffer[old_pos..].copy_from_slice(&buffer[..data_to_copy]);
            new_pos = received_bytes - data_to_copy;
            main_buffer[..new_pos].copy_from_slice(&buffer[data_to_copy..received_bytes]);
        } else {
            new_pos = old_pos + received_bytes;
            main_buffer[old_pos..new_pos].copy_from_slice(&buffer[..received_bytes]);
        }

        // info!("Main buffer: {}", main_buffer);

        //Timer::after(Duration::from_millis(1200)).await;
    }
}
