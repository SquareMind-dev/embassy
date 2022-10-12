#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::dma::NoDma;
use embassy_stm32::interrupt;
use embassy_stm32::usart::{Config, UartWithIdle};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World!");

    let config = Config::default();
    let irq = interrupt::take!(USART2);
    let mut usart = UartWithIdle::new(p.USART2, irq, p.PA3, p.PA2, NoDma, p.DMA1_CH5, config);

    const BUFFER_SIZE: usize = 8;
    let mut buffer: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];

    const MAIN_BUFFER_SIZE: usize = 16;
    let mut main_buffer: [u8; MAIN_BUFFER_SIZE] = [0; MAIN_BUFFER_SIZE];

    defmt::assert!(BUFFER_SIZE <= MAIN_BUFFER_SIZE);

    let mut new_pos = 0;

    // abcdefghijklmnopqrt
    // 12345678
    // 1234567
    // 123456
    // 12345
    // 1234
    // 123
    // 12
    loop {
        let received_bytes = usart.read_until_idle(&mut buffer).await.unwrap();

        info!("Received {} bytes: {:X}", received_bytes, buffer[..received_bytes]);

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

        info!("Main buffer: {:X}", main_buffer);
    }
}
