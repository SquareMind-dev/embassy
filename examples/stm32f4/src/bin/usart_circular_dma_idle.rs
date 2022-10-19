#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::interrupt;
use embassy_stm32::usart::{BufferedUartDmaRx, Config, ProcessStatus, State};
use embassy_time::{Duration, Timer};
use embedded_io::asynch::BufRead;
use {defmt_rtt as _, panic_probe as _};

// 0123456789
// abcdefghijkl
// abcdefghijklm

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World!");

    let config = Config::default();

    // let mut state = State::new();
    let irq = interrupt::take!(USART2);
    let mut rx_buf0 = [0u8; 8];
    let mut rx_buf1 = [0u8; 8];
    let mut rx = BufferedUartDmaRx::new(p.USART2, irq, p.PA3, p.DMA1_CH5, config);

    if let Err(e) = rx
        .read_continuous(&mut rx_buf0, &mut rx_buf1, &mut |buf| {
            defmt::warn!("Polling lambda called with buf: {}", buf);

            ProcessStatus::Continue
        })
        .await
    {
        defmt::error!("Error in read_continuous: {}", e);
    }

    loop {
        // let buf = buf_usart.fill_buf().await.unwrap();
        // info!("Received: {}", buf);

        // // Read bytes have to be explicitly consumed, otherwise fill_buf() will return them again
        // let n = buf.len();
        // buf_usart.consume(n);
        defmt::info!("rx_buf0: {}", rx_buf0);
        defmt::info!("rx_buf1: {}", rx_buf1);
        Timer::after(Duration::from_secs(1)).await;
    }
}
