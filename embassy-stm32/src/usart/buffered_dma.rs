use futures::future::poll_fn;

use super::*;

const POISON_ADDR: usize = 0xFFFF_FFFF;

#[derive(Copy, Clone, PartialEq, Eq, defmt::Format)]
pub enum ProcessStatus {
    Continue,
    Stop,
}

pub struct BufferedUartDmaRx<'d, T: BasicInstance, RxDma> {
    _peri: PeripheralRef<'d, T>,
    rx_dma: PeripheralRef<'d, RxDma>,
}

impl<'d, T: BasicInstance, RxDma> BufferedUartDmaRx<'d, T, RxDma>
where
    RxDma: crate::usart::RxDma<T>,
{
    pub fn new(
        peri: impl Peripheral<P = T> + 'd,
        irq: impl Peripheral<P = T::Interrupt> + 'd,
        rx: impl Peripheral<P = impl RxPin<T>> + 'd,
        rx_dma: impl Peripheral<P = RxDma> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(peri, irq, rx, rx_dma);

        T::enable();
        T::reset();
        let pclk_freq = T::frequency();

        // TODO: better calculation, including error checking and OVER8 if possible.
        let div = (pclk_freq.0 + (config.baudrate / 2)) / config.baudrate * T::MULTIPLIER;

        let r = T::regs();

        unsafe {
            rx.set_as_af(rx.af_num(), AFType::Input);

            r.cr2().write(|_w| {});
            r.cr3().write(|w| {
                // enable Error Interrupt: (Frame error, Noise error, Overrun error)
                w.set_eie(true);
            });
            r.brr().write_value(regs::Brr(div));
            r.cr1().write(|w| {
                // enable uart
                w.set_ue(true);
                // enable receiver
                w.set_re(true);
                // configure word size
                w.set_m0(if config.parity != Parity::ParityNone {
                    vals::M0::BIT9
                } else {
                    vals::M0::BIT8
                });
                // configure parity
                w.set_pce(config.parity != Parity::ParityNone);
                w.set_ps(match config.parity {
                    Parity::ParityOdd => vals::Ps::ODD,
                    Parity::ParityEven => vals::Ps::EVEN,
                    _ => vals::Ps::EVEN,
                });
            });
        }

        irq.set_handler(Self::on_interrupt);
        irq.unpend();
        irq.enable();

        // create state once!
        let _s = T::state();

        Self { _peri: peri, rx_dma }
    }

    fn on_interrupt_dma(_: *mut ()) {
        defmt::error!("== on_interrupt_dma called");
    }

    // fn on_interrupt_dma(&mut self) {
    //     defmt::error!("== on_interrupt_dma called");
    // }

    fn on_interrupt(_: *mut ()) {
        defmt::trace!("** on_interrupt called!");

        let r = T::regs();
        let s = T::state();

        let (sr, cr1, cr3) = unsafe {
            let sr = sr(r).read();
            // This read also clears the error and idle interrupt flags on v1.
            rdr(r).read_volatile();
            clear_interrupt_flags(r, sr);

            let cr1 = r.cr1().read();
            let cr3 = r.cr3().read();

            (sr, cr1, cr3)
        };

        defmt::trace!("** sr: {:b} - cr1: {:b} - cr3: {:b}", sr.0, cr1.0, cr3.0);

        if cr1.idleie() && sr.idle() {
            defmt::info!("** IDLE");
            s.rx_waker.wake();
        }

        if cr1.rxneie() && sr.rxne() {
            defmt::info!("** RXNE");
            s.rx_waker.wake();
        }
    }

    pub async fn read_continuous<F>(
        &mut self,
        buffer0: &mut [u8],
        buffer1: &mut [u8],
        process: &mut F,
    ) -> Result<(), Error>
    where
        F: FnMut(&[u8]) -> ProcessStatus,
    {
        assert_eq!(buffer0.len(), buffer1.len());
        let buffer_len = buffer0.len();

        let r = T::regs();

        let ch = &mut self.rx_dma;
        let request = ch.request();

        ch.set_handler(Self::on_interrupt_dma);

        // ch.set_handler(|p| unsafe {
        //     let me = &mut *(p as *mut Self);
        //     me.on_interrupt_dma();
        // });

        // let me_ptr: *mut Self = self as *mut Self;

        // let me_ptr: *mut Self = unsafe { core::mem::transmute(&self) };

        // ch.set_handler_context(me_ptr as *mut ());

        unsafe {
            ch.start_double_buffered_read(
                request,
                rdr(r),
                buffer0.as_mut_ptr(),
                buffer1.as_mut_ptr(),
                buffer_len,
                Default::default(),
            );

            r.cr1().modify(|w| {
                // disable RXNE interrupt
                w.set_rxneie(false);
                // enable parity interrupt if not ParityNone
                w.set_peie(w.pce());
                // enable idle line detection
                w.set_idleie(true);
            });

            r.cr3().modify(|w| {
                // enable Error Interrupt: (Frame error, Noise error, Overrun error)
                w.set_eie(true);
                // enable DMA Rx Request
                w.set_dmar(true);
            });

            compiler_fence(Ordering::SeqCst);
        }

        let mut buffer_index = 0;

        let mut start = 0;

        poll_fn(move |cx| -> Poll<Result<(), Error>> {
            defmt::trace!("------------------------------------------");
            let s = T::state();

            ch.set_waker(cx.waker());
            s.rx_waker.register(cx.waker());

            let is_buffer0_accessible = unsafe { ch.is_buffer0_accessible() };

            if buffer_index == 0 {
                if is_buffer0_accessible {
                    // TC M0AR -> M1AR

                    // protect buffer0 from beeing modified by DMA
                    unsafe {
                        ch.set_buffer0(POISON_ADDR as *mut u32);
                    }

                    defmt::trace!("TC M0AR -> M1AR");

                    buffer_index = 1;

                    let status = process(&buffer0[start..]);

                    // done processing data in buffer0
                    // queue buffer0 as the next dma buffer
                    unsafe {
                        ch.set_buffer0(buffer0.as_mut_ptr());
                    }

                    defmt::trace!("status: {}", status);

                    start = 0;

                    // check if there is more data available in buffer1
                    let n = buffer_len - (ch.remaining_transfers() as usize) - start;

                    if n > 0 {
                        // buffer1 slice [start; start+n[ has just been written by DMA so it is safe
                        // to give it to user for processing.
                        // at some point TC will be raised and further protect the entire buffer1 from DMA
                        let status = process(&buffer1[start..start + n]);

                        defmt::trace!("status: {}", status);

                        start += n;
                    }
                } else {
                    let n = buffer_len - (ch.remaining_transfers() as usize) - start;

                    defmt::trace!("IDLE in M0AR - n: {}", n);

                    if n > 0 {
                        // buffer0 slice [start; start+n[ has just been written by DMA so it is safe
                        // to give it to user for processing.
                        // at some point TC will be raised and further protect the entire buffer0 from DMA
                        let status = process(&buffer0[start..start + n]);

                        defmt::trace!("status: {}", status);

                        start += n;
                    }
                }
            } else if buffer_index == 1 {
                if !is_buffer0_accessible {
                    // TC M1AR -> M0AR

                    // protect buffer1 from beeing modified by DMA
                    unsafe {
                        ch.set_buffer1(POISON_ADDR as *mut u32);
                    }

                    defmt::trace!("TC M1AR -> M0AR");

                    buffer_index = 0;

                    let status = process(&buffer1[start..]);

                    // done processing data in buffer1
                    // queue buffer1 as the next dma buffer
                    unsafe {
                        ch.set_buffer1(buffer1.as_mut_ptr());
                    }

                    defmt::trace!("status: {}", status);

                    start = 0;

                    // check if there is more data available in buffer0
                    let n = buffer_len - (ch.remaining_transfers() as usize) - start;

                    if n > 0 {
                        // buffer0 slice [start; start+n[ has just been written by DMA so it is safe
                        // to give it to user for processing.
                        // at some point TC will be raised and further protect the entire buffer1 from DMA
                        let status = process(&buffer0[start..start + n]);

                        defmt::trace!("status: {}", status);

                        start += n;
                    }
                } else {
                    let remaining_transfers = ch.remaining_transfers() as usize;

                    defmt::trace!("remaining_transfers: {}", remaining_transfers);

                    let n = buffer_len - remaining_transfers - start;

                    defmt::trace!("IDLE in M1AR - n: {}", n);

                    if n > 0 {
                        // buffer1 slice [start; start+n[ has just been written by DMA so it is safe
                        // to give it to user for processing.
                        // at some point TC will be raised and further protect the entire buffer1 from DMA
                        let status = process(&buffer1[start..start + n]);

                        defmt::trace!("status: {}", status);

                        start += n;
                    }
                }
            }

            assert!(start < buffer_len);

            defmt::info!("start: {}", start);
            defmt::info!("buffer0: {}", buffer0);
            defmt::info!("buffer1: {}", buffer1);

            Poll::Pending
        })
        .await?;

        Ok(())
    }
}
