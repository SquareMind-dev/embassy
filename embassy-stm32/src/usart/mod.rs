#![macro_use]

use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use atomic_polyfill::{compiler_fence, Ordering};
use embassy_cortex_m::interrupt::InterruptExt;
use embassy_hal_common::{into_ref, PeripheralRef};

use crate::dma::NoDma;
use crate::gpio::sealed::AFType;
#[cfg(any(lpuart_v1, lpuart_v2))]
use crate::pac::lpuart::{regs, vals, Lpuart as Regs};
#[cfg(not(any(lpuart_v1, lpuart_v2)))]
use crate::pac::usart::{regs, vals, Usart as Regs};
use crate::{peripherals, Peripheral};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DataBits {
    DataBits8,
    DataBits9,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Parity {
    ParityNone,
    ParityEven,
    ParityOdd,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum StopBits {
    #[doc = "1 stop bit"]
    STOP1,
    #[doc = "0.5 stop bits"]
    STOP0P5,
    #[doc = "2 stop bits"]
    STOP2,
    #[doc = "1.5 stop bits"]
    STOP1P5,
}

#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Config {
    pub baudrate: u32,
    pub data_bits: DataBits,
    pub stop_bits: StopBits,
    pub parity: Parity,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            baudrate: 115200,
            data_bits: DataBits::DataBits8,
            stop_bits: StopBits::STOP1,
            parity: Parity::ParityNone,
        }
    }
}

/// Serial error
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Framing error
    Framing,
    /// Noise error
    Noise,
    /// RX buffer overrun
    Overrun,
    /// Parity check error
    Parity,
}

pub struct Uart<'d, T: BasicInstance, TxDma = NoDma, RxDma = NoDma> {
    phantom: PhantomData<&'d mut T>,
    tx: UartTx<'d, T, TxDma>,
    rx: UartRx<'d, T, RxDma>,
}

pub struct UartTx<'d, T: BasicInstance, TxDma = NoDma> {
    phantom: PhantomData<&'d mut T>,
    tx_dma: PeripheralRef<'d, TxDma>,
}

pub struct UartRx<'d, T: BasicInstance, RxDma = NoDma> {
    phantom: PhantomData<&'d mut T>,
    rx_dma: PeripheralRef<'d, RxDma>,
}

impl<'d, T: BasicInstance, TxDma> UartTx<'d, T, TxDma> {
    fn new(tx_dma: PeripheralRef<'d, TxDma>) -> Self {
        Self {
            tx_dma,
            phantom: PhantomData,
        }
    }

    pub async fn write(&mut self, buffer: &[u8]) -> Result<(), Error>
    where
        TxDma: crate::usart::TxDma<T>,
    {
        let ch = &mut self.tx_dma;
        let request = ch.request();
        unsafe {
            T::regs().cr3().modify(|reg| {
                reg.set_dmat(true);
            });
        }
        // If we don't assign future to a variable, the data register pointer
        // is held across an await and makes the future non-Send.
        let transfer = crate::dma::write(ch, request, buffer, tdr(T::regs()));
        transfer.await;
        Ok(())
    }

    pub fn blocking_write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        unsafe {
            let r = T::regs();
            for &b in buffer {
                while !sr(r).read().txe() {}
                tdr(r).write_volatile(b);
            }
        }
        Ok(())
    }

    pub fn blocking_flush(&mut self) -> Result<(), Error> {
        unsafe {
            let r = T::regs();
            while !sr(r).read().tc() {}
        }
        Ok(())
    }
}

impl<'d, T: BasicInstance, RxDma> UartRx<'d, T, RxDma> {
    fn new(rx_dma: PeripheralRef<'d, RxDma>) -> Self {
        Self {
            rx_dma,
            phantom: PhantomData,
        }
    }

    pub async fn read(&mut self, buffer: &mut [u8]) -> Result<(), Error>
    where
        RxDma: crate::usart::RxDma<T>,
    {
        let ch = &mut self.rx_dma;
        let request = ch.request();
        unsafe {
            T::regs().cr3().modify(|reg| {
                reg.set_dmar(true);
            });
        }
        // If we don't assign future to a variable, the data register pointer
        // is held across an await and makes the future non-Send.
        let transfer = crate::dma::read(ch, request, rdr(T::regs()), buffer);
        transfer.await;
        Ok(())
    }

    pub fn nb_read(&mut self) -> Result<u8, nb::Error<Error>> {
        let r = T::regs();
        unsafe {
            let sr = sr(r).read();
            if sr.pe() {
                rdr(r).read_volatile();
                Err(nb::Error::Other(Error::Parity))
            } else if sr.fe() {
                rdr(r).read_volatile();
                Err(nb::Error::Other(Error::Framing))
            } else if sr.ne() {
                rdr(r).read_volatile();
                Err(nb::Error::Other(Error::Noise))
            } else if sr.ore() {
                rdr(r).read_volatile();
                Err(nb::Error::Other(Error::Overrun))
            } else if sr.rxne() {
                Ok(rdr(r).read_volatile())
            } else {
                Err(nb::Error::WouldBlock)
            }
        }
    }

    pub fn blocking_read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        unsafe {
            let r = T::regs();
            for b in buffer {
                loop {
                    let sr = sr(r).read();
                    if sr.pe() {
                        rdr(r).read_volatile();
                        return Err(Error::Parity);
                    } else if sr.fe() {
                        rdr(r).read_volatile();
                        return Err(Error::Framing);
                    } else if sr.ne() {
                        rdr(r).read_volatile();
                        return Err(Error::Noise);
                    } else if sr.ore() {
                        rdr(r).read_volatile();
                        return Err(Error::Overrun);
                    } else if sr.rxne() {
                        break;
                    }
                }
                *b = rdr(r).read_volatile();
            }
        }
        Ok(())
    }
}

impl<'d, T: BasicInstance, TxDma, RxDma> Uart<'d, T, TxDma, RxDma> {
    pub fn new(
        _inner: impl Peripheral<P = T> + 'd,
        rx: impl Peripheral<P = impl RxPin<T>> + 'd,
        tx: impl Peripheral<P = impl TxPin<T>> + 'd,
        tx_dma: impl Peripheral<P = TxDma> + 'd,
        rx_dma: impl Peripheral<P = RxDma> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(_inner, rx, tx, tx_dma, rx_dma);

        T::enable();
        T::reset();
        let pclk_freq = T::frequency();

        // TODO: better calculation, including error checking and OVER8 if possible.
        let div = (pclk_freq.0 + (config.baudrate / 2)) / config.baudrate * T::MULTIPLIER;

        let r = T::regs();

        unsafe {
            rx.set_as_af(rx.af_num(), AFType::Input);
            tx.set_as_af(tx.af_num(), AFType::OutputPushPull);

            r.cr2().write(|_w| {});
            r.cr3().write(|_w| {});
            r.brr().write_value(regs::Brr(div));
            r.cr1().write(|w| {
                w.set_ue(true);
                w.set_te(true);
                w.set_re(true);
                w.set_m0(if config.parity != Parity::ParityNone {
                    vals::M0::BIT9
                } else {
                    vals::M0::BIT8
                });
                w.set_pce(config.parity != Parity::ParityNone);
                w.set_ps(match config.parity {
                    Parity::ParityOdd => vals::Ps::ODD,
                    Parity::ParityEven => vals::Ps::EVEN,
                    _ => vals::Ps::EVEN,
                });
            });
        }

        Self {
            tx: UartTx::new(tx_dma),
            rx: UartRx::new(rx_dma),
            phantom: PhantomData {},
        }
    }

    pub async fn write(&mut self, buffer: &[u8]) -> Result<(), Error>
    where
        TxDma: crate::usart::TxDma<T>,
    {
        self.tx.write(buffer).await
    }

    pub fn blocking_write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        self.tx.blocking_write(buffer)
    }

    pub fn blocking_flush(&mut self) -> Result<(), Error> {
        self.tx.blocking_flush()
    }

    pub async fn read(&mut self, buffer: &mut [u8]) -> Result<(), Error>
    where
        RxDma: crate::usart::RxDma<T>,
    {
        self.rx.read(buffer).await
    }

    pub fn nb_read(&mut self) -> Result<u8, nb::Error<Error>> {
        self.rx.nb_read()
    }

    pub fn blocking_read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        self.rx.blocking_read(buffer)
    }

    /// Split the Uart into a transmitter and receiver, which is
    /// particuarly useful when having two tasks correlating to
    /// transmitting and receiving.
    pub fn split(self) -> (UartTx<'d, T, TxDma>, UartRx<'d, T, RxDma>) {
        (self.tx, self.rx)
    }
}

pub struct UartWithIdle<'d, T: BasicInstance, TxDma = NoDma, RxDma = NoDma> {
    tx: UartTx<'d, T, TxDma>,
    rx: UartRxWithIdle<'d, T, RxDma>,
}

pub struct UartRxWithIdle<'d, T: BasicInstance, RxDma = NoDma> {
    _peri: PeripheralRef<'d, T>,
    rx: UartRx<'d, T, RxDma>,
}

impl<'d, T: BasicInstance, TxDma, RxDma> UartWithIdle<'d, T, TxDma, RxDma> {
    pub fn new(
        peri: impl Peripheral<P = T> + 'd,
        irq: impl Peripheral<P = T::Interrupt> + 'd,
        rx: impl Peripheral<P = impl RxPin<T>> + 'd,
        tx: impl Peripheral<P = impl TxPin<T>> + 'd,
        tx_dma: impl Peripheral<P = TxDma> + 'd,
        rx_dma: impl Peripheral<P = RxDma> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(peri, irq, rx, tx, tx_dma, rx_dma);

        T::enable();
        T::reset();
        let pclk_freq = T::frequency();

        // TODO: better calculation, including error checking and OVER8 if possible.
        let div = (pclk_freq.0 + (config.baudrate / 2)) / config.baudrate * T::MULTIPLIER;

        let r = T::regs();

        unsafe {
            rx.set_as_af(rx.af_num(), AFType::Input);
            tx.set_as_af(tx.af_num(), AFType::OutputPushPull);

            r.cr2().write(|_w| {});
            r.cr3().write(|_w| {});
            r.brr().write_value(regs::Brr(div));
            r.cr1().write(|w| {
                w.set_ue(true);
                w.set_te(true);
                w.set_re(true);
                w.set_m0(if config.parity != Parity::ParityNone {
                    vals::M0::BIT9
                } else {
                    vals::M0::BIT8
                });
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

        let s = T::state();
        // TODO(gmichel): implement drop
        s.tx_rx_refcount.store(1, Ordering::Relaxed);

        Self {
            tx: UartTx::new(tx_dma),
            rx: UartRxWithIdle {
                _peri: peri,
                rx: UartRx::new(rx_dma),
            },
        }
    }

    fn on_interrupt(_: *mut ()) {
        let r = T::regs();
        let s = T::state();

        // TODO(gmichel): implement interrupt handler correctly
        unsafe {
            let sr = sr(r).read();

            // This read also clears the error and idle interrupt flags on v1.
            let _ = rdr(r).read_volatile();

            clear_interrupt_flags(r, sr);

            trace!("** on_interrupt called. sr: {:b}", sr.0);

            s.sr.store(sr.0, Ordering::SeqCst);

            if sr.idle() {
                trace!("** interrupt IDLE");
                // disable idle line detection
                r.cr1().modify(|w| {
                    w.set_idleie(false);
                });

                s.endrx_waker.wake();
            }
        }

        trace!("** on_interrupt DONE");
    }

    pub fn split(self) -> (UartRxWithIdle<'d, T, RxDma>, UartTx<'d, T, TxDma>) {
        (self.rx, self.tx)
    }

    pub async fn read_until_idle(&mut self, buffer: &mut [u8]) -> Result<usize, Error>
    where
        RxDma: crate::usart::RxDma<T>,
    {
        self.rx.read_until_idle(buffer).await
    }
}

impl<'d, T: BasicInstance, RxDma> UartRxWithIdle<'d, T, RxDma> {
    pub async fn read(&mut self, buffer: &mut [u8]) -> Result<(), Error>
    where
        RxDma: crate::usart::RxDma<T>,
    {
        let r = T::regs();
        unsafe {
            r.cr1().modify(|w| {
                //w.set_rxneie(false);
                w.set_idleie(false);
            });
        }

        compiler_fence(Ordering::SeqCst);

        self.rx.read(buffer).await
    }

    pub fn nb_read(&mut self) -> Result<u8, nb::Error<Error>> {
        let r = T::regs();
        unsafe {
            r.cr1().modify(|w| {
                //w.set_rxneie(false);
                w.set_idleie(false);
            });
        }

        compiler_fence(Ordering::SeqCst);

        self.rx.nb_read()
    }

    pub fn blocking_read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        let r = T::regs();
        unsafe {
            r.cr1().modify(|w| {
                //w.set_rxneie(false);
                w.set_idleie(false);
            });
        }

        compiler_fence(Ordering::SeqCst);

        self.rx.blocking_read(buffer)
    }

    pub async fn read_until_idle(&mut self, buffer: &mut [u8]) -> Result<usize, Error>
    where
        RxDma: crate::usart::RxDma<T>,
    {
        trace!("------------------------------------------");
        // TODO(gmichel): we could return an Error::BufferTooLarge
        assert!(buffer.len() > 0 && buffer.len() <= 0xFFFF);

        let r = T::regs();
        unsafe {
            r.cr1().modify(|w| {
                w.set_idleie(true);
            });
        }

        compiler_fence(Ordering::SeqCst);

        let buffer_len = buffer.len();

        let ch = &mut self.rx.rx_dma;
        let request = ch.request();
        unsafe {
            r.cr3().modify(|reg| {
                reg.set_dmar(true);
            });
        }

        unsafe { ch.start_read(request, rdr(T::regs()), buffer, Default::default()) };

        // // If we don't assign future to a variable, the data register pointer
        // // is held across an await and makes the future non-Send.
        // let mut transfer = crate::dma::read(ch, request, rdr(T::regs()), buffer);

        let res = poll_fn(move |cx| {
            let s = T::state();

            let usart_sr = regs::Sr(s.sr.swap(0, Ordering::SeqCst));
            trace!("poll_fn sr: {:b}", usart_sr.0);

            if usart_sr.rxne() {
                trace!("RXNE set");
                if usart_sr.pe() {
                    // stop dma transfer
                    trace!("Stop DMA Parity");
                    ch.request_stop();
                    while ch.is_running() {}

                    return Poll::Ready(Err(Error::Parity));
                }
                if usart_sr.fe() {
                    // stop dma transfer
                    trace!("Stop DMA Framing");
                    ch.request_stop();
                    while ch.is_running() {}

                    return Poll::Ready(Err(Error::Framing));
                }
                if usart_sr.ne() {
                    // stop dma transfer
                    trace!("Stop DMA Noise");
                    ch.request_stop();
                    while ch.is_running() {}

                    return Poll::Ready(Err(Error::Noise));
                }
                if usart_sr.ore() {
                    // stop dma transfer
                    trace!("Stop DMA Overrun");
                    ch.request_stop();
                    while ch.is_running() {}

                    return Poll::Ready(Err(Error::Overrun));
                }
            }

            if usart_sr.idle() {
                trace!("IDLE set");

                // // because IDLE happens before DMA complete,
                // // we may still have 1 byte in DR
                // // so we wait for DMA to copy this byte
                // unsafe { while sr(r).read().rxne() {} }

                // stop dma transfer
                trace!("Stop DMA");
                ch.request_stop();
                while ch.is_running() {}

                let n = buffer_len - (ch.remaining_transfers() as usize);

                return Poll::Ready(Ok(n));
            } else if !ch.is_running() {
                trace!("DMA Completed");
                return Poll::Ready(Ok(buffer_len));
            }

            trace!("Pending");
            ch.set_waker(cx.waker());
            s.endrx_waker.register(cx.waker());

            Poll::Pending
        })
        .await;

        unsafe {
            r.cr1().modify(|w| {
                w.set_idleie(false);
            });
        }

        compiler_fence(Ordering::SeqCst);

        res
    }
}

impl<'a, T: BasicInstance, RxDma> Drop for UartRxWithIdle<'a, T, RxDma> {
    fn drop(&mut self) {
        trace!("uarte rx drop");

        let r = T::regs();

        // TODO(gmichel): add proper drop actions

        let s = T::state();

        drop_tx_rx(&r, &s);
    }
}

pub(crate) fn drop_tx_rx(_r: &crate::pac::usart::Usart, s: &sealed::State) {
    if s.tx_rx_refcount.fetch_sub(1, Ordering::Relaxed) == 1 {
        // Finally we can disable, and we do so for the peripheral
        // i.e. not just rx concerns.
        // TODO(gmichel): implement drop

        trace!("uarte tx and rx drop: done");
    }
}

mod eh02 {
    use super::*;

    impl<'d, T: BasicInstance, RxDma> embedded_hal_02::serial::Read<u8> for UartRx<'d, T, RxDma> {
        type Error = Error;
        fn read(&mut self) -> Result<u8, nb::Error<Self::Error>> {
            self.nb_read()
        }
    }

    impl<'d, T: BasicInstance, TxDma> embedded_hal_02::blocking::serial::Write<u8> for UartTx<'d, T, TxDma> {
        type Error = Error;
        fn bwrite_all(&mut self, buffer: &[u8]) -> Result<(), Self::Error> {
            self.blocking_write(buffer)
        }
        fn bflush(&mut self) -> Result<(), Self::Error> {
            self.blocking_flush()
        }
    }

    impl<'d, T: BasicInstance, TxDma, RxDma> embedded_hal_02::serial::Read<u8> for Uart<'d, T, TxDma, RxDma> {
        type Error = Error;
        fn read(&mut self) -> Result<u8, nb::Error<Self::Error>> {
            self.nb_read()
        }
    }

    impl<'d, T: BasicInstance, TxDma, RxDma> embedded_hal_02::blocking::serial::Write<u8> for Uart<'d, T, TxDma, RxDma> {
        type Error = Error;
        fn bwrite_all(&mut self, buffer: &[u8]) -> Result<(), Self::Error> {
            self.blocking_write(buffer)
        }
        fn bflush(&mut self) -> Result<(), Self::Error> {
            self.blocking_flush()
        }
    }
}

#[cfg(feature = "unstable-traits")]
mod eh1 {
    use super::*;

    impl embedded_hal_1::serial::Error for Error {
        fn kind(&self) -> embedded_hal_1::serial::ErrorKind {
            match *self {
                Self::Framing => embedded_hal_1::serial::ErrorKind::FrameFormat,
                Self::Noise => embedded_hal_1::serial::ErrorKind::Noise,
                Self::Overrun => embedded_hal_1::serial::ErrorKind::Overrun,
                Self::Parity => embedded_hal_1::serial::ErrorKind::Parity,
            }
        }
    }

    impl<'d, T: BasicInstance, TxDma, RxDma> embedded_hal_1::serial::ErrorType for Uart<'d, T, TxDma, RxDma> {
        type Error = Error;
    }

    impl<'d, T: BasicInstance, TxDma> embedded_hal_1::serial::ErrorType for UartTx<'d, T, TxDma> {
        type Error = Error;
    }

    impl<'d, T: BasicInstance, RxDma> embedded_hal_1::serial::ErrorType for UartRx<'d, T, RxDma> {
        type Error = Error;
    }

    impl<'d, T: BasicInstance, RxDma> embedded_hal_nb::serial::Read for UartRx<'d, T, RxDma> {
        fn read(&mut self) -> nb::Result<u8, Self::Error> {
            self.nb_read()
        }
    }

    impl<'d, T: BasicInstance, TxDma> embedded_hal_1::serial::Write for UartTx<'d, T, TxDma> {
        fn write(&mut self, buffer: &[u8]) -> Result<(), Self::Error> {
            self.blocking_write(buffer)
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            self.blocking_flush()
        }
    }

    impl<'d, T: BasicInstance, TxDma> embedded_hal_nb::serial::Write for UartTx<'d, T, TxDma> {
        fn write(&mut self, char: u8) -> nb::Result<(), Self::Error> {
            self.blocking_write(&[char]).map_err(nb::Error::Other)
        }

        fn flush(&mut self) -> nb::Result<(), Self::Error> {
            self.blocking_flush().map_err(nb::Error::Other)
        }
    }

    impl<'d, T: BasicInstance, TxDma, RxDma> embedded_hal_nb::serial::Read for Uart<'d, T, TxDma, RxDma> {
        fn read(&mut self) -> Result<u8, nb::Error<Self::Error>> {
            self.nb_read()
        }
    }

    impl<'d, T: BasicInstance, TxDma, RxDma> embedded_hal_1::serial::Write for Uart<'d, T, TxDma, RxDma> {
        fn write(&mut self, buffer: &[u8]) -> Result<(), Self::Error> {
            self.blocking_write(buffer)
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            self.blocking_flush()
        }
    }

    impl<'d, T: BasicInstance, TxDma, RxDma> embedded_hal_nb::serial::Write for Uart<'d, T, TxDma, RxDma> {
        fn write(&mut self, char: u8) -> nb::Result<(), Self::Error> {
            self.blocking_write(&[char]).map_err(nb::Error::Other)
        }

        fn flush(&mut self) -> nb::Result<(), Self::Error> {
            self.blocking_flush().map_err(nb::Error::Other)
        }
    }
}

#[cfg(all(
    feature = "unstable-traits",
    feature = "nightly",
    feature = "_todo_embedded_hal_serial"
))]
mod eha {
    use core::future::Future;

    use super::*;

    impl<'d, T: BasicInstance, TxDma> embedded_hal_async::serial::Write for UartTx<'d, T, TxDma>
    where
        TxDma: crate::usart::TxDma<T>,
    {
        type WriteFuture<'a> = impl Future<Output = Result<(), Self::Error>> + 'a where Self: 'a;

        fn write<'a>(&'a mut self, buf: &'a [u8]) -> Self::WriteFuture<'a> {
            self.write(buf)
        }

        type FlushFuture<'a> = impl Future<Output = Result<(), Self::Error>> + 'a where Self: 'a;

        fn flush<'a>(&'a mut self) -> Self::FlushFuture<'a> {
            async move { Ok(()) }
        }
    }

    impl<'d, T: BasicInstance, RxDma> embedded_hal_async::serial::Read for UartRx<'d, T, RxDma>
    where
        RxDma: crate::usart::RxDma<T>,
    {
        type ReadFuture<'a> = impl Future<Output = Result<(), Self::Error>> + 'a where Self: 'a;

        fn read<'a>(&'a mut self, buf: &'a mut [u8]) -> Self::ReadFuture<'a> {
            self.read(buf)
        }
    }

    impl<'d, T: BasicInstance, TxDma, RxDma> embedded_hal_async::serial::Write for Uart<'d, T, TxDma, RxDma>
    where
        TxDma: crate::usart::TxDma<T>,
    {
        type WriteFuture<'a> = impl Future<Output = Result<(), Self::Error>> + 'a where Self: 'a;

        fn write<'a>(&'a mut self, buf: &'a [u8]) -> Self::WriteFuture<'a> {
            self.write(buf)
        }

        type FlushFuture<'a> = impl Future<Output = Result<(), Self::Error>> + 'a where Self: 'a;

        fn flush<'a>(&'a mut self) -> Self::FlushFuture<'a> {
            async move { Ok(()) }
        }
    }

    impl<'d, T: BasicInstance, TxDma, RxDma> embedded_hal_async::serial::Read for Uart<'d, T, TxDma, RxDma>
    where
        RxDma: crate::usart::RxDma<T>,
    {
        type ReadFuture<'a> = impl Future<Output = Result<(), Self::Error>> + 'a where Self: 'a;

        fn read<'a>(&'a mut self, buf: &'a mut [u8]) -> Self::ReadFuture<'a> {
            self.read(buf)
        }
    }
}

#[cfg(feature = "nightly")]
pub use buffered::*;
#[cfg(feature = "nightly")]
mod buffered;

#[cfg(usart_v1)]
fn tdr(r: crate::pac::usart::Usart) -> *mut u8 {
    r.dr().ptr() as _
}

#[cfg(usart_v1)]
fn rdr(r: crate::pac::usart::Usart) -> *mut u8 {
    r.dr().ptr() as _
}

#[cfg(usart_v1)]
fn sr(r: crate::pac::usart::Usart) -> crate::pac::common::Reg<regs::Sr, crate::pac::common::RW> {
    r.sr()
}

#[cfg(usart_v1)]
#[allow(unused)]
unsafe fn clear_interrupt_flags(_r: Regs, _sr: regs::Sr) {
    // On v1 the flags are cleared implicitly by reads and writes to DR.
}

#[cfg(usart_v2)]
fn tdr(r: Regs) -> *mut u8 {
    r.tdr().ptr() as _
}

#[cfg(usart_v2)]
fn rdr(r: Regs) -> *mut u8 {
    r.rdr().ptr() as _
}

#[cfg(usart_v2)]
fn sr(r: Regs) -> crate::pac::common::Reg<regs::Isr, crate::pac::common::R> {
    r.isr()
}

#[cfg(usart_v2)]
#[allow(unused)]
unsafe fn clear_interrupt_flags(r: Regs, sr: regs::Isr) {
    r.icr().write(|w| *w = regs::Icr(sr.0));
}

pub(crate) mod sealed {
    use core::sync::atomic::AtomicU8;

    use atomic_polyfill::AtomicU32;
    use embassy_sync::waitqueue::AtomicWaker;

    use super::*;

    pub struct State {
        pub endrx_waker: AtomicWaker,
        // pub endtx_waker: AtomicWaker,
        pub tx_rx_refcount: AtomicU8,
        // status register in previous interrupt before clear
        pub sr: AtomicU32,
    }

    impl State {
        pub const fn new() -> Self {
            Self {
                endrx_waker: AtomicWaker::new(),
                // endtx_waker: AtomicWaker::new(),
                tx_rx_refcount: AtomicU8::new(0),
                sr: AtomicU32::new(0),
            }
        }
    }

    pub trait BasicInstance: crate::rcc::RccPeripheral {
        const MULTIPLIER: u32;
        type Interrupt: crate::interrupt::Interrupt;

        fn regs() -> Regs;
        fn state() -> &'static State;
    }

    pub trait FullInstance: BasicInstance {
        fn regs_uart() -> crate::pac::usart::Usart;
    }
}

pub trait BasicInstance: Peripheral<P = Self> + sealed::BasicInstance + 'static + Send {}

pub trait FullInstance: sealed::FullInstance {}

pin_trait!(RxPin, BasicInstance);
pin_trait!(TxPin, BasicInstance);
pin_trait!(CtsPin, BasicInstance);
pin_trait!(RtsPin, BasicInstance);
pin_trait!(CkPin, BasicInstance);

dma_trait!(TxDma, BasicInstance);
dma_trait!(RxDma, BasicInstance);

macro_rules! impl_lpuart {
    ($inst:ident, $irq:ident, $mul:expr) => {
        impl sealed::BasicInstance for crate::peripherals::$inst {
            const MULTIPLIER: u32 = $mul;
            type Interrupt = crate::interrupt::$irq;

            fn regs() -> Regs {
                Regs(crate::pac::$inst.0)
            }

            fn state() -> &'static crate::usart::sealed::State {
                static STATE: crate::usart::sealed::State = crate::usart::sealed::State::new();
                &STATE
            }
        }

        impl BasicInstance for peripherals::$inst {}
    };
}

foreach_interrupt!(
    ($inst:ident, lpuart, $block:ident, $signal_name:ident, $irq:ident) => {
        impl_lpuart!($inst, $irq, 255);
    };

    ($inst:ident, usart, $block:ident, $signal_name:ident, $irq:ident) => {
        impl_lpuart!($inst, $irq, 1);

        impl sealed::FullInstance for peripherals::$inst {

            fn regs_uart() -> crate::pac::usart::Usart {
                crate::pac::$inst
            }
        }

        impl FullInstance for peripherals::$inst {
        }
    };
);
