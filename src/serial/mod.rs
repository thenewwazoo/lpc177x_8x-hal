// let serial = hal::serial::uart0(&mut syscon, &pclk, 9600.baud(), (tx, rx));

use core::marker::PhantomData;

use hal::serial;
use lpc;
use nb;

// #[cfg(any(feature = "LQFP208", feature = "TFBGA208", feature = "TFBGA180"))]
#[cfg(feature = "LQFP208")]
pub mod pkg_208pin;

pub mod clocking {
    pub struct PClkSource(pub u32);
}

#[derive(Debug, Copy, Clone)]
pub enum Event {
    /// Receive Data Available, also Character Receive Time-out
    Rbr,
    /// Transmitter Holding Register Empty
    Thre,
    /// RX line status interrupts. The status of this interrupt can be read from LSR
    Rxie,
    /// end of auto-baud
    Abeo,
    /// auto-baud time-out
    Abto,
}

#[derive(Debug, Copy, Clone)]
pub enum Error {
    /// Overrun Error
    Oe,
    /// Parity Error
    Pe,
    /// Framing Error
    Fe,
    /// Error in RX FIFO
    Rxfe,
}

/// Serial abstraction
pub struct Serial<UART, PINS> {
    uart: UART,
    #[allow(dead_code)]
    pins: PINS,
}

/// Serial receiver
pub struct Rx<UART> {
    _uart: PhantomData<UART>,
}

/// Serial transmitter
pub struct Tx<UART> {
    _uart: PhantomData<UART>,
}

// FIXME these should be "closed" traits
/// TX pin - DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait TxPin<UART> {}

/// RX pin - DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait RxPin<UART> {}

pub struct Bps(pub u32);

macro_rules! uart {
    ($(
        $UARTX:ident: (
            $uartX:ident,
            $pcuartX:ident,
            $wordlen:ident),
    )+) => {
        $(
            use lpc::$UARTX;

            impl<TX, RX> Serial<$UARTX, (TX, RX)> {
                /// Configures the $UARTX peripheral to provide 8N1 asynchronous serial communication
                pub fn $uartX(
                    syscon: &mut lpc::SYSCON,
                    uart: $UARTX,
                    pins: (TX, RX),
                    baud_rate: Bps,
                    clock: clocking::PClkSource,
                ) -> Self
                where
                    TX: TxPin<$UARTX>,
                    RX: RxPin<$UARTX>,
                {
                    // From 18.2 Basic Configuration...

                    // Power: In the PCONP register (Table 16), set bits PCUART0/2/3
                    syscon.pconp.modify(|_,w| w.$pcuartX().set_bit());

                    // Baud rate: In register U0/2/3LCR (Table 399), set bit DLAB =1. This enables
                    // access to registers DLL (Table 393) and DLM (Table 394) for setting the baud
                    // rate. Also, if needed, set the fractional baud rate in the fractional
                    // divider register (Table 403).
                    uart.lcr.modify(|_,w| w
                                    .wls()
                                    .variant(lpc::$uartX::lcr::WLSW::$wordlen)
                                    .pe()
                                    .variant(lpc::$uartX::lcr::PEW::DISABLE)
                                    .sbs()
                                    .variant(lpc::$uartX::lcr::SBSW::_1_STOP_BIT)
                                    .dlab()
                                    .enable());

                    // The UARTn Divisor Latch is part of the UARTn Baud Rate Generator and holds
                    // the value used, along with the Fractional Divider, to divide the APB clock
                    // (PCLK) in order to produce the baud rate clock, which must be 16x the
                    // desired baud rate.

                    let (dl, m, d) = calc_dl(clock.0, baud_rate.0).expect("failed to calculate baud dividers");
                    unsafe { uart.dll.dll.modify(|_,w| w.dllsb().bits((dl & 0xFF) as u8)); }
                    unsafe { uart.dlm.dlm.modify(|_,w| w.dlmsb().bits((dl >> 8) as u8 & 0xFF)); }
                    uart.fdr.modify(|_,w| unsafe {
                        w
                            .divaddval()
                            .bits((d & 0xFF) as u8)
                            .mulval()
                            .bits((m & 0xFF) as u8)
                    });

                    uart.lcr.modify(|_,w| w.dlab().disable());

                    unsafe {
                        uart.fcr.fcr.write(|w| w
                                           .fifoen()
                                           .variant(lpc::$uartX::fcr::FIFOENW::ACTIVE_HIGH_EN)
                                           .rxfifores()
                                           .variant(lpc::$uartX::fcr::RXFIFORESW::CLEAR_ALL_BYTES)
                                           .txfifores()
                                           .variant(lpc::$uartX::fcr::TXFIFORESW::CLEAR_ALL_BYTES)
                                          );
                    }

                    Serial { uart, pins }
                }

                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::Rbr => unsafe { self.uart.dlm.ier.modify(|_,w| w.rbrie().set_bit()) },
                        Event::Thre => unsafe { self.uart.dlm.ier.modify(|_,w| w.threie().set_bit()) },
                        Event::Rxie => unsafe { self.uart.dlm.ier.modify(|_,w| w.rxie().set_bit()) },
                        Event::Abeo => unsafe { self.uart.dlm.ier.modify(|_,w| w.abeointen().set_bit()) },
                        Event::Abto => unsafe { self.uart.dlm.ier.modify(|_,w| w.abtointen().set_bit()) },
                    }
                }

                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::Rbr => unsafe { self.uart.dlm.ier.modify(|_,w| w.rbrie().clear_bit()) },
                        Event::Thre => unsafe { self.uart.dlm.ier.modify(|_,w| w.threie().clear_bit()) },
                        Event::Rxie => unsafe { self.uart.dlm.ier.modify(|_,w| w.rxie().clear_bit()) },
                        Event::Abeo => unsafe { self.uart.dlm.ier.modify(|_,w| w.abeointen().clear_bit()) },
                        Event::Abto => unsafe { self.uart.dlm.ier.modify(|_,w| w.abtointen().clear_bit()) },
                    }
                }

                /// Splits the `Serial` abstraction into a transmitter and a receiver half
                pub fn split(self) -> (Tx<$UARTX>, Rx<$UARTX>) {
                    (
                        Tx {
                            _uart: PhantomData,
                        },
                        Rx {
                            _uart: PhantomData,
                        },
                    )
                }

            }

            impl serial::Read<u8> for Rx<$UARTX> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    let lsr = unsafe { (*$UARTX::ptr()).lsr.read() };

                    Err(if lsr.oe().is_active() {
                            nb::Error::Other(Error::Oe)
                        } else if lsr.pe().is_active() {
                            nb::Error::Other(Error::Pe)
                        } else if lsr.fe().is_active() {
                            nb::Error::Other(Error::Fe)
                        } else if lsr.rxfe().is_errors() {
                            nb::Error::Other(Error::Rxfe)
                        } else if lsr.rdr().is_notempty() {
                            return Ok(unsafe {
                                (*$UARTX::ptr()).dll.rbr.read().rbr().bits()
                            });
                        } else {
                            nb::Error::WouldBlock
                        }
                       )
                }
            }

            impl serial::Write<u8> for Tx<$UARTX> {
                type Error = !;

                fn flush(&mut self) -> nb::Result<(), !> {
                    let lsr = unsafe { (*$UARTX::ptr()).lsr.read() };

                    if lsr.temt().is_empty() {
                        Ok(())
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                }

                fn write(&mut self, byte: u8) -> nb::Result<(), !> {
                    let lsr = unsafe { (*$UARTX::ptr()).lsr.read() };

                    if lsr.thre().is_empty() {
                        unsafe {
                            (*$UARTX::ptr()).dll.thr.write(|w| w.thr().bits(byte));
                        }
                        Ok(())
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                }
            }

         )+
    }
}

uart!(
    UART0: (uart0, pcuart0, _8_BIT),
    UART1: (uart1, pcuart1, _8_BIT_CHARACTER_LEN),
    UART2: (uart0, pcuart2, _8_BIT),
    UART3: (uart0, pcuart3, _8_BIT),
    UART4: (uart4, pcuart4, _8_BIT_CHARACTER_LEN),
);

fn calc_dl(clk: u32, baud: u32) -> Result<(u32, u32, u32), ()> {
    let mut bestd: u32 = 0;
    let mut bestm: u32 = 0;
    let mut best_divisor: u32 = 0;
    let mut best_error = 0xFFFF_FFFF_u32;

    for m in 1..=15u32 {
        for d in 1..m {
            let divisor: u64 = ((clk as u64) << 28) * m as u64 / (baud * (m + d)) as u64;
            let current_error = (divisor & 0xFFFF_FFFF_u64) as u32;
            let mut tmp: u32 = (divisor >> 32) as u32;

            if current_error > (1 << 31) {
                tmp += 1;
            }

            if tmp < 1 || tmp > 65536 {
                continue;
            }

            if current_error < best_error {
                best_error = current_error;
                best_divisor = tmp.into();
                bestd = d;
                bestm = m;

                if best_error == 0 {
                    break;
                }
            }
        }

        if best_error == 0 {
            break;
        }
    }

    if best_divisor == 0 {
        return Err(());
    }

    let recalcbaud: u32 = (clk >> 4) * bestm / (best_divisor * (bestm + bestd));

    if baud > recalcbaud {
        best_error = baud - recalcbaud;
    } else {
        best_error = recalcbaud - baud;
    }

    best_error = best_error * 100 / baud;

    if best_error < 3 {
        Ok((best_divisor, bestm, bestd))
    } else {
        Err(())
    }
}
