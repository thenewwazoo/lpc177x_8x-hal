//! Timers

use void::Void;
use nb;

use lpc;
use lpc::timer0::ctcr::CTMODEW;
use syscon::{Clocks, PoweredBlock, PCONP};
use hal::timer::{CountDown, Cancel, Periodic};
use time::Hertz;

/// Timer
pub struct Timer<'c, TIM> {
    /// Underlying timer peripheral
    timer: TIM,
    /// Handle to the configured clocks
    clocks: &'c Clocks,
}

macro_rules! impl_timer {
    ($(
            $TIMERX:ident: (
                $timerX:ident,
                $pctimX:ident),
    )+) => {
        $(
            use lpc::$TIMERX;

            impl<'c> PoweredBlock for Timer<'c, $TIMERX> {
                fn power_up<'w>(w: &'w mut lpc::syscon::pconp::W) -> &'w mut lpc::syscon::pconp::W {
                    w.$pctimX().set_bit()
                }

                fn power_down<'w>(w: &'w mut lpc::syscon::pconp::W) -> &'w mut lpc::syscon::pconp::W {
                    w.$pctimX().clear_bit()
                }
            }

            impl<'c> Timer<'c, $TIMERX> {
                pub fn $timerX(timer: $TIMERX, clocks: &'c Clocks, pconp: &mut PCONP) -> Self {
                    pconp.power_up::<Self>();
                    // disable the timer (for now) and reset its counters
                    timer.tcr.modify(|_,w| w.cen().clear_bit().crst().set_bit().crst().clear_bit());
                    // enable interrupt on MR0
                    // enable reset on MR0 - TC will be reset if MR0 matches it.
                    timer.mcr.modify(|_,w| w.mr0i().set_bit().mr0r().set_bit());
                    // set the timer into timer mode
                    timer.ctcr.modify(|_,w| w.ctmode().variant(CTMODEW::ON_PCLK_EDGE));

                    Timer{ timer, clocks }
                }
            }

            impl<'c> Periodic for Timer<'c, $TIMERX> {}

            impl<'c> CountDown for Timer<'c, $TIMERX> {
                type Time = Hertz;

                fn start<T>(&mut self, count: T)
                where
                    T: Into<Self::Time>
                {
                    // Disable the timer and reset it using CRST:
                    // From 24.6.2:
                    //  CRST: When 1, the Timer Counter and the Prescale Counter are synchronously
                    //  reset on the next positive edge of PCLK. The counters remain reset until
                    //  TCR[1] is returned to 0.
                    self.timer.tcr.modify(|_,w| w.cen().clear_bit().crst().set_bit().crst().clear_bit());

                    // Number of ticks per time period
                    let ticks = self.clocks.pclk / count.into().0;

                    // prescale the timer clock to account for multiples of the 32-bit counter size
                    let psc: u32 = (((ticks - 1) as u64) / (1 << 32)) as u32;
                    self.timer.pr.write(|w| unsafe { w.bits(psc) });

                    let mr: u32 = ((ticks as u64) / ((psc + 1) as u64)) as u32;
                    self.timer.mr[0].write(|w| unsafe { w.bits(mr) });

                    self.timer.tcr.modify(|_, w| w.cen().set_bit());
                }

                fn wait(&mut self) -> nb::Result<(), Void> {
                    match self.timer.ir.read().mr0int().bit_is_clear() {
                        true => Err(nb::Error::WouldBlock),
                        false => {
                            // clear the mr0 intr flag
                            self.timer.ir.modify(|_,w| w.mr0int().set_bit());
                            Ok(())
                        }
                    }
                }

            }

            impl<'c> Cancel for Timer<'c, $TIMERX> {
                type Error = Void;

                fn cancel(&mut self) -> Result<(), Self::Error> {
                    self.timer.tcr.modify(|_,w| w.cen().clear_bit().crst().set_bit().crst().clear_bit());
                    Ok(())
                }

            }

        )+
    }
}

impl_timer!(
    TIMER0: (timer0, pctim0),
    TIMER1: (timer1, pctim1),
    TIMER2: (timer2, pctim2),
    TIMER3: (timer3, pctim3),
);
