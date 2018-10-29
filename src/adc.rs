use core::marker::PhantomData;
use core::cmp::min;

use cortex_m::interrupt::free;
use gpio::TypeAAnalogInput;
use gpio::{P0_12, P0_13, P0_23, P0_24, P0_25, P0_26, P1_30, P1_31};
use hal::adc::{Channel, OneShot};
use lpc;
use lpc::{adc, ADC};
use nb;
use syscon::{Clocks, PoweredBlock, PCONP};

/// ADC related errors
#[derive(Debug)]
pub enum Error {
    /// A conversion on a different channel is already in progress
    AlreadyInProgress,
    /// The result waiting is for a different channel
    WrongChannel { chan: u8, value: u16 },
}

/// Marker trait for ADC running modes
///
/// Do not implement!
pub trait RunMode {
    /// Auto-configure the ADC run mode
    #[doc(hidden)]
    fn cfg(cr: &mut adc::cr::W) -> &mut adc::cr::W;
}

/// Single conversion mode
pub struct Single(());
impl RunMode for Single {
    fn cfg(w: &mut adc::cr::W) -> &mut adc::cr::W {
        w.burst().variant(adc::cr::BURSTW::SW)
    }
}

/// Burst conversion mode
pub struct Burst(());
impl RunMode for Burst {
    fn cfg(w: &mut adc::cr::W) -> &mut adc::cr::W {
        w.burst().variant(adc::cr::BURSTW::BURST)
    }
}

macro_rules! adc_pin {
    ($PXi:ident, $i:expr) => {
        impl<M> Channel<Adc<M>> for $PXi<TypeAAnalogInput>
        where
            M: RunMode,
        {
            type ID = u8;
            fn channel() -> u8 {
                $i
            }
        }
    };
}

adc_pin!(P0_23, 0);
adc_pin!(P0_24, 1);
adc_pin!(P0_25, 2);
adc_pin!(P0_26, 3);
adc_pin!(P1_30, 4);
adc_pin!(P1_31, 5);
adc_pin!(P0_12, 6);
adc_pin!(P0_13, 7);

/// Represents the ADC peripheral
pub struct Adc<MODE>
where
    MODE: RunMode,
{
    /// The raw ADC peripheral
    adc: ADC,
    #[doc(hidden)]
    /// Indicates single conversion vs burst conversion mode
    _mode: PhantomData<MODE>,
}

impl<MODE> PoweredBlock for Adc<MODE>
where
    MODE: RunMode,
{
    fn power_up<'w>(w: &'w mut lpc::syscon::pconp::W) -> &'w mut lpc::syscon::pconp::W {
        w.pcadc().set_bit()
    }

    fn power_down<'w>(w: &'w mut lpc::syscon::pconp::W) -> &'w mut lpc::syscon::pconp::W {
        w.pcadc().clear_bit()
    }
}

impl<MODE> Adc<MODE>
where
    MODE: RunMode,
{
    /// Configure the
    pub fn adc(adc: ADC, pconp: &mut PCONP, clocks: &Clocks) -> Adc<MODE>
    where
        MODE: RunMode,
    {
        // The APB clock (PCLK) is divided by (this value plus one) to produce the clock for the
        // A/D converter, which should be less than or equal to 12.4 MHz.
        let prscl: u32 = min(1, clocks.pclk / 12_400_000);
        if prscl > 255 {
            panic!("cannot scale ADC clock down");
        }

        /* The APB clock (PCLK_ADC0) is divided by (CLKDIV+1) to produce the clock for
         * A/D converter, which should be less than or equal to 12.4MHz.
         * A fully conversion requires 31 of these clocks.
         * ADC clock = PCLK_ADC0 / (CLKDIV + 1);
         * ADC rate = ADC clock / 31;
         */
        //let adc_clk_rate = 500_000; // some sane default rate
        //let prscl = (clocks.pclk * 2 + (adc_clk_rage * 31)) / (2 * (adc_clk_rage * 31));


        // The ADC is configured using the following registers:

        // 1. Power: In the PCONP register (Table 16), set the PCADC bit.
        //
        //    Remark: On reset, the ADC is disabled. To enable the ADC, first set the PCADC bit,
        //    and then enable the ADC in the AD0CR register (bit PDN Table 678). To disable the
        //    ADC, first clear the PDN bit, and then clear the PCADC bit.
        pconp.power_up::<Self>();
        adc.cr.modify(|_, w| unsafe {
            MODE::cfg(w) // Set the mode
                .pdn()
                .set_bit()
                // 2. Peripheral clock: The ADC operates from the common PCLK that clocks both the bus
                //    interface and functional portion of most APB peripherals. See Section 3.3.22. To
                //    scale the clock for the ADC, see bits CLKDIV in Table 678.
                .clkdiv()
                .bits((prscl & 0xFF) as u8)
        });

        Adc {
            adc,
            _mode: PhantomData,
        }
    }
}

impl<Pin> OneShot<Adc<Single>, u16, Pin> for Adc<Single>
where
    Pin: Channel<Adc<Single>, ID = u8>,
{
    type Error = Error;

    fn read(&mut self, _pin: &mut Pin) -> nb::Result<u16, Self::Error> {
        let chan = 1 << Pin::channel();

        free(|_cs| {
            if self.adc.gdr.read().done().bit_is_set() {
                // a conversion has completed! is it ours?
                if self.adc.gdr.read().chn().bits() == chan - 1 {
                    // Yes! (chn [000] = chan 1, ...)
                    Ok(self.adc.gdr.read().result().bits())
                } else {
                    // no :( but we have to do something with this conversion so we punt
                    Err(nb::Error::Other(Error::WrongChannel {
                        chan: self.adc.gdr.read().chn().bits(),
                        value: self.adc.gdr.read().result().bits(),
                    }))
                }
            } else {
                // there is no complete conversion - is one even ongoing?
                if self.adc.cr.read().start().is_no_start_this_value() {
                    // No conversion is in progress
                    self.adc.cr.modify(|_, w| unsafe { w.sel().bits(chan) });
                    self.adc.cr.modify(|_, w| w.start().start_conversion_now());
                    Err(nb::Error::WouldBlock)
                } else {
                    // a conversion is ongoing
                    if self.adc.cr.read().sel().bits() == chan {
                        // we're still waiting on our own conversion
                        Err(nb::Error::WouldBlock)
                    } else {
                        // we're waiting on some other channel's conversion to complete
                        Err(nb::Error::Other(Error::AlreadyInProgress))
                    }
                }
            }
        })
    }
}
