//! General Purpose Input / Output

#![allow(unknown_lints)]
#![allow(clippy)]

use core::marker::PhantomData;

use common::Constrain;
use hal::digital::{InputPin, OutputPin, StatefulOutputPin};
use lpc;
use lpc::{GPIO, IOCON};

pub mod regset;

impl Constrain<Gpio> for GPIO {
    fn constrain(self) -> Gpio {
        Gpio(
            regset::Zero(()),
            regset::One(()),
            regset::Two(()),
            regset::Three(()),
            regset::Four(()),
            regset::Five(()),
        )
    }
}

/// GPIO register groups
///
/// Gpio::Zero is a collection of GPIO registers relating to GPIO port 0, and so on. To access the
/// members, use `gpio.0` for port 0, etc.
pub struct Gpio(
    pub regset::Zero,
    pub regset::One,
    pub regset::Two,
    pub regset::Three,
    pub regset::Four,
    pub regset::Five,
);

impl Gpio {
    pub fn enable_gpio(syscon: &mut lpc::SYSCON) {
        syscon.pconp.modify(|_, w| w.pcgpio().set_bit());
    }

    pub fn disable_gpio(syscon: &mut lpc::SYSCON) {
        syscon.pconp.modify(|_, w| w.pcgpio().clear_bit());
    }
}

pub trait VariantSetting<VARIANT> {
    fn get_variant() -> VARIANT;
}

pub struct TypeDInput<PUPDMODE, HYS, INV> {
    _mode: PhantomData<PUPDMODE>,
    _hyst: PhantomData<HYS>,
    _inv: PhantomData<INV>,
}

pub struct TypeWInput<PUPDMODE, HYS, INV, FILTER> {
    _mode: PhantomData<PUPDMODE>,
    _hyst: PhantomData<HYS>,
    _inv: PhantomData<INV>,
    _filt: PhantomData<FILTER>,
}

pub struct TypeAAnalogInput;
pub struct TypeADigitalInput<PUPDMODE, INV, FILTER> {
    _mode: PhantomData<PUPDMODE>,
    _inv: PhantomData<INV>,
    _filt: PhantomData<FILTER>,
}

pub struct TypeIInput<INV, HS, HIDRIVE> {
    _inv: PhantomData<INV>,
    _hs: PhantomData<HS>,
    _hidrive: PhantomData<HIDRIVE>,
}

/// This type is a workaround for the impl_pins macro, which requires a type specifier for its
/// syntax rules.
type Nonce = !;

pub struct TypeUInput<NONCE> {
    _nonce: PhantomData<NONCE>,
}

pub struct TypeDWOutput<PUPDMODE, SLEW, ODMODE> {
    _pupdmode: PhantomData<PUPDMODE>,
    _slew: PhantomData<SLEW>,
    _odmode: PhantomData<ODMODE>,
}

pub struct TypeAOutput<PUPDMODE, ODMODE> {
    _pupdmode: PhantomData<PUPDMODE>,
    _odmode: PhantomData<ODMODE>,
}

pub struct TypeIOutput<HS, HIDRIVE> {
    _hs: PhantomData<HS>,
    _hidrive: PhantomData<HIDRIVE>,
}

pub struct TypeUOutput<NONCE> {
    _nonce: PhantomData<NONCE>,
}

pub struct Inactive;
pub struct PullDown;
pub struct PullUp;
pub struct Repeater;

macro_rules! pupdmode_pin {
    ($px_y:ident) => {
        impl VariantSetting<lpc::iocon::$px_y::MODEW> for Inactive {
            fn get_variant() -> lpc::iocon::$px_y::MODEW {
                lpc::iocon::$px_y::MODEW::INACTIVE
            }
        }

        impl VariantSetting<lpc::iocon::$px_y::MODEW> for PullDown {
            fn get_variant() -> lpc::iocon::$px_y::MODEW {
                lpc::iocon::$px_y::MODEW::PULLDOWN_EN
            }
        }

        impl VariantSetting<lpc::iocon::$px_y::MODEW> for PullUp {
            fn get_variant() -> lpc::iocon::$px_y::MODEW {
                lpc::iocon::$px_y::MODEW::PULLUP_EN
            }
        }

        impl VariantSetting<lpc::iocon::$px_y::MODEW> for Repeater {
            fn get_variant() -> lpc::iocon::$px_y::MODEW {
                lpc::iocon::$px_y::MODEW::REPEATER_MODE
            }
        }
    };
}

pub struct HysDis;
pub struct HysEn;

macro_rules! hysteresis_pin {
    ($px_y:ident) => {
        impl VariantSetting<lpc::iocon::$px_y::HYSW> for HysEn {
            fn get_variant() -> lpc::iocon::$px_y::HYSW {
                lpc::iocon::$px_y::HYSW::ENABLE
            }
        }

        impl VariantSetting<lpc::iocon::$px_y::HYSW> for HysDis {
            fn get_variant() -> lpc::iocon::$px_y::HYSW {
                lpc::iocon::$px_y::HYSW::DISABLE
            }
        }
    };
}

pub struct NonInverted;
pub struct Inverted;

macro_rules! invertible_pin {
    ($px_y:ident) => {
        impl VariantSetting<lpc::iocon::$px_y::INVW> for NonInverted {
            fn get_variant() -> lpc::iocon::$px_y::INVW {
                lpc::iocon::$px_y::INVW::INPUT_NOT_INVERTED
            }
        }

        impl VariantSetting<lpc::iocon::$px_y::INVW> for Inverted {
            fn get_variant() -> lpc::iocon::$px_y::INVW {
                lpc::iocon::$px_y::INVW::INPUT_INVERTED_HIGH
            }
        }
    };
}

pub struct Filtered;
pub struct NonFiltered;

macro_rules! w_filtered_pin {
    ($px_y:ident) => {
        impl VariantSetting<lpc::iocon::$px_y::FILTERW> for Filtered {
            fn get_variant() -> lpc::iocon::$px_y::FILTERW {
                lpc::iocon::$px_y::FILTERW::ENABLED
            }
        }

        impl VariantSetting<lpc::iocon::$px_y::FILTERW> for NonFiltered {
            fn get_variant() -> lpc::iocon::$px_y::FILTERW {
                lpc::iocon::$px_y::FILTERW::DISABLED
            }
        }
    };
}

macro_rules! filtered_pin {
    ($px_y:ident) => {
        impl VariantSetting<lpc::iocon::$px_y::FILTERW> for Filtered {
            fn get_variant() -> lpc::iocon::$px_y::FILTERW {
                lpc::iocon::$px_y::FILTERW::FILTER_ENABLED
            }
        }

        impl VariantSetting<lpc::iocon::$px_y::FILTERW> for NonFiltered {
            fn get_variant() -> lpc::iocon::$px_y::FILTERW {
                lpc::iocon::$px_y::FILTERW::FILTER_DISABLED
            }
        }
    };
}

pub struct StdSlew;
pub struct FastSlew;

macro_rules! slew_pin {
    ($px_y:ident) => {
        impl VariantSetting<lpc::iocon::$px_y::SLEWW> for StdSlew {
            fn get_variant() -> lpc::iocon::$px_y::SLEWW {
                lpc::iocon::$px_y::SLEWW::STANDARD
            }
        }

        impl VariantSetting<lpc::iocon::$px_y::SLEWW> for FastSlew {
            fn get_variant() -> lpc::iocon::$px_y::SLEWW {
                lpc::iocon::$px_y::SLEWW::FAST
            }
        }
    };
}

pub struct PushPull;
pub struct OpenDrain;

macro_rules! open_drain_pin {
    ($px_y:ident) => {
        impl VariantSetting<lpc::iocon::$px_y::ODW> for PushPull {
            fn get_variant() -> lpc::iocon::$px_y::ODW {
                lpc::iocon::$px_y::ODW::DISABLE
            }
        }

        impl VariantSetting<lpc::iocon::$px_y::ODW> for OpenDrain {
            fn get_variant() -> lpc::iocon::$px_y::ODW {
                lpc::iocon::$px_y::ODW::ENABLED
            }
        }
    };
}

pub struct I2CHSEn;
pub struct I2CHSDis;

macro_rules! hs_pin {
    ($px_y:ident) => {
        impl VariantSetting<lpc::iocon::$px_y::HSW> for I2CHSEn {
            fn get_variant() -> lpc::iocon::$px_y::HSW {
                lpc::iocon::$px_y::HSW::ENABLED
            }
        }

        impl VariantSetting<lpc::iocon::$px_y::HSW> for I2CHSDis {
            fn get_variant() -> lpc::iocon::$px_y::HSW {
                lpc::iocon::$px_y::HSW::DISABLED
            }
        }
    };
}

pub struct LowDrive;
pub struct HighDrive;

macro_rules! hidrive_pin {
    ($px_y:ident) => {
        impl VariantSetting<lpc::iocon::$px_y::HIDRIVEW> for LowDrive {
            fn get_variant() -> lpc::iocon::$px_y::HIDRIVEW {
                lpc::iocon::$px_y::HIDRIVEW::LOWDRIVE
            }
        }

        impl VariantSetting<lpc::iocon::$px_y::HIDRIVEW> for HighDrive {
            fn get_variant() -> lpc::iocon::$px_y::HIDRIVEW {
                lpc::iocon::$px_y::HIDRIVEW::HIGHDRIVE
            }
        }
    };
}

macro_rules! input_impl {
    ($PX_Y:ident, $input_type:ident<$($type:ident),+>, $pin:ident, $i:expr) => {
        impl<$($type,)+> InputPin for $PX_Y<$input_type<$($type,)+>> {
            fn is_high(&self) -> bool {
                let gpio = unsafe { &(*GPIO::ptr()) };
                gpio.$pin.read().bits() & (1 << $i) == 1
            }

            fn is_low(&self) -> bool {
                !self.is_high()
            }
        }
    }
}

macro_rules! stateful_output_impl {
    ($PX_Y:ident, $output_type:ident<$($type:ident),+>, $pin:ident, $i:expr) => {

        impl<$($type,)+> StatefulOutputPin for $PX_Y<$output_type<$($type,)+>> {
            fn is_set_high(&self) -> bool {
                let gpio = unsafe { &(*GPIO::ptr()) };
                gpio.$pin.read().bits() & (1 << $i) == 1
            }

            fn is_set_low(&self) -> bool {
                !self.is_set_high()
            }
        }
    }
}

macro_rules! output_impl {
    ($PX_Y:ident, $output_type:ident<$($type:ident),+>, $set:ident, $clr:ident, $i:expr) => {

        impl<$($type,)+> OutputPin for $PX_Y<$output_type<$($type,)+>> {
            fn set_high(&mut self) {
                let gpio = unsafe { &(*GPIO::ptr()) };
                gpio.$set.modify(|_,w| unsafe { w.bits(1 << $i)});
            }

            fn set_low(&mut self) {
                let gpio = unsafe { &(*GPIO::ptr()) };
                gpio.$clr.write(|w| unsafe { w.bits(1 << $i)});
            }
        }
    }
}

// type U: func
macro_rules! type_u_pin {
    ($PX_Y:ident, $px_y:ident, $dir:ident, $pin:ident, $set:ident, $clr:ident, $i:expr) => {
        pub struct $PX_Y<MODE>(PhantomData<MODE>);

        output_impl!($PX_Y, TypeUOutput<Nonce>, $set, $clr, $i);
        stateful_output_impl!($PX_Y, TypeUOutput<Nonce>, $pin, $i);
        input_impl!($PX_Y, TypeUInput<Nonce>, $pin, $i);

        impl<MODE> $PX_Y<MODE> {
            const OFFSET: u32 = $i;

            pub fn into_input(self) -> $PX_Y<TypeUInput<!>> {
                let gpio = unsafe { &(*GPIO::ptr()) };
                gpio.$dir
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << Self::OFFSET)) });
                let iocon = unsafe { &(*IOCON::ptr()) };
                iocon.$px_y.modify(|_, w| w.func().$px_y());

                $PX_Y(PhantomData)
            }

            pub fn into_output(self) -> $PX_Y<TypeUOutput<!>> {
                let gpio = unsafe { &(*GPIO::ptr()) };
                gpio.$dir
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << Self::OFFSET)) });
                let iocon = unsafe { &(*IOCON::ptr()) };
                iocon.$px_y.modify(|_, w| w.func().$px_y());

                $PX_Y(PhantomData)
            }
        }
    };
}

// type I: func          inv                             hs hidrive
macro_rules! type_i_pin {
    ($PX_Y:ident, $px_y:ident, $dir:ident, $pin:ident, $set:ident, $clr:ident, $i:expr) => {
        pub struct $PX_Y<MODE>(PhantomData<MODE>);

        invertible_pin!($px_y);
        hs_pin!($px_y);
        hidrive_pin!($px_y);

        output_impl!($PX_Y, TypeIOutput<HS, HIDRIVE>, $set, $clr, $i);
        stateful_output_impl!($PX_Y, TypeIOutput<HS, HIDRIVE>, $pin, $i);
        input_impl!($PX_Y, TypeIInput<INV, HS, HIDRIVE>, $pin, $i);

        impl<MODE> $PX_Y<MODE> {
            const OFFSET: u32 = $i;

            pub fn into_input<
                Inv: VariantSetting<lpc::iocon::$px_y::INVW>,
                Hs: VariantSetting<lpc::iocon::$px_y::HSW>,
                Hidrive: VariantSetting<lpc::iocon::$px_y::HIDRIVEW>,
            >(
                self,
            ) -> $PX_Y<TypeIInput<Inv, Hs, Hidrive>> {
                let gpio = unsafe { &(*GPIO::ptr()) };
                gpio.$dir
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << Self::OFFSET)) });
                let iocon = unsafe { &(*IOCON::ptr()) };
                iocon.$px_y.modify(|_, w| {
                    w.func()
                        .$px_y()
                        .inv()
                        .variant(Inv::get_variant())
                        .hs()
                        .variant(Hs::get_variant())
                        .hidrive()
                        .variant(Hidrive::get_variant())
                });

                $PX_Y(PhantomData)
            }

            pub fn into_output<
                Hs: VariantSetting<lpc::iocon::$px_y::HSW>,
                Hidrive: VariantSetting<lpc::iocon::$px_y::HIDRIVEW>,
            >(
                self,
            ) -> $PX_Y<TypeIOutput<Hs, Hidrive>> {
                let gpio = unsafe { &(*GPIO::ptr()) };
                gpio.$dir
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << Self::OFFSET)) });
                let iocon = unsafe { &(*IOCON::ptr()) };
                iocon.$px_y.modify(|_, w| {
                    w.func()
                        .$px_y()
                        .hs()
                        .variant(Hs::get_variant())
                        .hidrive()
                        .variant(Hidrive::get_variant())
                });

                $PX_Y(PhantomData)
            }
        }
    };
}

// type W: func mode hys inv        filter slew od
macro_rules! type_w_pin {
    ($PX_Y:ident, $px_y:ident, $dir:ident, $pin:ident, $set:ident, $clr:ident, $i:expr) => {
        pub struct $PX_Y<MODE>(PhantomData<MODE>);

        pupdmode_pin!($px_y);
        hysteresis_pin!($px_y);
        invertible_pin!($px_y);
        w_filtered_pin!($px_y);
        slew_pin!($px_y);
        open_drain_pin!($px_y);

        output_impl!($PX_Y, TypeDWOutput<OMODE, SLEW, OD>, $set, $clr, $i);
        stateful_output_impl!($PX_Y, TypeDWOutput<OMODE, SLEW, OD>, $pin, $i);
        input_impl!($PX_Y, TypeWInput<IMODE, HYS, INV, FILT>, $pin, $i);

        impl<MODE> $PX_Y<MODE> {
            const OFFSET: u32 = $i;

            pub fn into_input<
                IMode: VariantSetting<lpc::iocon::$px_y::MODEW>,
                Hys: VariantSetting<lpc::iocon::$px_y::HYSW>,
                Inv: VariantSetting<lpc::iocon::$px_y::INVW>,
                Filt: VariantSetting<lpc::iocon::$px_y::FILTERW>,
            >(
                self,
            ) -> $PX_Y<TypeWInput<IMode, Hys, Inv, Filt>> {
                let gpio = unsafe { &(*GPIO::ptr()) };
                gpio.$dir
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << Self::OFFSET)) });
                let iocon = unsafe { &(*IOCON::ptr()) };
                iocon.$px_y.modify(|_, w| {
                    w.func()
                        .$px_y()
                        .mode()
                        .variant(IMode::get_variant())
                        .hys()
                        .variant(Hys::get_variant())
                        .inv()
                        .variant(Inv::get_variant())
                        .filter()
                        .variant(Filt::get_variant())
                });

                $PX_Y(PhantomData)
            }

            pub fn into_output<
                OMode: VariantSetting<lpc::iocon::$px_y::MODEW>,
                Slew: VariantSetting<lpc::iocon::$px_y::SLEWW>,
                ODMode: VariantSetting<lpc::iocon::$px_y::ODW>,
            >(
                self,
            ) -> $PX_Y<TypeDWOutput<OMode, Slew, ODMode>> {
                let gpio = unsafe { &(*GPIO::ptr()) };
                gpio.$dir
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << Self::OFFSET)) });
                let iocon = unsafe { &(*IOCON::ptr()) };
                iocon.$px_y.modify(|_, w| {
                    w.func()
                        .$px_y()
                        .mode()
                        .variant(OMode::get_variant())
                        .slew()
                        .variant(Slew::get_variant())
                        .od()
                        .variant(ODMode::get_variant())
                });

                $PX_Y(PhantomData)
            }
        }
    };
}

// type A: func mode     inv admode filter      od dacen
macro_rules! type_a_pin {
    ($PX_Y:ident, $px_y:ident, $dir:ident, $pin:ident, $set:ident, $clr:ident, $i:expr) => {
        pub struct $PX_Y<MODE>(PhantomData<MODE>);

        pupdmode_pin!($px_y);
        invertible_pin!($px_y);
        filtered_pin!($px_y);
        open_drain_pin!($px_y);

        output_impl!($PX_Y, TypeAOutput<OMODE, OD>, $set, $clr, $i);
        stateful_output_impl!($PX_Y, TypeAOutput<OMODE, OD>, $pin, $i);
        input_impl!($PX_Y, TypeADigitalInput<IMODE, INV, FILT>, $pin, $i);

        impl<MODE> $PX_Y<MODE> {
            const OFFSET: u32 = $i;

            pub fn into_analog_input(self) -> $PX_Y<TypeAAnalogInput> {
                let gpio = unsafe { &(*GPIO::ptr()) };
                gpio.$dir
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << Self::OFFSET)) });
                let iocon = unsafe { &(*IOCON::ptr()) };
                iocon.$px_y.modify(|_, w| {
                    w.func()
                        .$px_y()
                        .mode()
                        .variant(lpc::iocon::$px_y::MODEW::INACTIVE)
                        .admode()
                        .variant(lpc::iocon::$px_y::ADMODEW::ANALOG_INPUT_MODE)
                });

                $PX_Y(PhantomData)
            }

            pub fn into_digital_input<
                IMode: VariantSetting<lpc::iocon::$px_y::MODEW>,
                Inv: VariantSetting<lpc::iocon::$px_y::INVW>,
                Filter: VariantSetting<lpc::iocon::$px_y::FILTERW>,
            >(
                self,
            ) -> $PX_Y<TypeADigitalInput<IMode, Inv, Filter>> {
                let gpio = unsafe { &(*GPIO::ptr()) };
                gpio.$dir
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << Self::OFFSET)) });
                let iocon = unsafe { &(*IOCON::ptr()) };
                iocon.$px_y.modify(|_, w| {
                    w.func()
                        .$px_y()
                        .mode()
                        .variant(IMode::get_variant())
                        .inv()
                        .variant(Inv::get_variant())
                        .filter()
                        .variant(Filter::get_variant())
                });

                $PX_Y(PhantomData)
            }

            pub fn into_output<
                OMode: VariantSetting<lpc::iocon::$px_y::MODEW>,
                ODMode: VariantSetting<lpc::iocon::$px_y::ODW>,
            >(
                self,
            ) -> $PX_Y<TypeAOutput<OMode, ODMode>> {
                let gpio = unsafe { &(*GPIO::ptr()) };
                gpio.$dir
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << Self::OFFSET)) });
                let iocon = unsafe { &(*IOCON::ptr()) };
                iocon.$px_y.modify(|_, w| {
                    w.func()
                        .$px_y()
                        .mode()
                        .variant(OMode::get_variant())
                        .od()
                        .variant(ODMode::get_variant())
                });

                $PX_Y(PhantomData)
            }
        }
    };
}

macro_rules! type_d_pin {
    ($PX_Y:ident, $px_y:ident, $dir:ident, $pin:ident, $set:ident, $clr:ident, $i:expr) => {
        pub struct $PX_Y<MODE>(PhantomData<MODE>);

        pupdmode_pin!($px_y);
        hysteresis_pin!($px_y);
        invertible_pin!($px_y);
        slew_pin!($px_y);
        open_drain_pin!($px_y);

        output_impl!($PX_Y, TypeDWOutput<OMODE, SLEW, OD>, $set, $clr, $i);
        stateful_output_impl!($PX_Y, TypeDWOutput<OMODE, SLEW, OD>, $pin, $i);
        input_impl!($PX_Y, TypeDInput<IMODE, HYS, INV>, $pin, $i);

        impl<MODE> $PX_Y<MODE> {
            const OFFSET: u32 = $i;

            pub fn into_input<
                IMode: VariantSetting<lpc::iocon::$px_y::MODEW>,
                Hys: VariantSetting<lpc::iocon::$px_y::HYSW>,
                Inv: VariantSetting<lpc::iocon::$px_y::INVW>,
            >(
                self,
            ) -> $PX_Y<TypeDInput<IMode, Hys, Inv>> {
                let gpio = unsafe { &(*GPIO::ptr()) };
                gpio.$dir
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << Self::OFFSET)) });
                let iocon = unsafe { &(*IOCON::ptr()) };
                iocon.$px_y.modify(|_, w| {
                    w.func()
                        .$px_y()
                        .mode()
                        .variant(IMode::get_variant())
                        .hys()
                        .variant(Hys::get_variant())
                        .inv()
                        .variant(Inv::get_variant())
                });

                $PX_Y(PhantomData)
            }

            pub fn into_output<
                OMode: VariantSetting<lpc::iocon::$px_y::MODEW>,
                Slew: VariantSetting<lpc::iocon::$px_y::SLEWW>,
                ODMode: VariantSetting<lpc::iocon::$px_y::ODW>,
            >(
                self,
            ) -> $PX_Y<TypeDWOutput<OMode, Slew, ODMode>> {
                let gpio = unsafe { &(*GPIO::ptr()) };
                gpio.$dir
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << Self::OFFSET)) });
                let iocon = unsafe { &(*IOCON::ptr()) };
                iocon.$px_y.modify(|_, w| {
                    w.func()
                        .$px_y()
                        .mode()
                        .variant(OMode::get_variant())
                        .slew()
                        .variant(Slew::get_variant())
                        .od()
                        .variant(ODMode::get_variant())
                });

                $PX_Y(PhantomData)
            }
        }
    };
}

/// Generate each of the specified pins.
macro_rules! impl_pins {
    ($($type_macro:ident, $default_type:ident<$($type:ident),+>, $PX_Y:ident, $px_y:ident, $dir:ident, $pin:ident, $set:ident, $clr:ident, $i:expr;)+) => {
        $(
            $type_macro!($PX_Y, $px_y, $dir, $pin, $set, $clr, $i);

            impl<MODE> $PX_Y<MODE> {

                pub fn into_alt_func<AF: AltFun>(self) -> $PX_Y<AF> {
                    let iocon = unsafe { &(*IOCON::ptr()) };
                    #[allow(unused_unsafe)]
                    iocon.$px_y.modify(|_, w| unsafe { w.func().bits(AF::NUM) });

                    $PX_Y(PhantomData)
                }
            }

         )+

            pub struct PinSet {
                $(
                    pub $px_y: $PX_Y<$default_type<$($type,)+>>,
                 )+
            }

            impl Gpio {
                pub fn split(self, _iocon: lpc::IOCON) -> PinSet {
                    PinSet {
                    $(
                        $px_y: $PX_Y(PhantomData),
                     )+
                    }
                }
            }
    }
}

macro_rules! impl_af {
    ( [$($af:ident, $i:expr;)*] ) => {
        $(
            pub struct $af;
            impl AltFun for $af {
                const NUM: u8 = $i;
            }
         )*
    }
}

/// Alternate Function Trait
/// Implemented only for corresponding structs.
///
/// Note: MUST not be implemented by user.
pub trait AltFun {
    const NUM: u8;
}

impl_af!([AF0, 0; AF1, 1; AF2, 2; AF3, 3; AF4, 4; AF5, 5; AF6, 6; AF7, 7;]);

impl_pins!(
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P0_0, p0_0, dir0, pin0, set0, clr0, 0;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P0_1, p0_1, dir0, pin0, set0, clr0, 1;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P0_2, p0_2, dir0, pin0, set0, clr0, 2;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P0_3, p0_3, dir0, pin0, set0, clr0, 3;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P0_4, p0_4, dir0, pin0, set0, clr0, 4;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P0_5, p0_5, dir0, pin0, set0, clr0, 5;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P0_6, p0_6, dir0, pin0, set0, clr0, 6;
    type_w_pin, TypeWInput<Inactive,HysDis,NonInverted,NonFiltered>, P0_7, p0_7, dir0, pin0, set0, clr0, 7;
    type_w_pin, TypeWInput<Inactive,HysDis,NonInverted,NonFiltered>, P0_8, p0_8, dir0, pin0, set0, clr0, 8;
    type_w_pin, TypeWInput<Inactive,HysDis,NonInverted,NonFiltered>, P0_9, p0_9, dir0, pin0, set0, clr0, 9;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P0_10, p0_10, dir0, pin0, set0, clr0, 10;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P0_11, p0_11, dir0, pin0, set0, clr0, 11;
    type_a_pin, TypeADigitalInput<PullUp,HysDis,NonFiltered>, P0_12, p0_12, dir0, pin0, set0, clr0, 12;
    type_a_pin, TypeADigitalInput<PullUp,HysDis,NonFiltered>, P0_13, p0_13, dir0, pin0, set0, clr0, 13;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P0_14, p0_14, dir0, pin0, set0, clr0, 14;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P0_15, p0_15, dir0, pin0, set0, clr0, 15;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P0_16, p0_16, dir0, pin0, set0, clr0, 16;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P0_17, p0_17, dir0, pin0, set0, clr0, 17;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P0_18, p0_18, dir0, pin0, set0, clr0, 18;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P0_19, p0_19, dir0, pin0, set0, clr0, 19;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P0_20, p0_20, dir0, pin0, set0, clr0, 20;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P0_21, p0_21, dir0, pin0, set0, clr0, 21;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P0_22, p0_22, dir0, pin0, set0, clr0, 22;
    type_a_pin, TypeADigitalInput<PullUp,HysDis,NonFiltered>, P0_23, p0_23, dir0, pin0, set0, clr0, 23;
    type_a_pin, TypeADigitalInput<PullUp,HysDis,NonFiltered>, P0_24, p0_24, dir0, pin0, set0, clr0, 24;
    type_a_pin, TypeADigitalInput<PullUp,HysDis,NonFiltered>, P0_25, p0_25, dir0, pin0, set0, clr0, 25;
    type_a_pin, TypeADigitalInput<PullUp,HysDis,NonFiltered>, P0_26, p0_26, dir0, pin0, set0, clr0, 26; // n.b. DACEN is not implemented
    type_i_pin, TypeIInput<NonInverted,I2CHSDis,LowDrive>, P0_27, p0_27, dir0, pin0, set0, clr0, 27;
    type_i_pin, TypeIInput<NonInverted,I2CHSDis,LowDrive>, P0_28, p0_28, dir0, pin0, set0, clr0, 28;
    type_u_pin, TypeUInput<Nonce>, P0_29, p0_29, dir0, pin0, set0, clr0, 29;
    type_u_pin, TypeUInput<Nonce>, P0_30, p0_30, dir0, pin0, set0, clr0, 30;
    type_u_pin, TypeUInput<Nonce>, P0_31, p0_31, dir0, pin0, set0, clr0, 31;

    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P1_0, p1_0, dir1, pin1, set1, clr1, 0;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P1_1, p1_1, dir1, pin1, set1, clr1, 1;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P1_2, p1_2, dir1, pin1, set1, clr1, 2;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P1_3, p1_3, dir1, pin1, set1, clr1, 3;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P1_4, p1_4, dir1, pin1, set1, clr1, 4;
    type_w_pin, TypeWInput<PullUp,HysDis,NonInverted,NonFiltered>, P1_5, p1_5, dir1, pin1, set1, clr1, 5;
    type_w_pin, TypeWInput<PullUp,HysDis,NonInverted,NonFiltered>, P1_6, p1_6, dir1, pin1, set1, clr1, 6;
    type_w_pin, TypeWInput<PullUp,HysDis,NonInverted,NonFiltered>, P1_7, p1_7, dir1, pin1, set1, clr1, 7;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P1_8, p1_8, dir1, pin1, set1, clr1, 8;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P1_9, p1_9, dir1, pin1, set1, clr1, 9;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P1_10, p1_10, dir1, pin1, set1, clr1, 10;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P1_11, p1_11, dir1, pin1, set1, clr1, 11;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P1_12, p1_12, dir1, pin1, set1, clr1, 12;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P1_13, p1_13, dir1, pin1, set1, clr1, 13;
    type_w_pin, TypeWInput<PullUp,HysDis,NonInverted,NonFiltered>, P1_14, p1_14, dir1, pin1, set1, clr1, 14;
    type_w_pin, TypeWInput<PullUp,HysDis,NonInverted,NonFiltered>, P1_16, p1_16, dir1, pin1, set1, clr1, 16;
    type_w_pin, TypeWInput<PullUp,HysDis,NonInverted,NonFiltered>, P1_17, p1_17, dir1, pin1, set1, clr1, 17;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P1_18, p1_18, dir1, pin1, set1, clr1, 18;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P1_19, p1_19, dir1, pin1, set1, clr1, 19;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P1_20, p1_20, dir1, pin1, set1, clr1, 20;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P1_21, p1_21, dir1, pin1, set1, clr1, 21;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P1_22, p1_22, dir1, pin1, set1, clr1, 22;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P1_23, p1_23, dir1, pin1, set1, clr1, 23;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P1_24, p1_24, dir1, pin1, set1, clr1, 24;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P1_25, p1_25, dir1, pin1, set1, clr1, 25;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P1_26, p1_26, dir1, pin1, set1, clr1, 26;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P1_27, p1_27, dir1, pin1, set1, clr1, 27;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P1_28, p1_28, dir1, pin1, set1, clr1, 28;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P1_29, p1_29, dir1, pin1, set1, clr1, 29;
    type_a_pin, TypeADigitalInput<PullUp,HysDis,NonFiltered>, P1_30, p1_30, dir0, pin0, set0, clr0, 30;
    type_a_pin, TypeADigitalInput<PullUp,HysDis,NonFiltered>, P1_31, p1_31, dir0, pin0, set0, clr0, 31;

    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_0, p2_0, dir2, pin2, set2, clr2, 0;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_1, p2_1, dir2, pin2, set2, clr2, 1;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_2, p2_2, dir2, pin2, set2, clr2, 2;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_3, p2_3, dir2, pin2, set2, clr2, 3;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_4, p2_4, dir2, pin2, set2, clr2, 4;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_5, p2_5, dir2, pin2, set2, clr2, 5;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_6, p2_6, dir2, pin2, set2, clr2, 6;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_7, p2_7, dir2, pin2, set2, clr2, 7;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_8, p2_8, dir2, pin2, set2, clr2, 8;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_9, p2_9, dir2, pin2, set2, clr2, 9;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_10, p2_10, dir2, pin2, set2, clr2, 10;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_11, p2_11, dir2, pin2, set2, clr2, 11;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_12, p2_12, dir2, pin2, set2, clr2, 12;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_13, p2_13, dir2, pin2, set2, clr2, 13;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_14, p2_14, dir2, pin2, set2, clr2, 14;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_15, p2_15, dir2, pin2, set2, clr2, 15;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_16, p2_16, dir2, pin2, set2, clr2, 16;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_17, p2_17, dir2, pin2, set2, clr2, 17;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_18, p2_18, dir2, pin2, set2, clr2, 18;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_19, p2_19, dir2, pin2, set2, clr2, 19;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_20, p2_20, dir2, pin2, set2, clr2, 20;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_21, p2_21, dir2, pin2, set2, clr2, 21;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_22, p2_22, dir2, pin2, set2, clr2, 22;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_23, p2_23, dir2, pin2, set2, clr2, 23;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_24, p2_24, dir2, pin2, set2, clr2, 24;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_25, p2_25, dir2, pin2, set2, clr2, 25;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_26, p2_26, dir2, pin2, set2, clr2, 26;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_27, p2_27, dir2, pin2, set2, clr2, 27;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_28, p2_28, dir2, pin2, set2, clr2, 28;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_29, p2_29, dir2, pin2, set2, clr2, 29;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_30, p2_30, dir2, pin2, set2, clr2, 30;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P2_31, p2_31, dir2, pin2, set2, clr2, 31;

    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_0, p3_0, dir3, pin3, set3, clr3, 0;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_1, p3_1, dir3, pin3, set3, clr3, 1;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_2, p3_2, dir3, pin3, set3, clr3, 2;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_3, p3_3, dir3, pin3, set3, clr3, 3;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_4, p3_4, dir3, pin3, set3, clr3, 4;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_5, p3_5, dir3, pin3, set3, clr3, 5;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_6, p3_6, dir3, pin3, set3, clr3, 6;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_7, p3_7, dir3, pin3, set3, clr3, 7;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_8, p3_8, dir3, pin3, set3, clr3, 8;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_9, p3_9, dir3, pin3, set3, clr3, 9;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_10, p3_10, dir3, pin3, set3, clr3, 10;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_11, p3_11, dir3, pin3, set3, clr3, 11;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_12, p3_12, dir3, pin3, set3, clr3, 12;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_13, p3_13, dir3, pin3, set3, clr3, 13;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_14, p3_14, dir3, pin3, set3, clr3, 14;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_15, p3_15, dir3, pin3, set3, clr3, 15;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_16, p3_16, dir3, pin3, set3, clr3, 16;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_17, p3_17, dir3, pin3, set3, clr3, 17;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_18, p3_18, dir3, pin3, set3, clr3, 18;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_19, p3_19, dir3, pin3, set3, clr3, 19;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_20, p3_20, dir3, pin3, set3, clr3, 20;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_21, p3_21, dir3, pin3, set3, clr3, 21;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_22, p3_22, dir3, pin3, set3, clr3, 22;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_23, p3_23, dir3, pin3, set3, clr3, 23;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_24, p3_24, dir3, pin3, set3, clr3, 24;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_25, p3_25, dir3, pin3, set3, clr3, 25;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_26, p3_26, dir3, pin3, set3, clr3, 26;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_27, p3_27, dir3, pin3, set3, clr3, 27;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_28, p3_28, dir3, pin3, set3, clr3, 28;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_29, p3_29, dir3, pin3, set3, clr3, 29;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_30, p3_30, dir3, pin3, set3, clr3, 30;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P3_31, p3_31, dir3, pin3, set3, clr3, 31;

    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_0, p4_0, dir4, pin4, set4, clr4, 0;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_1, p4_1, dir4, pin4, set4, clr4, 1;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_2, p4_2, dir4, pin4, set4, clr4, 2;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_3, p4_3, dir4, pin4, set4, clr4, 3;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_4, p4_4, dir4, pin4, set4, clr4, 4;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_5, p4_5, dir4, pin4, set4, clr4, 5;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_6, p4_6, dir4, pin4, set4, clr4, 6;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_7, p4_7, dir4, pin4, set4, clr4, 7;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_8, p4_8, dir4, pin4, set4, clr4, 8;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_9, p4_9, dir4, pin4, set4, clr4, 9;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_10, p4_10, dir4, pin4, set4, clr4, 10;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_11, p4_11, dir4, pin4, set4, clr4, 11;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_12, p4_12, dir4, pin4, set4, clr4, 12;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_13, p4_13, dir4, pin4, set4, clr4, 13;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_14, p4_14, dir4, pin4, set4, clr4, 14;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_15, p4_15, dir4, pin4, set4, clr4, 15;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_16, p4_16, dir4, pin4, set4, clr4, 16;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_17, p4_17, dir4, pin4, set4, clr4, 17;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_18, p4_18, dir4, pin4, set4, clr4, 18;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_19, p4_19, dir4, pin4, set4, clr4, 19;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_20, p4_20, dir4, pin4, set4, clr4, 20;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_21, p4_21, dir4, pin4, set4, clr4, 21;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_22, p4_22, dir4, pin4, set4, clr4, 22;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_23, p4_23, dir4, pin4, set4, clr4, 23;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_24, p4_24, dir4, pin4, set4, clr4, 24;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_25, p4_25, dir4, pin4, set4, clr4, 25;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_26, p4_26, dir4, pin4, set4, clr4, 26;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_27, p4_27, dir4, pin4, set4, clr4, 27;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_28, p4_28, dir4, pin4, set4, clr4, 28;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_29, p4_29, dir4, pin4, set4, clr4, 29;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_30, p4_30, dir4, pin4, set4, clr4, 30;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P4_31, p4_31, dir4, pin4, set4, clr4, 31;

    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P5_0, p5_0, dir5, pin5, set5, clr5, 0;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P5_1, p5_1, dir5, pin5, set5, clr5, 1;
    type_i_pin, TypeIInput<NonInverted,I2CHSDis,LowDrive>, P5_2, p5_2, dir5, pin5, set5, clr5, 2;
    type_i_pin, TypeIInput<NonInverted,I2CHSDis,LowDrive>, P5_3, p5_3, dir5, pin5, set5, clr5, 3;
    type_d_pin, TypeDInput<PullUp,HysDis,NonInverted>, P5_4, p5_4, dir5, pin5, set5, clr5, 4;
);
