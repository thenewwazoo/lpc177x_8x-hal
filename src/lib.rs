#![no_std]
#![feature(never_type)]

extern crate cortex_m;
extern crate embedded_hal as hal;
extern crate lpc177x_8x as lpc;
extern crate nb;
extern crate void;

pub mod adc;
pub mod common;
pub mod gpio;
pub mod serial;
pub mod syscon;
pub mod time;
pub mod timer;
