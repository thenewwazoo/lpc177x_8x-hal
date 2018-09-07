#![no_std]
#![feature(never_type)]

extern crate embedded_hal as hal;
extern crate lpc177x_8x as lpc;
extern crate nb;

pub mod common;
pub mod gpio;
pub mod serial;
