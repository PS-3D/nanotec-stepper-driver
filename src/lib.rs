//! Rust driver for Nanotec Stepper Motor Controllers as of firmware 25.01.2013
//!
//! For more information see the [Manual](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)

mod driver;
pub(crate) mod util;

pub use driver::{cmd::*, motor::Motor, responsehandle::ResponseHandle, Driver, DriverError};
