//! Rust driver for Nanotec Stepper Motor Controllers as of firmware 25.01.2013.
//! Older versions might also work, but are not guaranteed. The motor might not
//! understand some functions. Some functions did exist in older versions but
//! returned a different value, for example [`get_current_record`][Motor::get_current_record].
//! Both won't be usable with older firmware versions.
//!
//! For more information see the [manual](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)
//!
//! # Usage
//! Each RS485 port with connected motors gets its own [`Driver`]. Motors can then
//! be added to a driver by calling [`Driver::add_motor`]. A [`Motor`] then has
//! all the functions found in the manual, more or less named the same, sometimes
//! a bit shortened. Every function in a [`Motor`] returns a [`ResponseHandle`],
//! on which [`ResponseHandle::wait`] can be called to wait for the answer of the
//! motor and, depending on the command, return a value. If multiple commands are
//! sent to the same motor, the returned handles should be waited on in order,
//! otherwise they might return an error.
//!
//! # Examples
//! ```no_run
//! # use nanotec_stepper_driver::{Driver, ResponseHandle};
//! use std::time::Duration;
//! use serialport;
//!
//! let s = serialport::new("/dev/ttyUSB0", 115200)
//!     .timeout(Duration::from_secs(1))
//!     .open()
//!     .unwrap();
//! let mut driver = Driver::new(s);
//! let mut m1 = driver.add_motor(1).unwrap();
//!
//! m1.load_record(3).unwrap().wait().unwrap();
//! m1.set_continuation_record(0).unwrap().wait().unwrap();
//! m1.start_motor().unwrap().wait().unwrap();
//!
//! let steps = m1.get_travel_distance().unwrap().wait().unwrap();
//! let max_freq = m1.get_max_frequency().unwrap().wait().unwrap();
//! println!("Drove {} steps with a top speed of {} steps/min", steps, max_freq);
//! ```
//!

mod driver;
pub(crate) mod util;

pub use driver::{cmd::*, motor::Motor, responsehandle::ResponseHandle, Driver, DriverError};
