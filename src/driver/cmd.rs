//! Provides types for the raw commands of the driver.
//!
//! [`frame`] is responsible for everything except the payload, e.g. the message
//! as a whole or the [`MotorAddress`][frame::MotorAdrress].
//! [`payload`] is responsible for all the types needed for the various payloads.

pub mod frame;
pub mod payload;
