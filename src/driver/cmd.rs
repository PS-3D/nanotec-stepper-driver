//! Provides bindings for the raw commands of the stepper drivers.
//!
//! You usually don't have to talk to the motor directly, there is almost certainly
//! a better way.\
//! It might also be important to note that error checks on values aren't performed
//! in this module. If you send a value that isn't allowed, you will get an error
//! by the driver.

#[cfg(test)]
mod tests;

use nom::{self, error::FromExternalError, IResult, Parser};
use num_derive::FromPrimitive;
use num_traits::FromPrimitive;
use std::fmt::{Debug, Display};
use thiserror::Error;

/// Gets thrown when there is an error while parsing the various enums which
/// represent values for the commands
#[derive(Error, Debug)]
pub enum ParseError {
    /// Gets thrown when a a value of a command doesn't have a matching enum variant,
    /// which usually means it's too big.
    #[error("Invalid Value while Parsing, probably too big")]
    InvalidValue,
}

impl ParseError {
    fn into_nom_error<I>(self, input: I) -> nom::Err<nom::error::Error<I>> {
        match self {
            ParseError::InvalidValue => nom::Err::Failure(nom::error::Error::from_external_error(
                input,
                nom::error::ErrorKind::TooLarge,
                self,
            )),
        }
    }
}

#[inline]
fn parse_enum_value<'a, P, C, O, O2>(
    s: &'a [u8],
    parser: P,
    constructor: C,
) -> IResult<&'a [u8], O2, nom::error::Error<&'a [u8]>>
where
    P: Fn(&'a [u8]) -> IResult<&'a [u8], O, nom::error::Error<&'a [u8]>>,
    C: Fn(O) -> Option<O2>,
{
    let (rem, res) = parser(s)?;
    Ok((
        rem,
        constructor(res).ok_or(ParseError::InvalidValue.into_nom_error(s))?,
    ))
}

/// Binding for values of [1.5.1 Setting the motor type](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf#%5B%7B%22num%22%3A45%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C113%2C699%2Cnull%5D)
#[derive(Debug, PartialEq, Eq, Hash, Copy, Clone, FromPrimitive)]
pub enum MotorType {
    Stepper,
    BLDCHall,
    BLDCHallEncoder,
}

impl MotorType {
    pub(super) fn parse(s: &[u8]) -> IResult<&[u8], Self> {
        parse_enum_value(s, nom::character::complete::u8, MotorType::from_u8)
    }
}

impl Display for MotorType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", *self as u8)
    }
}

/// Binding for values of [1.6.2 Stopping a motor](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf#%5B%7B%22num%22%3A120%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C113%2C506%2Cnull%5D)
#[derive(Debug, PartialEq, Eq, Hash, Copy, Clone, FromPrimitive)]
pub enum MotorStop {
    QuickStop,
    BrakeRamp,
}

impl Display for MotorStop {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", *self as u8)
    }
}

/// Binding for values of [1.6.4 Reading out the current record](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf#%5B%7B%22num%22%3A123%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C113%2C742%2Cnull%5D)
#[derive(Debug, PartialEq, Eq, Hash, Copy, Clone, FromPrimitive)]
pub enum RespondMode {
    Quiet,
    NotQuiet,
}

impl RespondMode {
    pub(super) fn parse<'b>(s: &'b [u8]) -> IResult<&'b [u8], Self> {
        parse_enum_value(s, nom::character::complete::u8, RespondMode::from_u8)
    }
}

impl Display for RespondMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", *self as u8)
    }
}

/// Binding for values of [1.6.6 Setting the positioning mode](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf#%5B%7B%22num%22%3A128%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C113%2C738%2Cnull%5D)
#[derive(Debug, PartialEq, Eq, Hash, Copy, Clone, FromPrimitive)]
pub enum PositioningMode {
    Relative = 1,
    Absolute,
    InternalReference,
    ExternalReference,
    Speed,
    Flag,
    ClockDirectionManualLeft,
    ClockDirectionManualRight,
    ClockDirectionInternalReference,
    ClockDirectionExternalReference,
    Analog,
    Joystick,
    AnalogPositioning,
    HWReference,
    Torque,
    CLQuickTest,
    CLTest,
    CLAutotune,
    CLQuickTest2,
}

impl PositioningMode {
    pub(super) fn parse(s: &[u8]) -> IResult<&[u8], Self> {
        parse_enum_value(s, nom::character::complete::u8, PositioningMode::from_u8)
    }
}

impl Display for PositioningMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", *self as u8)
    }
}

/// Binding for values of [1.6.15 Setting the direction of rotation](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf#%5B%7B%22num%22%3A143%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C113%2C742%2Cnull%5D)
#[derive(Debug, PartialEq, Eq, Hash, Copy, Clone, FromPrimitive)]
pub enum RotationDirection {
    Left,
    Right,
}

impl RotationDirection {
    pub(super) fn parse(s: &[u8]) -> IResult<&[u8], Self> {
        parse_enum_value(s, nom::character::complete::u8, RotationDirection::from_u8)
    }
}

impl Display for RotationDirection {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", *self as u8)
    }
}

#[derive(Debug, PartialEq, Eq, Hash, Clone)]
pub(super) struct Msg {
    pub address: Option<u8>,
    pub payload: Vec<u8>,
}

impl Msg {
    pub fn parse(s: &[u8]) -> IResult<&[u8], Self> {
        use nom::{
            branch::alt,
            bytes::complete::{tag, take_until1},
            character::complete::u8,
            sequence::{terminated, tuple},
        };
        tuple((
            alt((tag(b"*").map(|_: &[u8]| None), u8.map(Option::from))),
            terminated(take_until1(b"\r" as &[u8]), tag(b"\r")),
        ))
        .map(|(a, p)| Self {
            address: a,
            payload: p.to_vec(),
        })
        .parse(s)
    }
}
