//! Provides bindings for the raw commands of the stepper drivers.
//!
//! You usually don't have to talk to the motor directly, there is almost certainly
//! a better way.\
//! It might also be important to note that error checks on values aren't performed
//! in this module. If you send a value that isn't allowed, you will get an error
//! by the driver.

mod map;
#[cfg(test)]
mod tests;

use nom::{self, error::FromExternalError, IResult, Parser};
use num_derive::FromPrimitive;
use num_traits::FromPrimitive;
use std::fmt::{Debug, Display};
use thiserror::Error;
use map as cm;

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
    s: &'a str,
    parser: P,
    constructor: C,
) -> IResult<&'a str, O2, nom::error::Error<&'a str>>
where
    P: Fn(&'a str) -> IResult<&'a str, O, nom::error::Error<&'a str>>,
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
    pub fn parse(s: &str) -> IResult<&str, Self> {
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

impl MotorStop {
    pub fn parse(s: &str) -> IResult<&str, Self> {
        parse_enum_value(s, nom::character::complete::u8, MotorStop::from_u8)
    }
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
    pub fn parse(s: &str) -> IResult<&str, Self> {
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
    pub fn parse(s: &str) -> IResult<&str, Self> {
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
    pub fn parse(s: &str) -> IResult<&str, Self> {
        parse_enum_value(s, nom::character::complete::u8, RotationDirection::from_u8)
    }
}

impl Display for RotationDirection {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", *self as u8)
    }
}

/// The payload-part of a command
///
/// Meaning everything but the `"#"`, and the address at the beginning and the
/// `"\r"` at the end, e.g. `"u133742"` or `"A"`.
///
/// Error checks on values are not performed, menaing if you send an invalid value
/// to the driver, you get an error back.
///
/// If a value of a command can also be read, it is wrapped into an [Option].
/// [None] means that the corresponding command will be a read-command,
/// [Some] means it will be a write-command. If parsing from text to rust, it
/// won't matter if it was a response to a read- or write-command, both will be
/// turned into [Some].
#[derive(Debug, PartialEq, Eq, Hash, Copy, Clone)]
pub enum Payload {
    //MotorType(Option<MotorType>),
    StartMotor,
    StopMotor(MotorStop),
    LoadRecord(u8),
    /// This Command doesn't exist in the Firmware. This is due to
    /// [1.6.4 Reading out the current record](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf#%5B%7B%22num%22%3A123%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C113%2C742%2Cnull%5D)
    /// being such a weird command. Here it is seperated into two commands, [`SetRespondMode`][Payload::SetRespondMode]
    /// and `ReadCurrentRecord`, which doesn't exist yet. This command corresponds
    /// to `'|'` with just `0` or `1`, while ReadCurrentRecord does the reading of records part.
    SetRespondMode(RespondMode),
    //ReadCurrentRecord,
    SaveRecord(u8),
    PositioningMode(Option<PositioningMode>),
    TravelDistance(Option<i32>),
    MinFrequency(Option<u32>),
    MaxFrequency(Option<u32>),
    MaxFrequency2(Option<u32>),
    AccelRamp(Option<u16>),
    BrakeRamp(Option<u16>),
    RotationDirection(Option<RotationDirection>),
    RotationDirectionChange(Option<bool>),
    Repetitions(Option<u32>),
    RecordPause(Option<u16>),
    ContinuationRecord(Option<u8>),
    MaxAccelJerk(Option<u32>),
    MaxBrakeJerk(Option<u32>),
}

impl Payload {
    /// Parse a `&str` to payload.\
    /// Also see [nom]
    ///
    /// Responses to read-commands and responses to write-commands are not differentiated.
    /// This means that `"Zs1337"` and `"s1337"` would both be converted to
    /// `Payload::TravelDistance(Some(1337))`
    #[rustfmt::skip]
    pub fn parse(s: &str) -> IResult<&str, Self> {
        use Payload::*;
        use nom::{branch::alt, bytes::complete::tag, sequence::{tuple, preceded}, combinator::opt, character::complete::{u8, u16, u32, i32}};


        #[inline]
        fn opt_read<'a, P, O, E>(p: P) -> impl Parser<&'a str, O, E>
        where
            P: Parser<&'a str, O, E>,
            E: nom::error::ParseError<&'a str>,
        {
            preceded(opt(tag(cm::READ)), p)
        }

        macro_rules! parse_readable {
            ($tag:expr, $subparser:expr, $const:ident) => {
                opt_read(preceded(tag($tag), $subparser).map(|n| $const(Some(n))))
            };
        }

        // There is no (semi-)easy way to make this less ugly, i've tried
        // Watch out: If you forget a Command here, you might only notice once
        // it doesn't work in the wild
        alt((
            tag(cm::START_MOTOR).map(|_| StartMotor),
            preceded(tag(cm::STOP_MOTOR), MotorStop::parse).map(|mt| StopMotor(mt)),
            preceded(tag(cm::LOAD_RECORD), u8).map(|n| LoadRecord(n)),
            preceded(tag(cm::READ_CURRENT_RECORD), RespondMode::parse).map(|n| SetRespondMode(n)),
            //tuple((tag(cm::READ_CURRENT_RECORD), RespondMode::parse)).map(|(_, rm)| ReadCurrentRecord(Some(rm))),
            preceded(tag(cm::SAVE_RECORD), u8).map(|n| SaveRecord(n)),
            parse_readable!(cm::POSITIONING_MODE, self::PositioningMode::parse, PositioningMode),
            parse_readable!(cm::TRAVEL_DISTANCE, i32, TravelDistance),
            parse_readable!(cm::MIN_FREQUENCY, u32, MinFrequency),
            parse_readable!(cm::MAX_FREQUENCY, u32, MaxFrequency),
            parse_readable!(cm::MAX_FREQUENCY2, u32, MaxFrequency2),
            parse_readable!(cm::ACCEL_RAMP, u16, AccelRamp),
            parse_readable!(cm::BRAKE_RAMP, u16, BrakeRamp),
            parse_readable!(cm::ROTATION_DIRECTION, self::RotationDirection::parse, RotationDirection),
            opt_read(preceded(tag(cm::ROTATION_DIRECTION_CHANGE), u8).map(|n| RotationDirectionChange(Some(n == 1)))),
            parse_readable!(cm::REPETITIONS, u32, Repetitions),
            parse_readable!(cm::RECORD_PAUSE, u16, RecordPause),
            parse_readable!(cm::CONTINUATION_RECORD, u8, ContinuationRecord),
            parse_readable!(cm::MAX_ACCEL_JERK, u32, MaxAccelJerk),
            parse_readable!(cm::MAX_BRAKE_JERK, u32, MaxBrakeJerk),
        ))(s)
    }
}

impl Display for Payload {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // Provide read-command if argument is None, else write command
        macro_rules! mkcmd {
            ($letter:expr, $arg:expr) => {
                match $arg {
                    Some(a) => write!(f, "{}{}", $letter, a),
                    None => write!(f, "Z{}", $letter),
                }
            };
        }
        macro_rules! mklcmd {
            ($cmd:literal, $arg:expr) => {
                match $arg {
                    Some(a) => write!(f, "{}={}", $cmd, a),
                    None => write!(f, "{}", $cmd),
                }
            };
        }
        use Payload::*;
        match &self {
            //MotorType(o) => mklcmd!(":CL_motor_type", o),
            StartMotor => write!(f, "{}", cm::START_MOTOR),
            StopMotor(ms) => write!(f, "{}{}", cm::STOP_MOTOR, ms),
            LoadRecord(r) => write!(f, "{}{}", cm::LOAD_RECORD, r),
            SetRespondMode(rm) => write!(f, "{}{}", cm::READ_CURRENT_RECORD, rm),
            //ReadCurrentRecord(o) => mkcmd!(cm::READ_CURRENT_RECORD, o),
            SaveRecord(r) => write!(f, "{}{}", cm::SAVE_RECORD, r),
            PositioningMode(o) => mkcmd!(cm::POSITIONING_MODE, o),
            TravelDistance(o) => mkcmd!(cm::TRAVEL_DISTANCE, o),
            MinFrequency(o) => mkcmd!(cm::MIN_FREQUENCY, o),
            MaxFrequency(o) => mkcmd!(cm::MAX_FREQUENCY, o),
            MaxFrequency2(o) => mkcmd!(cm::MAX_FREQUENCY2, o),
            AccelRamp(o) => mkcmd!(cm::ACCEL_RAMP, o),
            BrakeRamp(o) => mkcmd!(cm::BRAKE_RAMP, o),
            RotationDirection(o) => mkcmd!(cm::ROTATION_DIRECTION, o),
            RotationDirectionChange(o) => mkcmd!(cm::ROTATION_DIRECTION_CHANGE, o.map(|b| b as u8)),
            Repetitions(o) => mkcmd!(cm::REPETITIONS, o),
            RecordPause(o) => mkcmd!(cm::RECORD_PAUSE, o),
            ContinuationRecord(o) => mkcmd!(cm::CONTINUATION_RECORD, o),
            MaxAccelJerk(o) => mkcmd!(cm::MAX_ACCEL_JERK, o),
            MaxBrakeJerk(o) => mkcmd!(cm::MAX_BRAKE_JERK, o),
        }
    }
}

#[derive(Debug, PartialEq, Eq, Hash, Copy, Clone)]
pub struct Cmd {
    address: Option<u8>,
    payload: Payload,
}

impl Display for Cmd {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.address {
            Some(address) => write!(f, "#{}", address)?,
            None => write!(f, "#*")?,
        }
        write!(f, "{}\r", self.payload)
    }
}

#[derive(Debug, PartialEq, Eq, Hash, Copy, Clone)]
pub struct Response {
    address: Option<u8>,
    payload: Payload,
}

