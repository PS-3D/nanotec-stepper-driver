use nom::{self, error::FromExternalError, IResult, Parser};
use num_derive::FromPrimitive;
use num_traits::FromPrimitive;
use std::fmt::{Debug, Display};
use thiserror::Error;

mod cmd_map {
    macro_rules! makepl {
        ($($name:ident, $val:literal),*) => {
            $(pub const $name: &str = $val;)*
        };
    }
    #[rustfmt::skip]
    makepl!(
        READ, "Z",
        START_MOTOR, "A",
        STOP_MOTOR, "S",
        LOAD_RECORD, "y",
        READ_CURRENT_RECORD, "|",
        SAVE_RECORD, ">",
        POSITIONING_MODE, "p",
        TRAVEL_DISTANCE, "s",
        MIN_FREQUENCY, "u",
        MAX_FREQUENCY, "o",
        MAX_FREQUENCY2, "n",
        ACCEL_RAMP, "b",
        BRAKE_RAMP, "b",
        ROTATION_DIRECTION, "d",
        ROTATION_DIRECTION_CHANGE, "t",
        REPETITIONS, "W",
        RECORD_PAUSE, "P",
        CONTINUATION_RECORD, "N",
        MAX_ACCEL_JERK, ":b",
        MAX_BRAKE_JERK, ":B"
    );
}

use cmd_map as cm;

#[derive(Error, Debug)]
pub enum ParseError {
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

#[derive(Debug, PartialEq, Eq, Hash, Copy, Clone)]
pub enum Payload {
    //MotorType(Option<MotorType>),
    StartMotor,
    StopMotor(MotorStop),
    LoadRecord(u8),
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

#[cfg(test)]
mod tests {

    mod payload {
        use super::super::{MotorStop, Payload};

        #[test]
        fn parse_standalone() {
            let (remainder, res) = Payload::parse("A").unwrap();
            assert!(remainder.len() == 0);
            assert_eq!(res, Payload::StartMotor);
        }

        #[test]
        fn parse_subparser() {
            let (remainder, res) = Payload::parse("S1").unwrap();
            assert!(remainder.len() == 0);
            assert_eq!(res, Payload::StopMotor(MotorStop::BrakeRamp));
        }

        #[test]
        fn parse_read_subparser() {
            let (remainder, res) = Payload::parse("Zu133742").unwrap();
            assert!(remainder.len() == 0);
            assert_eq!(res, Payload::MinFrequency(Some(133742)));
        }

        #[test]
        #[should_panic]
        fn parse_invalid_key() {
            let (_, _) = Payload::parse("some_key_that_doesn't_exist133742").unwrap();
        }

        #[test]
        #[should_panic]
        fn parse_read_invalid_key() {
            let (_, _) = Payload::parse("Zsome_key_that_doesn't_exist133742").unwrap();
        }

        #[test]
        #[should_panic]
        fn parse_missing_value() {
            let (_, _) = Payload::parse("S").unwrap();
        }

        #[test]
        #[should_panic]
        fn parse_read_missing_value() {
            let (_, _) = Payload::parse("Zo").unwrap();
        }

        #[test]
        fn parse_rotation_direction_change() {
            let (remainder, res) = Payload::parse("t1").unwrap();
            assert!(remainder.len() == 0);
            assert_eq!(res, Payload::RotationDirectionChange(Some(true)));
        }

        #[test]
        fn fmt_standalone() {
            let pl = Payload::StartMotor;
            assert_eq!(format!("{}", pl), "A");
        }

        #[test]
        fn fmt_with_value() {
            let pl = Payload::StopMotor(MotorStop::BrakeRamp);
            assert_eq!(format!("{}", pl), "S1");
        }

        #[test]
        fn fmt_write() {
            let pl = Payload::TravelDistance(Some(-1337));
            assert_eq!(format!("{}", pl), "s-1337");
        }

        #[test]
        fn fmt_read() {
            let pl = Payload::MinFrequency(None);
            assert_eq!(format!("{}", pl), "Zu");
        }
    }

    mod cmd {
        use super::super::{Cmd, Payload};

        #[test]
        fn fmt_single() {
            let cmd = Cmd {
                address: Some(42),
                payload: Payload::StartMotor,
            };
            assert_eq!(format!("{}", cmd), "#42A\r");
        }

        #[test]
        fn fmt_all() {
            let cmd = Cmd {
                address: None,
                payload: Payload::StartMotor,
            };
            assert_eq!(format!("{}", cmd), "#*A\r")
        }
    }
}
