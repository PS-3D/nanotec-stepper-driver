#[cfg(test)]
mod tests;

use super::super::{
    map,
    parse::{parse_enum_value, parse_su16, parse_su32, parse_su8, ParseError},
};
use chrono::naive::NaiveDate;
use nom::{
    self,
    character::complete::{i32 as parse_i32, u16 as parse_u16, u8 as parse_u8},
    IResult, Parser,
};
use num_derive::FromPrimitive;
use num_traits::FromPrimitive;
use std::fmt::{Debug, Display};

// unfortunately, due to rustfmt not having the blank_lines_upper_bound feature
// stable yet, we gotta put comments in between the different sections. otherwise
// its just too much

//

/// Binding for values of [1.5.1 Setting the motor type](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)
#[derive(Debug, PartialEq, Eq, Hash, Copy, Clone, FromPrimitive)]
pub enum MotorType {
    Stepper,
    BLDCHall,
    BLDCHallEncoder,
}

impl MotorType {
    pub(crate) fn parse(s: &[u8]) -> IResult<&[u8], Self, ParseError<&[u8]>> {
        parse_enum_value(s, parse_su8, MotorType::from_u8)
    }
}

impl Display for MotorType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", *self as u8)
    }
}

//

/// Binding for values of [1.5.6 Setting the step mode](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)
#[derive(Debug, PartialEq, Eq, Hash, Copy, Clone, FromPrimitive)]
pub enum StepMode {
    S1 = 1,
    S2 = 2,
    S4 = 4,
    S5 = 5,
    S8 = 8,
    S10 = 10,
    S16 = 16,
    S32 = 32,
    S64 = 64,
    Feedrate = 254,
    Adaptive = 255,
}

impl StepMode {
    pub(crate) fn parse(s: &[u8]) -> IResult<&[u8], Self, ParseError<&[u8]>> {
        parse_enum_value(s, parse_su8, StepMode::from_u8)
    }
}

impl Display for StepMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", *self as u8)
    }
}

//

/// Helper binding for values of [1.5.9 Setting the limit switch behavior](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)
///
/// Represents the 2 possibilities given under "Bahavior of the ... limit switch during
/// a reference run"
#[derive(Debug, PartialEq, Eq, Hash, Copy, Clone)]
pub enum LimitSwitchBehaviorReference {
    FreeTravelForwards,
    FreeTravelBackwards,
}

//

/// Helper binding for values of [1.5.9 Setting the limit switch behavior](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)
///
/// Represents the 4 possibilities given under "Bahavior of the ... limit switch during
/// a normal run"
#[derive(Debug, PartialEq, Eq, Hash, Copy, Clone)]
pub enum LimitSwitchBehaviorNormal {
    FreeTravelForwards,
    FreeTravelBackwards,
    Stop,
    Ignore,
}

//

/// Binding for values of [1.5.9 Setting the limit switch behavior](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)
///
/// The 4 parts of the bit mask are split into 4 different enums in this struct,
/// which is then converted to a u32.
///
/// # Examples
/// ```
/// # use nanotec_stepper_driver::{LimitSwitchBehavior, LimitSwitchBehaviorReference,
/// #     LimitSwitchBehaviorNormal};
/// let example = LimitSwitchBehavior {
///     internal_reference: LimitSwitchBehaviorReference::FreeTravelForwards,
///     internal_normal: LimitSwitchBehaviorNormal::FreeTravelForwards,
///     external_reference: LimitSwitchBehaviorReference::FreeTravelForwards,
///     external_normal: LimitSwitchBehaviorNormal::FreeTravelForwards,
/// };
/// assert_eq!(0xa05, u32::from(example))
/// ```
/// `example` would correspond to bits 0, 2, 9 and 11 being set.
#[derive(Debug, PartialEq, Eq, Hash, Copy, Clone)]
pub struct LimitSwitchBehavior {
    /// Corresponds to the first 2 bits
    pub internal_reference: LimitSwitchBehaviorReference,
    /// Corresponds to bits 2-5
    pub internal_normal: LimitSwitchBehaviorNormal,
    /// Corresponds to bits 9-10
    pub external_reference: LimitSwitchBehaviorReference,
    /// Corresponds to bits 11-14
    pub external_normal: LimitSwitchBehaviorNormal,
}

impl LimitSwitchBehavior {
    pub(crate) fn parse(s: &[u8]) -> IResult<&[u8], Self, ParseError<&[u8]>> {
        let (rem, res) = parse_su32(s)?;
        if let Some(l) = Self::from_u32(res) {
            Ok((rem, l))
        } else {
            Err(nom::Err::Error(nom::error::ParseError::from_error_kind(
                s,
                // idk, for lack of some better ErrorKind its just fail
                nom::error::ErrorKind::Fail,
            )))
        }
    }

    /// Converts from [`u32`] to LimitSwitchBehavior
    ///
    /// # Examples
    /// ```
    /// # use nanotec_stepper_driver::{LimitSwitchBehavior, LimitSwitchBehaviorReference,
    /// #     LimitSwitchBehaviorNormal};
    /// let example = LimitSwitchBehavior {
    ///     internal_reference: LimitSwitchBehaviorReference::FreeTravelForwards,
    ///     internal_normal: LimitSwitchBehaviorNormal::FreeTravelForwards,
    ///     external_reference: LimitSwitchBehaviorReference::FreeTravelForwards,
    ///     external_normal: LimitSwitchBehaviorNormal::FreeTravelForwards,
    /// };
    /// assert_eq!(LimitSwitchBehavior::from_u32(0xa05).unwrap(), example)
    pub fn from_u32(b: u32) -> Option<Self> {
        fn reference(b: u32) -> Option<LimitSwitchBehaviorReference> {
            match b & 0x3 {
                0x1 => Some(LimitSwitchBehaviorReference::FreeTravelForwards),
                0x2 => Some(LimitSwitchBehaviorReference::FreeTravelBackwards),
                _ => None,
            }
        }
        fn normal(b: u32) -> Option<LimitSwitchBehaviorNormal> {
            match b & 0xf {
                0x1 => Some(LimitSwitchBehaviorNormal::FreeTravelForwards),
                0x2 => Some(LimitSwitchBehaviorNormal::FreeTravelBackwards),
                0x4 => Some(LimitSwitchBehaviorNormal::Stop),
                0x8 => Some(LimitSwitchBehaviorNormal::Ignore),
                _ => None,
            }
        }
        Some(Self {
            internal_reference: reference(b)?,
            internal_normal: normal(b >> 2)?,
            external_reference: reference(b >> 9)?,
            external_normal: normal(b >> 11)?,
        })
    }
}

impl From<LimitSwitchBehavior> for u32 {
    fn from(l: LimitSwitchBehavior) -> Self {
        fn reference(b: LimitSwitchBehaviorReference) -> u32 {
            match b {
                LimitSwitchBehaviorReference::FreeTravelForwards => 0x1,
                LimitSwitchBehaviorReference::FreeTravelBackwards => 0x2,
            }
        }
        fn normal(b: LimitSwitchBehaviorNormal) -> u32 {
            match b {
                LimitSwitchBehaviorNormal::FreeTravelForwards => 0x1,
                LimitSwitchBehaviorNormal::FreeTravelBackwards => 0x2,
                LimitSwitchBehaviorNormal::Stop => 0x4,
                LimitSwitchBehaviorNormal::Ignore => 0x8,
            }
        }
        reference(l.internal_reference)
            | (normal(l.internal_normal) << 2)
            | (reference(l.external_reference) << 9)
            | (normal(l.external_normal) << 11)
    }
}

impl From<u32> for LimitSwitchBehavior {
    /// Convertsfrom [`u32`] to LimitSwitchBehavior
    ///
    /// # Panics
    /// Panics if the given u32 is not a valid LimitSwitchBehavior i.e. multiple bits
    /// per section are set.
    fn from(b: u32) -> Self {
        Self::from_u32(b).unwrap()
    }
}

impl Display for LimitSwitchBehavior {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", u32::from(*self))
    }
}

//

/// Binding for values of [1.5.10 Setting the error correction mode](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)
#[derive(Debug, PartialEq, Eq, Hash, Copy, Clone, FromPrimitive)]
pub enum ErrorCorrectionMode {
    Off,
    AfterTravel,
    DuringTravel,
}

impl ErrorCorrectionMode {
    pub(crate) fn parse(s: &[u8]) -> IResult<&[u8], Self, ParseError<&[u8]>> {
        parse_enum_value(s, parse_su8, Self::from_u8)
    }
}

impl Display for ErrorCorrectionMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", *self as u8)
    }
}

//

/// Binding for values of [1.5.18 Reading out the error memory](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)
#[derive(Debug, PartialEq, Eq, Hash, Copy, Clone, FromPrimitive)]
pub enum MotorError {
    LowVoltage = 0x1,
    Temperature,
    TMC = 0x4,
    EE = 0x8,
    QEI = 0x10,
    Internal = 0x20,
    Driver = 0x80,
}

impl MotorError {
    pub(crate) fn parse(s: &[u8]) -> IResult<&[u8], Self, ParseError<&[u8]>> {
        parse_enum_value(s, parse_su8, Self::from_u8)
    }
}

impl Display for MotorError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", *self as u8)
    }
}

//

/// Helper binding for values of [1.5.23 Reading out the firmware version](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)
#[derive(Debug, PartialEq, Eq, Hash, Copy, Clone)]
#[allow(non_camel_case_types)]
pub enum HardwareType {
    SMCI47_S,
    PD6_N,
    PD4_N,
    PD2_N,
    SMCI33,
    SMCI35,
    SMCI36,
    SMCI12,
    SMCP33,
}

impl HardwareType {
    pub(crate) fn parse(s: &[u8]) -> IResult<&[u8], Self, ParseError<&[u8]>> {
        use nom::{branch::alt, bytes::complete::tag};
        alt((
            tag("SMCI47-S").map(|_| HardwareType::SMCI47_S),
            tag("PD6-N").map(|_| HardwareType::PD6_N),
            tag("PD4-N").map(|_| HardwareType::PD4_N),
            tag("PD2-N").map(|_| HardwareType::PD2_N),
            tag("SMCI33").map(|_| HardwareType::SMCI33),
            tag("SMCI35").map(|_| HardwareType::SMCI35),
            tag("SMCI36").map(|_| HardwareType::SMCI36),
            tag("SMCI12").map(|_| HardwareType::SMCI12),
            tag("SMCP33").map(|_| HardwareType::SMCP33),
        ))
        .parse(s)
    }
}

impl Display for HardwareType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            HardwareType::SMCI47_S => write!(f, "SMCI47-S"),
            HardwareType::PD6_N => write!(f, "PD6-N"),
            HardwareType::PD4_N => write!(f, "PD4-N"),
            HardwareType::PD2_N => write!(f, "PD2-N"),
            HardwareType::SMCI33 => write!(f, "SMCI33"),
            HardwareType::SMCI35 => write!(f, "SMCI35"),
            HardwareType::SMCI36 => write!(f, "SMCI36"),
            HardwareType::SMCI12 => write!(f, "SMCI12"),
            HardwareType::SMCP33 => write!(f, "SMCP33"),
        }
    }
}

//

/// Helper binding for values of [1.5.23 Reading out the firmware version](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)
#[derive(Debug, PartialEq, Eq, Hash, Copy, Clone)]
pub enum CommunicationType {
    USB,
    RS485,
}

impl CommunicationType {
    pub(crate) fn parse(s: &[u8]) -> IResult<&[u8], Self, ParseError<&[u8]>> {
        use nom::{branch::alt, bytes::complete::tag};
        alt((
            tag("USB").map(|_| CommunicationType::USB),
            tag("RS485").map(|_| CommunicationType::RS485),
        ))
        .parse(s)
    }
}

impl Display for CommunicationType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CommunicationType::USB => write!(f, "USB"),
            CommunicationType::RS485 => write!(f, "RS485"),
        }
    }
}

//

/// Binding for values of [1.5.23 Reading out the firmware version](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)
///
/// Each of the four sections of the firmware version corresponds to a member
/// of the struct.
///
/// # Examples
/// ```
/// # use nanotec_stepper_driver::{FirmwareVersion, HardwareType, CommunicationType};
/// # use chrono::NaiveDate;
/// let version = FirmwareVersion {
///     hardware: HardwareType::SMCI47_S,
///     communication: CommunicationType::RS485,
///     release_date: NaiveDate::from_ymd(2011, 5, 17),
///     revision: 3711,
/// };
/// assert_eq!("SMCI47-S_RS485_17-05-2011-rev3711", format!("{}", version))
/// ```
#[derive(Debug, PartialEq, Eq, Hash, Copy, Clone)]
pub struct FirmwareVersion {
    pub hardware: HardwareType,
    pub communication: CommunicationType,
    pub release_date: NaiveDate,
    pub revision: u16,
}

impl FirmwareVersion {
    pub(crate) fn parse(s: &[u8]) -> IResult<&[u8], Self, ParseError<&[u8]>> {
        use nom::{bytes::complete::tag, sequence::tuple};
        tuple((
            HardwareType::parse,
            tag("_"),
            CommunicationType::parse,
            tag("_"),
            parse_u8,
            tag("-"),
            parse_u8,
            tag("-"),
            parse_u16,
            tag("-rev"),
            parse_u16,
        ))
        .map(
            |(hardware, _, communication, _, day, _, month, _, year, _, revision)| Self {
                hardware,
                communication,
                release_date: NaiveDate::from_ymd(year.into(), month.into(), day.into()),
                revision,
            },
        )
        .parse(s)
    }
}

impl Display for FirmwareVersion {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}_{}_{}-rev{}",
            self.hardware,
            self.communication,
            self.release_date.format("%d-%m-%Y"),
            self.revision
        )
    }
}

//

/// Binding for values of [1.5.25 Setting the function of the digital inputs](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)
#[derive(Debug, PartialEq, Eq, Hash, Copy, Clone, FromPrimitive)]
pub enum DigitalInputFunction {
    UserDefined,
    StartRecordErrorReset,
    RecordSelectionBit0,
    RecordSelectionBit1,
    RecordSelectionBit2,
    RecordSelectionBit3,
    RecordSelectionBit4,
    ExternalReferenceSwitch,
    Trigger,
    Direction,
    Enable,
    Clock,
    ClockDirectionModeSelection1,
    ClockDirectionModeSelection2,
}

impl DigitalInputFunction {
    pub(crate) fn parse(s: &[u8]) -> IResult<&[u8], Self, ParseError<&[u8]>> {
        parse_enum_value(s, parse_su8, Self::from_u8)
    }
}

impl Display for DigitalInputFunction {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", *self as u8)
    }
}

//

/// Binding for values of [1.5.26 Setting the function of the digital outputs](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)
#[derive(Debug, PartialEq, Eq, Hash, Copy, Clone, FromPrimitive)]
pub enum DigitalOutputFunction {
    UserDefined,
    Ready,
    Running,
    Error,
}

impl DigitalOutputFunction {
    pub(crate) fn parse(s: &[u8]) -> IResult<&[u8], Self, ParseError<&[u8]>> {
        parse_enum_value(s, parse_su8, Self::from_u8)
    }
}

impl Display for DigitalOutputFunction {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", *self as u8)
    }
}

//

/// Binding for values of [1.5.36 Setting the ramp type](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)
#[derive(Debug, PartialEq, Eq, Hash, Copy, Clone, FromPrimitive)]
pub enum RampType {
    Trapizoidal,
    Sinus,
    JerkFree,
}

impl RampType {
    pub(crate) fn parse(s: &[u8]) -> IResult<&[u8], Self, ParseError<&[u8]>> {
        parse_enum_value(s, parse_su8, RampType::from_u8)
    }
}

impl Display for RampType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", *self as u8)
    }
}

//

/// Binding for values of [1.5.40 Setting baud rate of the controller](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)
///
/// Be aware that the numerical values of the enum **don't** correspond to the values
/// of the baud rate.
#[derive(Debug, PartialEq, Eq, Hash, Copy, Clone, FromPrimitive)]
pub enum BaudRate {
    B110 = 1,
    B300,
    B600,
    B1200,
    B2400,
    B4800,
    B9600,
    B14400,
    B19200,
    B38400,
    B57600,
    B115200,
}

impl BaudRate {
    pub(crate) fn parse(s: &[u8]) -> IResult<&[u8], Self, ParseError<&[u8]>> {
        parse_enum_value(s, parse_su8, BaudRate::from_u8)
    }
}

impl Display for BaudRate {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", *self as u8)
    }
}

//

/// Binding for values of [1.6.2 Stopping a motor](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)
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

//

/// Binding for values of [1.6.4 Reading out the current record](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)
#[derive(Debug, PartialEq, Eq, Hash, Copy, Clone, FromPrimitive)]
pub enum RespondMode {
    Quiet,
    NotQuiet,
}

impl RespondMode {
    pub(crate) fn parse<'b>(s: &'b [u8]) -> IResult<&'b [u8], Self, ParseError<&[u8]>> {
        parse_enum_value(s, parse_su8, RespondMode::from_u8)
    }

    pub fn is_quiet(&self) -> bool {
        *self == RespondMode::Quiet
    }

    pub fn is_responding(&self) -> bool {
        *self == RespondMode::NotQuiet
    }
}

impl Display for RespondMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", *self as u8)
    }
}

//

/// Binding for values of [1.6.6 Setting the positioning mode](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)
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
    pub(crate) fn parse(s: &[u8]) -> IResult<&[u8], Self, ParseError<&[u8]>> {
        parse_enum_value(s, parse_su8, PositioningMode::from_u8)
    }
}

impl Display for PositioningMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", *self as u8)
    }
}

//

/// Binding for values of [1.6.15 Setting the direction of rotation](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)
#[derive(Debug, PartialEq, Eq, Hash, Copy, Clone, FromPrimitive)]
pub enum RotationDirection {
    Left,
    Right,
}

impl RotationDirection {
    pub(crate) fn parse(s: &[u8]) -> IResult<&[u8], Self, ParseError<&[u8]>> {
        parse_enum_value(s, parse_su8, RotationDirection::from_u8)
    }
}

impl Display for RotationDirection {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", *self as u8)
    }
}

//

/// Holds a Record
///
/// See also [1.6.4 Reading out the current record](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)
#[derive(Debug, PartialEq, Eq, Hash, Clone)]
pub struct Record {
    pub positioning_mode: PositioningMode,
    pub travel_distance: i32,
    pub min_frequency: u32,
    pub max_frequency: u32,
    pub max_frequency2: u32,
    pub accel_ramp: u16,
    pub brake_ramp: u16,
    pub rotation_direction: RotationDirection,
    pub rotation_direction_change: bool,
    pub repetitions: u32,
    pub record_pause: u16,
    pub continuation_record: u8,
    pub max_accel_jerk: u32,
    pub max_brake_jerk: u32,
}

impl Record {
    pub(crate) fn parse(s: &[u8]) -> IResult<&[u8], Self, ParseError<&[u8]>> {
        use nom::{
            bytes::complete::tag,
            sequence::{preceded, tuple},
        };
        tuple((
            preceded(tag(map::POSITIONING_MODE), PositioningMode::parse),
            preceded(tag(map::TRAVEL_DISTANCE), parse_i32),
            preceded(tag(map::MIN_FREQUENCY), parse_su32),
            preceded(tag(map::MAX_FREQUENCY), parse_su32),
            preceded(tag(map::MAX_FREQUENCY2), parse_su32),
            preceded(tag(map::ACCEL_RAMP), parse_su16),
            preceded(tag(map::BRAKE_RAMP), parse_su16),
            preceded(tag(map::ROTATION_DIRECTION), RotationDirection::parse),
            preceded(tag(map::ROTATION_DIRECTION_CHANGE), parse_su8).map(|n| n == 1),
            preceded(tag(map::REPETITIONS), parse_su32),
            preceded(tag(map::RECORD_PAUSE), parse_su16),
            preceded(tag(map::CONTINUATION_RECORD), parse_su8),
            preceded(tag(map::MAX_ACCEL_JERK), parse_su32),
            preceded(tag(map::MAX_BRAKE_JERK), parse_su32),
        ))
        .map(
            |(
                positioning_mode,
                travel_distance,
                min_frequency,
                max_frequency,
                max_frequency2,
                accel_ramp,
                brake_ramp,
                rotation_direction,
                rotation_direction_change,
                repetitions,
                record_pause,
                continuation_record,
                max_accel_jerk,
                max_brake_jerk,
            )| Self {
                positioning_mode,
                travel_distance,
                min_frequency,
                max_frequency,
                max_frequency2,
                accel_ramp,
                brake_ramp,
                rotation_direction,
                rotation_direction_change,
                repetitions,
                record_pause,
                continuation_record,
                max_accel_jerk,
                max_brake_jerk,
            },
        )
        .parse(s)
    }
}
