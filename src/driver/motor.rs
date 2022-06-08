mod map;

use super::{
    cmd::{MotorStop, PositioningMode, RespondMode, RotationDirection},
    responsehandle::{ReadResponseHandle, ResponseHandle, WriteResponseHandle},
    DriverError, InnerDriver,
};
use crate::util::ensure;
use nom::{
    character::complete::{i32 as parse_i32, u16 as parse_u16, u32 as parse_u32, u8 as parse_u8},
    Finish, Parser,
};
use std::{
    cell::RefCell,
    io::{Read, Write},
    rc::Rc,
};

macro_rules! read {
    ($self:expr, $parser:expr, $args:expr) => {{
        $self
            .driver
            .as_ref()
            .borrow_mut()
            .send_fmt(format_args!("#{}{}\r", $self.address, $args))?;
        Ok(ReadResponseHandle::new(
            $self.driver.clone(),
            $self.address,
            move |input| {
                let (remainder, t) =
                    $parser
                        .parse(input)
                        .finish()
                        .map_err(|_: nom::error::Error<&[u8]>| {
                            DriverError::NonMatchingPayloads(input.to_vec())
                        })?;
                if !remainder.is_empty() {
                    Err(DriverError::ParsingError(nom::error::Error {
                        input: remainder.to_vec(),
                        code: nom::error::ErrorKind::TooLarge,
                    }))
                } else {
                    Ok(t)
                }
            },
        ))
    }};
}

macro_rules! short_read {
    ($self:expr, $mnemonic:expr, $parser:expr) => {
        read!(
            $self,
            nom::sequence::preceded(
                nom::sequence::preceded(
                    nom::bytes::complete::tag(map::READ),
                    nom::bytes::complete::tag($mnemonic)
                ),
                $parser
            ),
            format_args!("{}", $mnemonic)
        )
    };
}

macro_rules! long_read {
    ($self:expr, $mnemonic:expr, $parser:expr) => {
        read!(
            $self,
            nom::sequence::preceded(nom::bytes::complete::tag($mnemonic), $parser),
            format_args!("{}", $mnemonic)
        )
    };
}

macro_rules! write {
    ($self:expr, $args:expr) => {{
        $self
            .driver
            .as_ref()
            .borrow_mut()
            .send_fmt(format_args!("#{}{}\r", $self.address, $args))?;
        let mut sent = Vec::with_capacity(64);
        sent.write_fmt($args)?;
        Ok(WriteResponseHandle::new(
            $self.driver.clone(),
            $self.address,
            sent,
        ))
    }};
}

macro_rules! short_write {
    ($self:expr, $mnemonic:expr, $data:expr) => {
        write!($self, format_args!("{}{}", $mnemonic, $data))
    };
}

macro_rules! long_write {
    ($self:expr, $mnemonic:expr, $data:expr) => {
        write!($self, format_args!("{}={}", $mnemonic, $data))
    };
}

/// This type is not Threadsafe
#[derive(Debug)]
pub struct Motor<I: Write + Read> {
    driver: Rc<RefCell<InnerDriver<I>>>,
    address: u8,
}

impl<I: Write + Read> Motor<I> {
    pub(super) fn new(driver: Rc<RefCell<InnerDriver<I>>>, address: u8) -> Self {
        Motor { driver, address }
    }

    pub fn start_motor(&mut self) -> Result<impl ResponseHandle<()>, DriverError> {
        short_write!(self, map::START_MOTOR, "")
    }

    pub fn stop_motor(&mut self, stop: MotorStop) -> Result<impl ResponseHandle<()>, DriverError> {
        short_write!(self, map::STOP_MOTOR, stop)
    }

    pub fn load_record(&mut self, n: u8) -> Result<impl ResponseHandle<()>, DriverError> {
        ensure!(n >= 1 && n <= 32, DriverError::InvalidArgument);
        short_write!(self, map::LOAD_RECORD, n)
    }

    pub fn get_respond_mode(&mut self) -> Result<impl ResponseHandle<RespondMode>, DriverError> {
        short_read!(self, map::READ_CURRENT_RECORD, RespondMode::parse)
    }

    // TODO get current record and get record

    pub fn set_respond_mode(
        &mut self,
        mode: RespondMode,
    ) -> Result<impl ResponseHandle<()>, DriverError> {
        short_write!(self, map::READ_CURRENT_RECORD, mode)
    }

    pub fn save_record(&mut self, n: u8) -> Result<impl ResponseHandle<()>, DriverError> {
        ensure!(n >= 1 && n <= 32, DriverError::InvalidArgument);
        short_write!(self, map::SAVE_RECORD, n)
    }

    pub fn get_positioning_mode(
        &mut self,
    ) -> Result<impl ResponseHandle<PositioningMode>, DriverError> {
        short_read!(self, map::POSITIONING_MODE, PositioningMode::parse)
    }

    pub fn set_positioning_mode(
        &mut self,
        mode: PositioningMode,
    ) -> Result<impl ResponseHandle<()>, DriverError> {
        short_write!(self, map::POSITIONING_MODE, mode)
    }

    pub fn get_travel_distance(&mut self) -> Result<impl ResponseHandle<i32>, DriverError> {
        short_read!(self, map::TRAVEL_DISTANCE, parse_i32)
    }

    pub fn set_travel_distance(
        &mut self,
        distance: i32,
    ) -> Result<impl ResponseHandle<()>, DriverError> {
        ensure!(
            distance >= -100_000_000 && distance <= 100_000_000,
            DriverError::InvalidArgument
        );
        short_write!(self, map::TRAVEL_DISTANCE, distance)
    }

    pub fn get_min_frequency(&mut self) -> Result<impl ResponseHandle<u32>, DriverError> {
        short_read!(self, map::MIN_FREQUENCY, parse_u32)
    }

    pub fn set_min_frequency(
        &mut self,
        frequency: u32,
    ) -> Result<impl ResponseHandle<()>, DriverError> {
        ensure!(
            frequency >= 1 && frequency <= 160_000,
            DriverError::InvalidArgument
        );
        short_write!(self, map::MIN_FREQUENCY, frequency)
    }

    pub fn get_max_frequency(&mut self) -> Result<impl ResponseHandle<u32>, DriverError> {
        short_read!(self, map::MAX_FREQUENCY, parse_u32)
    }

    pub fn set_max_frequency(
        &mut self,
        frequency: u32,
    ) -> Result<impl ResponseHandle<()>, DriverError> {
        ensure!(
            frequency >= 1 && frequency <= 1_000_000,
            DriverError::InvalidArgument
        );
        short_write!(self, map::MAX_FREQUENCY, frequency)
    }

    pub fn get_max_frequency2(&mut self) -> Result<impl ResponseHandle<u32>, DriverError> {
        short_read!(self, map::MAX_FREQUENCY2, parse_u32)
    }

    pub fn set_max_frequency2(
        &mut self,
        frequency: u32,
    ) -> Result<impl ResponseHandle<()>, DriverError> {
        ensure!(
            frequency >= 1 && frequency <= 1_000_000,
            DriverError::InvalidArgument
        );
        short_write!(self, map::MAX_FREQUENCY2, frequency)
    }

    pub fn get_accel_ramp(&mut self) -> Result<impl ResponseHandle<u16>, DriverError> {
        short_read!(self, map::ACCEL_RAMP, parse_u16)
    }

    pub fn set_accel_ramp(&mut self, n: u16) -> Result<impl ResponseHandle<()>, DriverError> {
        ensure!(n >= 1, DriverError::InvalidArgument);
        short_write!(self, map::ACCEL_RAMP, n)
    }

    pub fn get_accel_ramp_no_conversion(
        &mut self,
    ) -> Result<impl ResponseHandle<u32>, DriverError> {
        long_read!(self, map::ACCEL_RAMP_NO_CONVERSION, parse_u32)
    }

    pub fn set_accel_ramp_no_conversion(
        &mut self,
        n: u32,
    ) -> Result<impl ResponseHandle<()>, DriverError> {
        ensure!(n >= 1 && n <= 3_000_000, DriverError::InvalidArgument);
        long_write!(self, map::ACCEL_RAMP_NO_CONVERSION, n)
    }

    pub fn get_brake_ramp(&mut self) -> Result<impl ResponseHandle<u16>, DriverError> {
        short_read!(self, map::BRAKE_RAMP, parse_u16)
    }

    pub fn set_brake_ramp(&mut self, n: u16) -> Result<impl ResponseHandle<()>, DriverError> {
        short_write!(self, map::BRAKE_RAMP, n)
    }

    pub fn get_brake_ramp_no_conversion(
        &mut self,
    ) -> Result<impl ResponseHandle<u32>, DriverError> {
        long_read!(self, map::BRAKE_RAMP_NO_CONVERSION, parse_u32)
    }

    pub fn set_brake_ramp_no_conversion(
        &mut self,
        n: u32,
    ) -> Result<impl ResponseHandle<()>, DriverError> {
        ensure!(n <= 3_000_000, DriverError::InvalidArgument);
        long_write!(self, map::BRAKE_RAMP_NO_CONVERSION, n)
    }

    pub fn get_rotation_direction(
        &mut self,
    ) -> Result<impl ResponseHandle<RotationDirection>, DriverError> {
        short_read!(self, map::ROTATION_DIRECTION, RotationDirection::parse)
    }

    pub fn set_rotation_direction(
        &mut self,
        direction: RotationDirection,
    ) -> Result<impl ResponseHandle<()>, DriverError> {
        short_write!(self, map::ROTATION_DIRECTION, direction)
    }

    pub fn get_rotation_direction_change(
        &mut self,
    ) -> Result<impl ResponseHandle<bool>, DriverError> {
        short_read!(
            self,
            map::ROTATION_DIRECTION_CHANGE,
            parse_u8.map(|n| n == 1)
        )
    }

    pub fn set_rotation_direction_change(
        &mut self,
        change: bool,
    ) -> Result<impl ResponseHandle<()>, DriverError> {
        short_write!(self, map::ROTATION_DIRECTION_CHANGE, change)
    }

    pub fn get_repetitions(&mut self) -> Result<impl ResponseHandle<u32>, DriverError> {
        short_read!(self, map::REPETITIONS, parse_u32)
    }

    pub fn set_repetitions(&mut self, n: u32) -> Result<impl ResponseHandle<()>, DriverError> {
        ensure!(n <= 254, DriverError::InvalidArgument);
        short_write!(self, map::REPETITIONS, n)
    }

    pub fn get_record_pause(&mut self) -> Result<impl ResponseHandle<u16>, DriverError> {
        short_read!(self, map::RECORD_PAUSE, parse_u16)
    }

    pub fn set_record_pause(&mut self, n: u16) -> Result<impl ResponseHandle<()>, DriverError> {
        short_write!(self, map::RECORD_PAUSE, n)
    }

    pub fn get_continuation_record(&mut self) -> Result<impl ResponseHandle<u8>, DriverError> {
        short_read!(self, map::CONTINUATION_RECORD, parse_u8)
    }

    pub fn set_continuation_record(
        &mut self,
        n: u8,
    ) -> Result<impl ResponseHandle<()>, DriverError> {
        ensure!(n <= 32, DriverError::InvalidArgument);
        short_write!(self, map::CONTINUATION_RECORD, n)
    }

    pub fn get_max_accel_jerk(&mut self) -> Result<impl ResponseHandle<u32>, DriverError> {
        short_read!(self, map::MAX_ACCEL_JERK, parse_u32)
    }

    pub fn set_max_accel_jerk(&mut self, n: u32) -> Result<impl ResponseHandle<()>, DriverError> {
        ensure!(n >= 1 && n <= 100_000_000, DriverError::InvalidArgument);
        short_write!(self, map::MAX_ACCEL_JERK, n)
    }

    pub fn get_max_brake_jerk(&mut self) -> Result<impl ResponseHandle<u32>, DriverError> {
        short_read!(self, map::MAX_BRAKE_JERK, parse_u32)
    }

    pub fn set_max_brake_jerk(&mut self, n: u32) -> Result<impl ResponseHandle<()>, DriverError> {
        ensure!(n >= 1 && n <= 100_000_000, DriverError::InvalidArgument);
        short_write!(self, map::MAX_BRAKE_JERK, n)
    }
}

impl<I: Write + Read> Drop for Motor<I> {
    fn drop(&mut self) {
        self.driver.as_ref().borrow_mut().drop_motor(&self.address)
    }
}
