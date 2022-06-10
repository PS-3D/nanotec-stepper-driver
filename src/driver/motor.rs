use super::{
    cmd::{MotorStop, PositioningMode, Record, RespondMode, RotationDirection},
    map,
    responsehandle::{ReadResponseHandle, ResponseHandle, WriteResponseHandle},
    DriverError, InnerDriver,
};
use crate::util::ensure;
use nom::{
    bytes::complete::tag,
    character::complete::{i32 as parse_i32, u16 as parse_u16, u32 as parse_u32, u8 as parse_u8},
    sequence::{preceded, tuple},
    Finish, Parser,
};
use std::{
    cell::RefCell,
    io::{Read, Write},
    rc::Rc,
};

// sends read command to motor
// in theory a read command isn't diffrent than a write command, the
// difference is in the returned responsehandle.
//
// self is the Motor the macro was called in.
// parser is a parser (nom::Parser) that recognises the the response of the sent
// command and extracts a value.
// args are the format_arguments for the payload to send.
//
// an invocation of this macro could look like this:
// read!(self, preceded(tag("Zs"), parse_i32), format_args!("Zs"))
//
// usually it is invoked by the short_read or long_read macros though
macro_rules! read {
    ($self:expr, $parser:expr, $args:expr) => {{
        // send command
        $self
            .driver
            .as_ref()
            .borrow_mut()
            .send_fmt(format_args!("#{}{}\r", $self.address, $args))?;
        Ok(ReadResponseHandle::new(
            $self.driver.clone(),
            $self.address,
            // parsing closure
            move |input| {
                let (remainder, t) =
                    $parser
                        .parse(input)
                        .finish()
                        .map_err(|_: nom::error::Error<&[u8]>| {
                            DriverError::NonMatchingPayloads(input.to_vec())
                        })?;
                if !remainder.is_empty() {
                    // TODO make own errorkind, adjust doc for wait
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

// sends short read command to the motor
// in theory a read command isn't diffrent than a write command, the
// difference is in the returned responsehandle.
// This macro is only for read commands which have the form "Z<mnemonic>"
// and return a payload of the form "Z<mnemonic><value>". Commands of any other
// form should use read directly
//
// self is the Motor the macro was called in.
// mnemonic is the symbol(s) of the command, e.g. s
// parser is a parser (nom::Parser) that recognises the the response of the sent
// command and extracts a value.
//
// an invocation of this macro could look like this:
// short_read!(self, "s", parse_i32)
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

// sends long read command to the motor
// in theory a read command isn't diffrent than a write command, the
// difference is in the returned responsehandle.
// This macro is only for read commands which have the form "<mnemonic>"
// and return a payload of the form "<mnemonic><value>". Commands of any other
// form should use read directly
//
// self is the Motor the macro was called in.
// mnemonic is the symbol(s) of the command, e.g. s
// parser is a parser (nom::Parser) that recognises the the response of the sent
// command and extracts a value.
//
// an invocation of this macro could look like this:
// long_read!(self, ":CL_motor_type", MotorType::parse)
macro_rules! long_read {
    ($self:expr, $mnemonic:expr, $parser:expr) => {
        read!(
            $self,
            nom::sequence::preceded(nom::bytes::complete::tag($mnemonic), $parser),
            format_args!("{}", $mnemonic)
        )
    };
}

// sends write command to motor
// in theory a read command isn't diffrent than a write command, the
// difference is in the returned responsehandle.
//
// self is the Motor the macro was called in.
// args are the format_arguments for the payload to send.
//
// an invocation of this macro could look like this:
// write!(self, format_args!("A"))
//
// usually it is invoked by the short_write or long_write macros though
macro_rules! write {
    ($self:expr, $args:expr) => {{
        $self
            .driver
            .as_ref()
            .borrow_mut()
            .send_fmt(format_args!("#{}{}\r", $self.address, $args))?;
        // unfortunately there isn't a better way rn.
        // value chosen sorta random, 64 bytes should be enough for nearly all
        // commands tho
        let mut sent = Vec::with_capacity(64);
        sent.write_fmt($args)?;
        Ok(WriteResponseHandle::new(
            $self.driver.clone(),
            $self.address,
            sent,
        ))
    }};
}

// sends short write command to motor
// in theory a read command isn't diffrent than a write command, the
// difference is in the returned responsehandle.
//
// self is the Motor the macro was called in.
// mnemonic is the symbol of the command, e.g. "S"
// data is the data which should be written
//
// an invocation of this macro could look like this:
// short_write!(self, "S", MotorStop::QuickStop)
macro_rules! short_write {
    ($self:expr, $mnemonic:expr, $data:expr) => {
        write!($self, format_args!("{}{}", $mnemonic, $data))
    };
}

// sends long write command to motor
// in theory a read command isn't diffrent than a write command, the
// difference is in the returned responsehandle.
//
// self is the Motor the macro was called in.
// mnemonic is the symbol of the command, e.g. ":CL_motor_type"
// data is the data which should be written
//
// an invocation of this macro could look like this:
// long_write!(self, ":CL_motor_type", MotorType::Stepper)
macro_rules! long_write {
    ($self:expr, $mnemonic:expr, $data:expr) => {
        write!($self, format_args!("{}={}", $mnemonic, $data))
    };
}

// TODO return error if motor still waits for a command
/// Controls a single motor
///
/// This struct actually communicates with the motor. Basically all commands that
/// can be found in the manual are mapped to methods in this struct. Usually a
/// command is split into a getter and a setter. Only some like reading the current
/// record are split even more. Nearly all methods block until the command got
/// sent.
///
/// The methods usually don't return values directly. Instead they return a
/// [`ResponseHandle`], on which [`ResponseHandle::wait`] can be called to then
/// wait for the response of the motor and, depending on the command, obtain a
/// value. Only the handles of getters actually return a value, the others only
/// wait for the response and check it was correct. This approach was chosen so
/// commands can be sent to different motors before having to wait for the
/// responses. This reduces waittimes.
///
/// # Errors
/// If a value doesn't match the specifications of the corresponding command
/// in the manual, [`DriverError::InvalidArgument`] is returned. A
/// [`DriverError::IoError`] is also possible, if there was an error sending the
/// command.
///
/// # Examples
/// ```no_run
/// # use nanotec_stepper_driver::{Driver, ResponseHandle};
/// use std::time::Duration;
/// use serialport;
///
/// let s = serialport::new("/dev/ttyUSB0", 115200)
///     .timeout(Duration::from_secs(1))
///     .open()
///     .unwrap();
/// let driver = Driver::new(s);
/// let mut m1 = driver.add_motor(1).unwrap();
///
/// m1.load_record(3).unwrap().wait().unwrap();
/// m1.set_continuation_record(0).unwrap().wait().unwrap();
/// m1.start_motor().unwrap().wait().unwrap();
///
/// let steps = m1.get_travel_distance().unwrap().wait().unwrap();
/// let max_freq = m1.get_max_frequency().unwrap().wait().unwrap();
/// println!("Drove {} steps with a top speed of {} steps/min", steps, max_freq);
///
/// let record = m1.get_current_record().unwrap().wait().unwrap();
/// println!("Current record: {:?}", record);
/// ```
/// Or sending multiple commands more or less at the same time:
/// ```no_run
/// # use nanotec_stepper_driver::{Driver, ResponseHandle};
/// use std::time::Duration;
/// use serialport;
///
/// let s = serialport::new("/dev/ttyUSB0", 115200)
///     .timeout(Duration::from_secs(1))
///     .open()
///     .unwrap();
/// let driver = Driver::new(s);
/// let mut m1 = driver.add_motor(1).unwrap();
/// let mut m2 = driver.add_motor(2).unwrap();
///
/// let handle1 = m1.start_motor().unwrap();
/// let handle2 = m2.start_motor().unwrap();
/// println!("started motors");
/// handle1.wait().unwrap();
/// handle2.wait().unwrap();
/// println!("motors stopped");
/// ```
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

    // it's not possible to read the respondmode, but in the future it will
    // be stored in the motor struct, since it will be needed there anyways
    // then we can comment in this function again
    // pub fn get_respond_mode(&mut self) -> Result<impl ResponseHandle<RespondMode>, DriverError> {
    //     short_read!(self, map::READ_CURRENT_RECORD, RespondMode::parse)
    // }

    /// See [`set_respond_mode`][Motor::set_respond_mode]
    pub fn get_current_record(&mut self) -> Result<impl ResponseHandle<Record>, DriverError> {
        read!(
            self,
            preceded(
                tuple((tag(map::READ), tag(map::READ_CURRENT_RECORD))),
                Record::parse
            ),
            format_args!("{}{}", map::READ, map::READ_CURRENT_RECORD)
        )
    }

    /// See [`set_respond_mode`][Motor::set_respond_mode]
    pub fn get_record(&mut self, n: u8) -> Result<impl ResponseHandle<Record>, DriverError> {
        ensure!(n <= 32, DriverError::InvalidArgument);
        read!(
            self,
            preceded(
                tuple((tag(map::READ), parse_u8, tag(map::READ_CURRENT_RECORD))),
                Record::parse
            ),
            format_args!("{}{}{}", map::READ, n, map::READ_CURRENT_RECORD)
        )
    }

    /// This command doesn't exist in the manual.
    /// [1.6.4 Reading out the current record](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf#%5B%7B%22num%22%3A123%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C113%2C742%2Cnull%5D)
    /// was so convoluted that it was split into multiple functions, namely this
    /// one, [`get_current_record`][Motor::get_current_record] and
    /// [`get_record`][Motor::get_record].
    /// This function is responsible for setting whether or not the firmware
    /// responds to most commands and the other 2 are responsible for actually
    /// reading out records.
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
    /// Removes this motor from the driver.\
    /// Afterwards, a motor with this address can be added again by calling
    /// [`Driver::add_motor`][super::Driver::add_motor].
    ///
    /// See also [here][`Drop`]
    fn drop(&mut self) {
        self.driver.as_ref().borrow_mut().drop_motor(&self.address)
    }
}
