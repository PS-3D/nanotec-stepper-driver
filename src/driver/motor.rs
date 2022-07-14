use super::{
    cmd::{
        BaudRate, DigitalInputFunction, DigitalOutputFunction, ErrorCorrectionMode,
        FirmwareVersion, LimitSwitchBehavior, MotorError, MotorStop, MotorType, PositioningMode,
        RampType, Record, RespondMode, RotationDirection, StepMode,
    },
    map,
    parse::{parse_su16, parse_su32, parse_su64, parse_su8},
    responsehandle::{
        DummyResponseHandle, ReadResponseHandle, ResponseHandle, WrapperResponseHandle,
        WriteResponseHandle,
    },
    DriverError, InnerDriver,
};
use crate::util::ensure;
use chrono::{DateTime, Duration, Local};
use nom::{
    bytes::complete::tag,
    character::complete::{i32 as parse_i32, i64 as parse_i64},
    sequence::{preceded, tuple},
    Finish, Parser,
};
use std::{
    cell::RefCell,
    io::{Read, Write},
    rc::Rc,
};

type DResult<T> = Result<T, DriverError>;
/// temporary alias, should be changed in the future
pub type AllMotor<I> = Motor<I>;

// sends read command to motor
// in theory a read command isn't diffrent than a write command, the
// difference is in the returned responsehandle.
//
// self is the Motor the macro was called in.
// parser is a parser (nom::Parser) that recognises the the response of the sent
// command and extracts a value.
// args are the format_arguments for the payload to send.
//
// panics if this motor is the all motor i.e. if self.addres == None
// panics if the respondmode is NotQuiet
//
// an invocation of this macro could look like this:
// read!(self, preceded(tag("Zs"), parse_i32), format_args!("Zs"))
//
// usually it is invoked by the short_read or long_read macros though
macro_rules! read {
    ($self:expr, $parser:expr, $args:expr) => {{
        // a read command to all motors would be stupid, you wouldn't be
        // able to distinguish the answers
        let address = $self
            .address
            .expect("Can't send a read command to all motors");
        let mut driver = $self.driver.as_ref().borrow_mut();
        assert_eq!(driver.get_respond_mode(address), RespondMode::NotQuiet);
        // send command
        driver.send_single_with_response(address, $args)?;
        Ok(ReadResponseHandle::new(
            $self.driver.clone(),
            address,
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
            format_args!("Z{}", $mnemonic)
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
        let rm = match $self.address {
            Some(a) => {
                let mut driver = $self.driver.as_ref().borrow_mut();
                let rm = driver.get_respond_mode(a);
                if rm == RespondMode::NotQuiet {
                    driver.send_single_with_response(a, $args)?;
                } else {
                    driver.send_single_no_response(a, $args)?;
                }
                rm
            }
            None => $self.driver.as_ref().borrow_mut().send_all($args)?,
        };
        if rm == RespondMode::NotQuiet {
            // unfortunately there isn't a better way rn.
            // value chosen sorta random, 64 bytes should be enough for nearly all
            // commands tho
            let mut sent = Vec::with_capacity(64);
            sent.write_fmt($args)?;
            Ok(WrapperResponseHandle::Write(WriteResponseHandle::new(
                $self.driver.clone(),
                $self.address,
                sent,
            )))
        } else {
            Ok(WrapperResponseHandle::Dummy(DummyResponseHandle::new()))
        }
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

/// Controls a single motor or all motors at once
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
/// To send commands to a motor in quick succession, the [`RespondMode`] can
/// be set to [`Quiet`][RespondMode::Quiet] with [`set_respond_mode`][Motor::set_respond_mode].
/// In this case the motor doesn't respond to commands. Be aware though that
/// getters will panic. See also [1.6.4 Reading out the current record](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf#%5B%7B%22num%22%3A123%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C113%2C742%2Cnull%5D)
///
/// # Errors
/// If a value doesn't match the specifications of the corresponding command
/// in the manual, [`DriverError::InvalidArgument`] is returned. If the given motor
/// was already waiting for a response, [`DriverError::NotAvailable`] is returned.
/// A [`DriverError::NotAvailable`] is also returned if a command is sent to all
/// motors while some are still waiting for a response. Or if a command is sent
/// to a single motors while not all motors have responded to a command for all
/// motors yet, since it isn't possible to distinguish the responses from single
/// motors in that case.
/// A [`DriverError::IoError`] is also possible, if there was an error sending the
/// command.
///
/// # Panics
/// If this is the all-motor, i.e. the motor used to send commands to all motors,
/// and a getter is called, the getter panics since it isn't possible to distinguish
/// the answers from the motors.\
/// Also panics if a getter is called and the [`RespondMode`] is
/// [`RespondMode::NotQuiet`] since it's impossible to get a value when there won't
/// be a response
/// (see also [1.6.4 Reading out the current record](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf#%5B%7B%22num%22%3A123%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C113%2C742%2Cnull%5D)).
///
/// # Examples
/// ```no_run
/// # use nanotec_stepper_driver::{Driver, ResponseHandle, RespondMode};
/// use std::time::Duration;
/// use serialport;
///
/// let s = serialport::new("/dev/ttyUSB0", 115200)
///     .timeout(Duration::from_secs(1))
///     .open()
///     .unwrap();
/// let mut driver = Driver::new(s);
/// let mut m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
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
/// # use nanotec_stepper_driver::{Driver, ResponseHandle, RespondMode};
/// use std::time::Duration;
/// use serialport;
///
/// let s = serialport::new("/dev/ttyUSB0", 115200)
///     .timeout(Duration::from_secs(1))
///     .open()
///     .unwrap();
/// let mut driver = Driver::new(s);
/// let mut m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
/// let mut m2 = driver.add_motor(2, RespondMode::NotQuiet).unwrap();
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
    address: Option<u8>,
}

// DResult<impl ResponseHandle<T>> is not an alias since aliases with
// impl aren't supported yet
impl<I: Write + Read> Motor<I> {
    pub(super) fn new(driver: Rc<RefCell<InnerDriver<I>>>, address: Option<u8>) -> Self {
        Motor { driver, address }
    }

    pub fn get_motor_type(&mut self) -> DResult<impl ResponseHandle<MotorType>> {
        long_read!(self, map::MOTOR_TYPE, MotorType::parse)
    }

    pub fn set_motor_type(&mut self, t: MotorType) -> DResult<impl ResponseHandle<()>> {
        long_write!(self, map::MOTOR_TYPE, t)
    }

    pub fn get_phase_current(&mut self) -> DResult<impl ResponseHandle<u8>> {
        short_read!(self, map::PHASE_CURRENT, parse_su8)
    }

    pub fn set_phase_current(&mut self, c: u8) -> DResult<impl ResponseHandle<()>> {
        ensure!(c <= 150, DriverError::InvalidArgument);
        short_write!(self, map::PHASE_CURRENT, c)
    }

    pub fn get_standstill_phase_current(&mut self) -> DResult<impl ResponseHandle<u8>> {
        short_read!(self, map::STANDSTILL_PHASE_CURRENT, parse_su8)
    }

    pub fn set_standstill_phase_current(&mut self, c: u8) -> DResult<impl ResponseHandle<()>> {
        ensure!(c <= 150, DriverError::InvalidArgument);
        short_write!(self, map::STANDSTILL_PHASE_CURRENT, c)
    }

    pub fn get_bldc_peak_current(&mut self) -> DResult<impl ResponseHandle<u8>> {
        long_read!(self, map::BLDC_PEAK_CURRENT, parse_su8)
    }

    pub fn set_bldc_peak_current(&mut self, c: u8) -> DResult<impl ResponseHandle<()>> {
        ensure!(c <= 150, DriverError::InvalidArgument);
        long_write!(self, map::BLDC_PEAK_CURRENT, c)
    }

    pub fn get_bldc_current_time_constant(&mut self) -> DResult<impl ResponseHandle<u16>> {
        long_read!(self, map::BLDC_CURRENT_TIME_CONSTANT, parse_su16)
    }

    pub fn set_bldc_current_time_constant(&mut self, t: u16) -> DResult<impl ResponseHandle<()>> {
        long_write!(self, map::BLDC_CURRENT_TIME_CONSTANT, t)
    }

    pub fn get_step_mode(&mut self) -> DResult<impl ResponseHandle<StepMode>> {
        short_read!(self, map::STEP_MODE, StepMode::parse)
    }

    pub fn set_step_mode(&mut self, m: StepMode) -> DResult<impl ResponseHandle<()>> {
        short_write!(self, map::STEP_MODE, m)
    }

    pub fn get_drive_address(&mut self) -> DResult<impl ResponseHandle<u8>> {
        short_read!(self, map::DRIVE_ADDRESS, parse_su8)
    }

    pub fn set_drive_address(&mut self, a: u8) -> DResult<impl ResponseHandle<()>> {
        ensure!(a >= 1 && a <= 254, DriverError::InvalidArgument);
        short_write!(self, map::DRIVE_ADDRESS, a)
    }

    pub fn get_motor_id(&mut self) -> DResult<impl ResponseHandle<u32>> {
        long_read!(self, map::MOTOR_ID, parse_su32)
    }

    pub fn set_motor_id(&mut self, id: u32) -> DResult<impl ResponseHandle<()>> {
        ensure!(id <= 2147483647, DriverError::InvalidArgument);
        long_write!(self, map::MOTOR_ID, id)
    }

    pub fn get_limit_switch_behavior(
        &mut self,
    ) -> DResult<impl ResponseHandle<LimitSwitchBehavior>> {
        short_read!(self, map::LIMIT_SWITCH_BEHAVIOR, LimitSwitchBehavior::parse)
    }

    pub fn set_limit_switch_behavior(
        &mut self,
        l: LimitSwitchBehavior,
    ) -> DResult<impl ResponseHandle<()>> {
        short_write!(self, map::LIMIT_SWITCH_BEHAVIOR, l)
    }

    pub fn get_error_correction_mode(
        &mut self,
    ) -> DResult<impl ResponseHandle<ErrorCorrectionMode>> {
        short_read!(self, map::ERROR_CORRECTION_MODE, ErrorCorrectionMode::parse)
    }

    pub fn set_error_correction_mode(
        &mut self,
        m: ErrorCorrectionMode,
    ) -> DResult<impl ResponseHandle<()>> {
        short_write!(self, map::ERROR_CORRECTION_MODE, m)
    }

    pub fn get_auto_correction_record(&mut self) -> DResult<impl ResponseHandle<u8>> {
        short_read!(self, map::AUTO_CORRECTION_RECORD, parse_su8)
    }

    pub fn set_auto_correction_record(&mut self, r: u8) -> DResult<impl ResponseHandle<()>> {
        ensure!(r <= 32, DriverError::InvalidArgument);
        short_write!(self, map::AUTO_CORRECTION_RECORD, r)
    }

    // TODO encoder direction

    pub fn get_swing_out_time(&mut self) -> DResult<impl ResponseHandle<u8>> {
        short_read!(self, map::SWING_OUT_TIME, parse_su8)
    }

    pub fn set_swing_out_time(&mut self, st: u8) -> DResult<impl ResponseHandle<()>> {
        ensure!(st <= 250, DriverError::InvalidArgument);
        short_write!(self, map::SWING_OUT_TIME, st)
    }

    pub fn get_max_encoder_deviation(&mut self) -> DResult<impl ResponseHandle<u8>> {
        short_read!(self, map::MAX_ENCODER_DEVIATION, parse_su8)
    }

    pub fn set_max_encoder_deviation(&mut self, d: u8) -> DResult<impl ResponseHandle<()>> {
        ensure!(d <= 250, DriverError::InvalidArgument);
        short_write!(self, map::MAX_ENCODER_DEVIATION, d)
    }

    pub fn get_feedrate_numerator(&mut self) -> DResult<impl ResponseHandle<u32>> {
        long_read!(self, map::FEED_RATE_NUMERATOR, parse_su32)
    }

    pub fn set_feedrate_numerator(&mut self, n: u32) -> DResult<impl ResponseHandle<()>> {
        ensure!(n <= 2147483647, DriverError::InvalidArgument);
        long_write!(self, map::FEED_RATE_NUMERATOR, n)
    }

    pub fn get_feedrate_denominator(&mut self) -> DResult<impl ResponseHandle<u32>> {
        long_read!(self, map::FEED_RATE_DENOMINATOR, parse_su32)
    }

    pub fn set_feedrate_denominator(&mut self, d: u32) -> DResult<impl ResponseHandle<()>> {
        ensure!(d <= 2147483647, DriverError::InvalidArgument);
        long_write!(self, map::FEED_RATE_DENOMINATOR, d)
    }

    pub fn reset_position_error(&mut self, p: i32) -> DResult<impl ResponseHandle<()>> {
        ensure!(
            p >= -100_000_000 && p <= 100_000_000,
            DriverError::InvalidArgument
        );
        short_write!(self, map::RESET_POS_ERR, p)
    }

    pub fn get_error(&mut self, p: u8) -> DResult<impl ResponseHandle<MotorError>> {
        ensure!(p <= 32, DriverError::InvalidArgument);
        // FIXME concrete value instead of just u8
        read!(
            self,
            preceded(
                tuple((tag(map::READ), parse_su8, tag(map::READ_ERR_MEM))),
                MotorError::parse
            ),
            format_args!("{}{}{}", map::READ, p, map::READ_ERR_MEM)
        )
    }

    pub fn get_encoder_position(&mut self) -> DResult<impl ResponseHandle<i32>> {
        short_read!(self, map::READ_ENCODER_POS, parse_i32)
    }

    pub fn get_position(&mut self) -> DResult<impl ResponseHandle<i32>> {
        short_read!(self, map::READ_POS, parse_i32)
    }

    pub fn is_motor_referenced(&mut self) -> DResult<impl ResponseHandle<bool>> {
        long_read!(self, map::IS_REFERENCED, parse_su8.map(|n| n == 1))
    }

    // TODO reading out status

    pub fn get_firmware_version(&mut self) -> DResult<impl ResponseHandle<FirmwareVersion>> {
        short_read!(self, map::READ_FIRMWARE_VERSION, FirmwareVersion::parse)
    }

    pub fn get_operating_time(&mut self) -> DResult<impl ResponseHandle<u64>> {
        long_read!(self, map::READ_OPERATING_TIME, parse_su64)
    }

    /// This command is not in the manual. It just gives you the time the motor
    /// operation started in the current time zone, similar to [`get_operating_time`][Motor::get_operating_time]
    pub fn get_operation_start(&mut self) -> DResult<impl ResponseHandle<DateTime<Local>>> {
        long_read!(
            self,
            map::READ_OPERATING_TIME,
            parse_i64.map(|s| {
                let now = Local::now();
                now - Duration::seconds(s)
            })
        )
    }

    pub fn get_digital_input_function(
        &mut self,
        i: u8,
    ) -> DResult<impl ResponseHandle<DigitalInputFunction>> {
        ensure!(i <= 8 && i >= 1, DriverError::InvalidArgument);
        let mut name = map::DIGITAL_INPUT_FUNCTION_PARTIAL.to_string();
        name.push(char::from_u32(60 + (i as u32)).unwrap());
        long_read!(self, name.as_str(), DigitalInputFunction::parse)
    }

    pub fn set_digital_input_function(
        &mut self,
        i: u8,
        f: DigitalInputFunction,
    ) -> DResult<impl ResponseHandle<()>> {
        ensure!(i <= 8 && i >= 1, DriverError::InvalidArgument);
        let mut name = map::DIGITAL_INPUT_FUNCTION_PARTIAL.to_string();
        name.push(char::from_u32(60 + (i as u32)).unwrap());
        long_write!(self, name.as_str(), f)
    }

    pub fn get_digital_output_function(
        &mut self,
        o: u8,
    ) -> DResult<impl ResponseHandle<DigitalOutputFunction>> {
        ensure!(o <= 8 && o >= 1, DriverError::InvalidArgument);
        let mut name = map::DIGITAL_OUTPUT_FUNCTION_PARTIAL.to_string();
        name.push(char::from_u32(60 + (o as u32)).unwrap());
        long_read!(self, name.as_str(), DigitalOutputFunction::parse)
    }

    pub fn set_digital_output_function(
        &mut self,
        o: u8,
        f: DigitalOutputFunction,
    ) -> DResult<impl ResponseHandle<()>> {
        ensure!(o <= 8 && o >= 1, DriverError::InvalidArgument);
        let mut name = map::DIGITAL_OUTPUT_FUNCTION_PARTIAL.to_string();
        name.push(char::from_u32(60 + (o as u32)).unwrap());
        long_write!(self, name.as_str(), f)
    }

    // Not implementing Masking and demasking inputs since it is deprecated

    pub fn get_reverse_in_out_polarity(&mut self) -> DResult<impl ResponseHandle<u32>> {
        short_read!(self, map::REVERSE_IN_OUT_POLARITY, parse_su32)
    }

    pub fn set_reverse_in_out_polarity(&mut self, b: u32) -> DResult<impl ResponseHandle<()>> {
        ensure!(b & 0x1ff00ff == b, DriverError::InvalidArgument);
        short_write!(self, map::REVERSE_IN_OUT_POLARITY, b)
    }

    pub fn get_input_debounce_time(&mut self) -> DResult<impl ResponseHandle<u8>> {
        short_read!(self, map::INPUT_DEBOUNCE_TIME, parse_su8)
    }

    pub fn set_input_debounce_time(&mut self, t: u8) -> DResult<impl ResponseHandle<()>> {
        ensure!(t <= 250, DriverError::InvalidArgument);
        short_write!(self, map::INPUT_DEBOUNCE_TIME, t)
    }

    // TODO set outputs
    // TODO read eeprom byte

    pub fn reset_eeprom(&mut self) -> DResult<impl ResponseHandle<()>> {
        short_write!(self, map::RESET_EEPROM, "")
    }

    // TODO automatic satus sending
    // TODO start bootloader

    pub fn get_reverse_clearance(&mut self) -> DResult<impl ResponseHandle<u16>> {
        short_read!(self, map::REVERSE_CLEARANCE, parse_su16)
    }

    pub fn set_reverse_clearance(&mut self, c: u16) -> DResult<impl ResponseHandle<()>> {
        ensure!(c <= 9999, DriverError::InvalidArgument);
        short_write!(self, map::REVERSE_CLEARANCE, c)
    }

    pub fn get_ramp_type(&mut self) -> DResult<impl ResponseHandle<RampType>> {
        long_read!(self, map::RAMP_TYPE, RampType::parse)
    }

    pub fn set_ramp_type(&mut self, t: RampType) -> DResult<impl ResponseHandle<()>> {
        long_write!(self, map::RAMP_TYPE, t)
    }

    pub fn get_brake_voltage_off_wait_time(&mut self) -> DResult<impl ResponseHandle<u16>> {
        long_read!(self, map::WAIT_TIME_BRAKE_VOLTAGE_OFF, parse_su16)
    }

    pub fn set_brake_voltage_off_wait_time(&mut self, t: u16) -> DResult<impl ResponseHandle<()>> {
        long_write!(self, map::WAIT_TIME_BRAKE_VOLTAGE_OFF, t)
    }

    pub fn get_motor_movement_wait_time(&mut self) -> DResult<impl ResponseHandle<u16>> {
        long_read!(self, map::WAIT_TIME_MOTOR_MOVE, parse_su16)
    }

    pub fn set_motor_movement_wait_time(&mut self, t: u16) -> DResult<impl ResponseHandle<()>> {
        long_write!(self, map::WAIT_TIME_MOTOR_MOVE, t)
    }

    pub fn get_motor_current_off_wait_time(&mut self) -> DResult<impl ResponseHandle<u16>> {
        long_read!(self, map::WAIT_TIME_MOTOR_CURRENT_OFF, parse_su16)
    }

    pub fn set_motor_current_off_wait_time(&mut self, t: u16) -> DResult<impl ResponseHandle<()>> {
        long_write!(self, map::WAIT_TIME_MOTOR_CURRENT_OFF, t)
    }

    pub fn get_baud_rate(&mut self) -> DResult<impl ResponseHandle<BaudRate>> {
        long_read!(self, map::BAUD_RATE, BaudRate::parse)
    }

    pub fn set_baud_rate(&mut self, br: BaudRate) -> DResult<impl ResponseHandle<()>> {
        long_write!(self, map::BAUD_RATE, br)
    }

    // TODO set crc checksum
    // TODO set hall config

    pub fn get_temp_raw(&mut self) -> DResult<impl ResponseHandle<u16>> {
        short_read!(self, map::READ_TEMP, parse_su16)
    }

    pub fn get_temp(&mut self) -> DResult<impl ResponseHandle<f64>> {
        short_read!(
            self,
            map::READ_TEMP,
            parse_su16.map(|r| (1_266_500f64
                / (4250f64
                    + ((0.33f64 * (((r as f64) / 1023f64) / (1f64 - ((r as f64) / 1023f64))))
                        .log10()
                        * 298f64)))
                - 273f64)
        )
    }

    pub fn get_quickstop_ramp(&mut self) -> DResult<impl ResponseHandle<u16>> {
        short_read!(self, map::QUICKSTOP_RAMP, parse_su16)
    }

    pub fn set_quickstop_ramp(&mut self, s: u16) -> DResult<impl ResponseHandle<()>> {
        ensure!(s <= 8000, DriverError::InvalidArgument);
        short_write!(self, map::QUICKSTOP_RAMP, s)
    }

    pub fn get_quickstop_ramp_no_conversion(&mut self) -> DResult<impl ResponseHandle<u32>> {
        long_read!(self, map::QUICKSTOP_RAMP_NO_CONVERSION, parse_su32)
    }

    pub fn set_quickstop_ramp_no_conversion(&mut self, r: u32) -> DResult<impl ResponseHandle<()>> {
        ensure!(r <= 3_000_000, DriverError::InvalidArgument);
        long_write!(self, map::QUICKSTOP_RAMP_NO_CONVERSION, r)
    }

    pub fn get_gearfactor_numerator(&mut self) -> DResult<impl ResponseHandle<u8>> {
        long_read!(self, map::GEAR_FACTOR_NUMERATOR, parse_su8)
    }

    pub fn set_gearfactor_numerator(&mut self, n: u8) -> DResult<impl ResponseHandle<()>> {
        long_write!(self, map::GEAR_FACTOR_NUMERATOR, n)
    }

    pub fn get_gearfactor_denominator(&mut self) -> DResult<impl ResponseHandle<u8>> {
        long_read!(self, map::GEAR_FACTOR_DENOMINATOR, parse_su8)
    }

    pub fn set_gearfactor_denominator(&mut self, d: u8) -> DResult<impl ResponseHandle<()>> {
        long_write!(self, map::GEAR_FACTOR_DENOMINATOR, d)
    }

    pub fn start_motor(&mut self) -> DResult<impl ResponseHandle<()>> {
        short_write!(self, map::START_MOTOR, "")
    }

    pub fn stop_motor(&mut self, stop: MotorStop) -> DResult<impl ResponseHandle<()>> {
        short_write!(self, map::STOP_MOTOR, stop)
    }

    pub fn load_record(&mut self, n: u8) -> DResult<impl ResponseHandle<()>> {
        ensure!(n >= 1 && n <= 32, DriverError::InvalidArgument);
        short_write!(self, map::LOAD_RECORD, n)
    }

    /// This method doesn't return a ResponseHandle since it doesn't actually send
    /// a command from the motor but rather just returns a value stored within
    /// this struct. See also [`set_respond_mode`][Motor::set_respond_mode]
    pub fn get_respond_mode(&mut self) -> RespondMode {
        self.driver
            .as_ref()
            .borrow()
            .get_respond_mode(self.address.expect("Can only read from one motor"))
    }

    /// See [`set_respond_mode`][Motor::set_respond_mode]
    pub fn get_current_record(&mut self) -> DResult<impl ResponseHandle<Record>> {
        read!(
            self,
            preceded(tag(map::READ), Record::parse),
            format_args!("{}{}", map::READ, map::READ_CURRENT_RECORD)
        )
    }

    /// See [`set_respond_mode`][Motor::set_respond_mode]
    pub fn get_record(&mut self, n: u8) -> DResult<impl ResponseHandle<Record>> {
        ensure!(n <= 32, DriverError::InvalidArgument);
        read!(
            self,
            // FIXME concrete value instead of just parse_su8
            preceded(tuple((tag(map::READ), parse_su8)), Record::parse),
            format_args!("{}{}{}", map::READ, n, map::READ_CURRENT_RECORD)
        )
    }

    /// This command doesn't exist in the manual.
    /// [1.6.4 Reading out the current record](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf#%5B%7B%22num%22%3A123%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C113%2C742%2Cnull%5D)
    /// was so convoluted that it was split into multiple functions, namely this
    /// one, [`get_current_record`][Motor::get_current_record],
    /// [`get_record`][Motor::get_record] and [`get_respond_mode`][Motor::get_respond_mode].
    /// This function is responsible for setting whether or not the firmware
    /// responds to most commands and the other 2 are responsible for actually
    /// reading out records.
    pub fn set_respond_mode(&mut self, mode: RespondMode) -> DResult<impl ResponseHandle<()>> {
        if let Some(a) = self.address {
            self.driver.as_ref().borrow_mut().set_respond_mode(a, mode);
        } else {
            self.driver.as_ref().borrow_mut().set_respond_mode_all(mode);
        }
        short_write!(self, map::READ_CURRENT_RECORD, mode)
    }

    pub fn save_record(&mut self, n: u8) -> DResult<impl ResponseHandle<()>> {
        ensure!(n >= 1 && n <= 32, DriverError::InvalidArgument);
        short_write!(self, map::SAVE_RECORD, n)
    }

    pub fn get_positioning_mode(&mut self) -> DResult<impl ResponseHandle<PositioningMode>> {
        short_read!(self, map::POSITIONING_MODE, PositioningMode::parse)
    }

    pub fn set_positioning_mode(
        &mut self,
        mode: PositioningMode,
    ) -> DResult<impl ResponseHandle<()>> {
        short_write!(self, map::POSITIONING_MODE, mode)
    }

    pub fn get_travel_distance(&mut self) -> DResult<impl ResponseHandle<i32>> {
        short_read!(self, map::TRAVEL_DISTANCE, parse_i32)
    }

    pub fn set_travel_distance(&mut self, distance: i32) -> DResult<impl ResponseHandle<()>> {
        ensure!(
            distance >= -100_000_000 && distance <= 100_000_000,
            DriverError::InvalidArgument
        );
        short_write!(self, map::TRAVEL_DISTANCE, distance)
    }

    pub fn get_min_frequency(&mut self) -> DResult<impl ResponseHandle<u32>> {
        short_read!(self, map::MIN_FREQUENCY, parse_su32)
    }

    pub fn set_min_frequency(&mut self, frequency: u32) -> DResult<impl ResponseHandle<()>> {
        ensure!(
            frequency >= 1 && frequency <= 160_000,
            DriverError::InvalidArgument
        );
        short_write!(self, map::MIN_FREQUENCY, frequency)
    }

    pub fn get_max_frequency(&mut self) -> DResult<impl ResponseHandle<u32>> {
        short_read!(self, map::MAX_FREQUENCY, parse_su32)
    }

    pub fn set_max_frequency(&mut self, frequency: u32) -> DResult<impl ResponseHandle<()>> {
        ensure!(
            frequency >= 1 && frequency <= 1_000_000,
            DriverError::InvalidArgument
        );
        short_write!(self, map::MAX_FREQUENCY, frequency)
    }

    pub fn get_max_frequency2(&mut self) -> DResult<impl ResponseHandle<u32>> {
        short_read!(self, map::MAX_FREQUENCY2, parse_su32)
    }

    pub fn set_max_frequency2(&mut self, frequency: u32) -> DResult<impl ResponseHandle<()>> {
        ensure!(
            frequency >= 1 && frequency <= 1_000_000,
            DriverError::InvalidArgument
        );
        short_write!(self, map::MAX_FREQUENCY2, frequency)
    }

    pub fn get_accel_ramp(&mut self) -> DResult<impl ResponseHandle<u16>> {
        short_read!(self, map::ACCEL_RAMP, parse_su16)
    }

    pub fn set_accel_ramp(&mut self, n: u16) -> DResult<impl ResponseHandle<()>> {
        ensure!(n >= 1, DriverError::InvalidArgument);
        short_write!(self, map::ACCEL_RAMP, n)
    }

    pub fn get_accel_ramp_no_conversion(&mut self) -> DResult<impl ResponseHandle<u32>> {
        long_read!(self, map::ACCEL_RAMP_NO_CONVERSION, parse_su32)
    }

    pub fn set_accel_ramp_no_conversion(&mut self, n: u32) -> DResult<impl ResponseHandle<()>> {
        ensure!(n >= 1 && n <= 3_000_000, DriverError::InvalidArgument);
        long_write!(self, map::ACCEL_RAMP_NO_CONVERSION, n)
    }

    pub fn get_brake_ramp(&mut self) -> DResult<impl ResponseHandle<u16>> {
        short_read!(self, map::BRAKE_RAMP, parse_su16)
    }

    pub fn set_brake_ramp(&mut self, n: u16) -> DResult<impl ResponseHandle<()>> {
        short_write!(self, map::BRAKE_RAMP, n)
    }

    pub fn get_brake_ramp_no_conversion(&mut self) -> DResult<impl ResponseHandle<u32>> {
        long_read!(self, map::BRAKE_RAMP_NO_CONVERSION, parse_su32)
    }

    pub fn set_brake_ramp_no_conversion(&mut self, n: u32) -> DResult<impl ResponseHandle<()>> {
        ensure!(n <= 3_000_000, DriverError::InvalidArgument);
        long_write!(self, map::BRAKE_RAMP_NO_CONVERSION, n)
    }

    pub fn get_rotation_direction(&mut self) -> DResult<impl ResponseHandle<RotationDirection>> {
        short_read!(self, map::ROTATION_DIRECTION, RotationDirection::parse)
    }

    pub fn set_rotation_direction(
        &mut self,
        direction: RotationDirection,
    ) -> DResult<impl ResponseHandle<()>> {
        short_write!(self, map::ROTATION_DIRECTION, direction)
    }

    pub fn get_rotation_direction_change(&mut self) -> DResult<impl ResponseHandle<bool>> {
        short_read!(
            self,
            map::ROTATION_DIRECTION_CHANGE,
            parse_su8.map(|n| n == 1)
        )
    }

    pub fn set_rotation_direction_change(
        &mut self,
        change: bool,
    ) -> DResult<impl ResponseHandle<()>> {
        short_write!(self, map::ROTATION_DIRECTION_CHANGE, change as u8)
    }

    // TODO maybe work out better type since 0 means endless
    pub fn get_repetitions(&mut self) -> DResult<impl ResponseHandle<u32>> {
        short_read!(self, map::REPETITIONS, parse_su32)
    }

    pub fn set_repetitions(&mut self, n: u32) -> DResult<impl ResponseHandle<()>> {
        ensure!(n <= 254, DriverError::InvalidArgument);
        short_write!(self, map::REPETITIONS, n)
    }

    pub fn get_record_pause(&mut self) -> DResult<impl ResponseHandle<u16>> {
        short_read!(self, map::RECORD_PAUSE, parse_su16)
    }

    pub fn set_record_pause(&mut self, n: u16) -> DResult<impl ResponseHandle<()>> {
        short_write!(self, map::RECORD_PAUSE, n)
    }

    pub fn get_continuation_record(&mut self) -> DResult<impl ResponseHandle<u8>> {
        short_read!(self, map::CONTINUATION_RECORD, parse_su8)
    }

    // TODO make option for no continuation record
    pub fn set_continuation_record(&mut self, n: u8) -> DResult<impl ResponseHandle<()>> {
        ensure!(n <= 32, DriverError::InvalidArgument);
        short_write!(self, map::CONTINUATION_RECORD, n)
    }

    pub fn get_max_accel_jerk(&mut self) -> DResult<impl ResponseHandle<u32>> {
        short_read!(self, map::MAX_ACCEL_JERK, parse_su32)
    }

    pub fn set_max_accel_jerk(&mut self, n: u32) -> DResult<impl ResponseHandle<()>> {
        ensure!(n >= 1 && n <= 100_000_000, DriverError::InvalidArgument);
        short_write!(self, map::MAX_ACCEL_JERK, n)
    }

    pub fn get_max_brake_jerk(&mut self) -> DResult<impl ResponseHandle<u32>> {
        short_read!(self, map::MAX_BRAKE_JERK, parse_su32)
    }

    pub fn set_max_brake_jerk(&mut self, n: u32) -> DResult<impl ResponseHandle<()>> {
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
        if let Some(a) = self.address {
            self.driver.as_ref().borrow_mut().drop_motor(&a)
        } else {
            self.driver.as_ref().borrow_mut().drop_all_motor()
        }
    }
}
