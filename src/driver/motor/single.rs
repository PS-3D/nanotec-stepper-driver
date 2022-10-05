use super::{
    super::{
        cmd::{
            frame::MotorAddress,
            payload::{
                BaudRate, DigitalInputFunction, DigitalOutputFunction, ErrorCorrectionMode,
                FirmwareVersion, LimitSwitchBehavior, MotorError, MotorStatus, MotorStop,
                MotorType, PositioningMode, RampType, Record, Repetitions, RespondMode,
                RotationDirection, StepMode,
            },
        },
        map,
        parse::{parse_su16, parse_su32, parse_su64, parse_su8, ParseError},
        responsehandle::{
            map::ResponseHandleMap,
            read::{ReadResponseHandle, StatusResponseHandle},
            write::{
                DummyResponseHandle, MotorMappingError, MotorMappingResponseHandle,
                WrapperResponseHandle, WriteResponseHandle,
            },
            ResponseHandle,
        },
        DriverError, InnerDriver,
    },
    motor_common_functions, DResult,
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
    fmt::{Arguments, Debug, Display},
    io::Write,
    marker::PhantomData,
    mem,
    rc::Rc,
};

//

/// Serves to allow 2 different implementations some functions
///
/// Makes it possible to allow different implementations for all funcitons that
/// are affected by the automatic status sending (see also [`SendAutoStatus`],
/// [`NoSendAutoStatus`] and [1.5.33 Setting automatic sending of the status](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf))
pub trait AutoStatusMode {}

/// Serves to allow 2 different implementations some functions
///
/// Makes it possible to allow different implementations for all funcitons that
/// are affected by the automatic status sending (see also [`NoSendAutoStatus`]
/// and [1.5.33 Setting automatic sending of the status](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf))
#[derive(Debug)]
pub struct SendAutoStatus();

impl AutoStatusMode for SendAutoStatus {}

/// Serves to allow 2 different implementations some functions
///
/// Makes it possible to allow different implementations for all funcitons that
/// are affected by the automatic status sending (see also [`SendAutoStatus`] and
/// [1.5.33 Setting automatic sending of the status](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf))
#[derive(Debug)]
pub struct NoSendAutoStatus();

impl AutoStatusMode for NoSendAutoStatus {}

//

// builds a function with type Fn(&[u8]) -> Result<T, DriverError>
// from the given Parser<&[u8], T, ParseError<&[u8]>
//
// this cant really be a seperate function, see https://github.com/rust-lang/rust/issues/86921
// this is the reason all the read macros exist
macro_rules! build_read_parser {
    ($parser:expr) => {{
        move |input| {
            let (remainder, t) = $parser
                .parse(input)
                .finish()
                .map_err(|_: ParseError<&[u8]>| DriverError::NonMatchingPayloads(input.to_vec()))?;
            if !remainder.is_empty() {
                Err(DriverError::NonMatchingPayloads(input.to_vec()))
            } else {
                Ok(t)
            }
        }
    }};
}

// sends read command to motor
// in theory a read command isn't diffrent than a write command, the
// difference is in the returned responsehandle.
//
// self is the Motor the macro was called in.
// parser is a parser (nom::Parser) that recognises the the response of the sent
// command and extracts a value.
// args are the format_arguments for the payload to send.
//
// panics if the respondmode is NotQuiet
//
// an invocation of this macro could look like this:
// read!(self, preceded(tag("Zs"), parse_i32), format_args!("Zs"))
//
// usually it is invoked by the short_read or long_read macros though
//
// in theory this could be put into a seperate function but for some reason
// the compiler doesn't really like it that way due to lifetimes
// maybe in the future
macro_rules! read {
    ($self:expr, $parser:expr, $args:expr) => {{
        let mut driver = $self.driver.borrow_mut();
        assert_eq!(
            driver.get_respond_mode($self.address),
            RespondMode::NotQuiet
        );
        // send command
        driver.send_single_with_response($self.address, $args)?;
        Ok(ReadResponseHandle::new(
            Rc::clone(&$self.driver),
            $self.address,
            build_read_parser!($parser),
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
    ($self:expr, $mnemonic:expr, $parser:expr) => {{
        read!(
            $self,
            nom::sequence::preceded(
                nom::sequence::preceded(
                    nom::bytes::complete::tag(map::READ),
                    nom::bytes::complete::tag($mnemonic),
                ),
                $parser,
            ),
            format_args!("Z{}", $mnemonic)
        )
    }};
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
    ($self:expr, $mnemonic:expr, $parser:expr) => {{
        read!(
            $self,
            nom::sequence::preceded(nom::bytes::complete::tag($mnemonic), $parser),
            format_args!("{}", $mnemonic)
        )
    }};
}

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
/// To send commands to a motor in quick succession, the [`RespondMode`] can
/// be set to [`Quiet`][RespondMode::Quiet] with [`set_respond_mode`][Motor::set_respond_mode].
/// In this case the motor doesn't respond to commands. Be aware though that
/// getters will panic. See also [1.6.4 Reading out the current record](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf#%5B%7B%22num%22%3A123%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C113%2C742%2Cnull%5D)
///
/// ## AutoStatusMode
/// The generic `AS` denotes whether or not the motor automatically sends its
/// status after moving. It's implemented as a marker struct so that there
/// can be different `start_motor` implementations with different returntypes,
/// because if automatic status sending is enabled, we need to receive that as well.
/// See also [`Motor<NoSendAutoStatus>::start_motor`], [`Motor<SendAutoStatus>::start_motor`],
/// [`start_sending_auto_status`][Motor::start_sending_auto_status],
/// [`stop_sending_auto_status`][Motor::stop_sending_auto_status]
/// [1.5.33 Setting automatic sending of the status](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)
///
/// # Errors
/// If a value doesn't match the specifications of the corresponding command
/// in the manual, [`DriverError::InvalidArgument`] is returned. If the given motor
/// was already waiting for a response, [`DriverError::NotAvailable`] is returned.
/// Or if a command is sent to a single motors while not all motors have responded
/// to a command for all motors yet, since it isn't possible to distinguish the
/// responses from single motors in that case.
/// A [`DriverError::IoError`] is also possible, if there was an error sending the
/// command.
///
/// # Panics
/// Panics if a getter is called and the [`RespondMode`] is
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
/// let mut driver = Driver::new(s).unwrap();
/// let mut m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
///
/// m1.load_record(3).unwrap().wait().unwrap();
/// m1.set_continuation_record(None).unwrap().wait().unwrap();
/// m1.start_motor().unwrap().wait().unwrap();
///
/// let steps = m1.get_travel_distance().unwrap().wait().unwrap();
/// let max_freq = m1.get_max_frequency().unwrap().wait().unwrap();
/// println!("Driving {} steps with a top speed of {} steps/min", steps, max_freq);
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
/// let mut driver = Driver::new(s).unwrap();
/// let mut m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
/// let mut m2 = driver.add_motor(2, RespondMode::NotQuiet).unwrap();
///
/// let handle1 = m1.start_motor().unwrap();
/// let handle2 = m2.start_motor().unwrap();
/// println!("started motors");
/// handle1.wait().unwrap();
/// handle2.wait().unwrap();
/// ```
#[derive(Debug)]
// needed for transmutation
// unfortunately there's no better way
#[repr(C)]
pub struct Motor<AS: AutoStatusMode> {
    driver: Rc<RefCell<InnerDriver>>,
    address: u8,
    marker_as: PhantomData<AS>,
}

// DResult<impl ResponseHandle<Ret = T>> is not an alias since aliases with
// impl aren't supported yet
impl<AS: AutoStatusMode> Motor<AS> {
    fn write(
        &self,
        args: Arguments<'_>,
        expect_args: Arguments<'_>,
    ) -> Result<WrapperResponseHandle, DriverError> {
        let rm = {
            let mut driver = self.driver.borrow_mut();
            // FIXME maybe move this logic to driver and return RespondMode
            let rm = driver.get_respond_mode(self.address);
            if rm == RespondMode::NotQuiet {
                driver.send_single_with_response(self.address, args)?;
            } else {
                driver.send_single_no_response(self.address, args)?;
            }
            rm
        };
        Ok(match rm {
            RespondMode::NotQuiet => {
                // unfortunately there isn't a better way rn.
                // value chosen sorta random, 64 bytes should be enough for nearly all
                // commands tho
                let mut expect = Vec::with_capacity(64);
                expect.write_fmt(expect_args)?;
                WrapperResponseHandle::Write(WriteResponseHandle::new(
                    Rc::clone(&self.driver),
                    MotorAddress::Single(self.address),
                    expect,
                ))
            }
            RespondMode::Quiet => WrapperResponseHandle::Dummy(DummyResponseHandle::new(())),
        })
    }

    /// Returns this motors address directly from memory. To actually ask the motor
    /// what its address is, see [`get_drive_address`][Motor::get_drive_address]
    pub fn address(&self) -> u8 {
        self.address
    }

    motor_common_functions! {Motor::write}

    pub fn get_motor_type(&mut self) -> DResult<impl ResponseHandle<Ret = MotorType>> {
        long_read!(self, map::MOTOR_TYPE, MotorType::parse)
    }

    pub fn get_phase_current(&mut self) -> DResult<impl ResponseHandle<Ret = u8>> {
        short_read!(self, map::PHASE_CURRENT, parse_su8)
    }

    pub fn get_standstill_phase_current(&mut self) -> DResult<impl ResponseHandle<Ret = u8>> {
        short_read!(self, map::STANDSTILL_PHASE_CURRENT, parse_su8)
    }

    pub fn get_bldc_peak_current(&mut self) -> DResult<impl ResponseHandle<Ret = u8>> {
        long_read!(self, map::BLDC_PEAK_CURRENT, parse_su8)
    }

    pub fn get_bldc_current_time_constant(&mut self) -> DResult<impl ResponseHandle<Ret = u16>> {
        long_read!(self, map::BLDC_CURRENT_TIME_CONSTANT, parse_su16)
    }

    pub fn get_step_mode(&mut self) -> DResult<impl ResponseHandle<Ret = StepMode>> {
        short_read!(self, map::STEP_MODE, StepMode::parse)
    }

    /// This will actually send a message to the motor asking it what its address
    /// is. For a faster and less-hassle version see [`address`][Motor::address]
    pub fn get_drive_address(&mut self) -> DResult<impl ResponseHandle<Ret = u8>> {
        short_read!(self, map::DRIVE_ADDRESS, parse_su8)
    }

    pub fn get_motor_id(&mut self) -> DResult<impl ResponseHandle<Ret = u32>> {
        long_read!(self, map::MOTOR_ID, parse_su32)
    }

    pub fn get_limit_switch_behavior(
        &mut self,
    ) -> DResult<impl ResponseHandle<Ret = LimitSwitchBehavior>> {
        short_read!(self, map::LIMIT_SWITCH_BEHAVIOR, LimitSwitchBehavior::parse)
    }

    pub fn get_error_correction_mode(
        &mut self,
    ) -> DResult<impl ResponseHandle<Ret = ErrorCorrectionMode>> {
        short_read!(self, map::ERROR_CORRECTION_MODE, ErrorCorrectionMode::parse)
    }

    pub fn get_auto_correction_record(&mut self) -> DResult<impl ResponseHandle<Ret = u8>> {
        short_read!(self, map::AUTO_CORRECTION_RECORD, parse_su8)
    }

    // TODO encoder direction

    pub fn get_swing_out_time(&mut self) -> DResult<impl ResponseHandle<Ret = u8>> {
        short_read!(self, map::SWING_OUT_TIME, parse_su8)
    }

    pub fn get_max_encoder_deviation(&mut self) -> DResult<impl ResponseHandle<Ret = u8>> {
        short_read!(self, map::MAX_ENCODER_DEVIATION, parse_su8)
    }

    pub fn get_feedrate_numerator(&mut self) -> DResult<impl ResponseHandle<Ret = u32>> {
        long_read!(self, map::FEED_RATE_NUMERATOR, parse_su32)
    }

    pub fn get_feedrate_denominator(&mut self) -> DResult<impl ResponseHandle<Ret = u32>> {
        long_read!(self, map::FEED_RATE_DENOMINATOR, parse_su32)
    }

    pub fn get_error(&mut self, p: u8) -> DResult<impl ResponseHandle<Ret = MotorError>> {
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

    pub fn get_encoder_position(&mut self) -> DResult<impl ResponseHandle<Ret = i32>> {
        short_read!(self, map::READ_ENCODER_POS, parse_i32)
    }

    pub fn get_position(&mut self) -> DResult<impl ResponseHandle<Ret = i32>> {
        short_read!(self, map::READ_POS, parse_i32)
    }

    pub fn is_motor_referenced(&mut self) -> DResult<impl ResponseHandle<Ret = bool>> {
        long_read!(self, map::IS_REFERENCED, parse_su8.map(|n| n == 1))
    }

    // TODO reading out status

    pub fn get_firmware_version(&mut self) -> DResult<impl ResponseHandle<Ret = FirmwareVersion>> {
        short_read!(self, map::READ_FIRMWARE_VERSION, FirmwareVersion::parse)
    }

    pub fn get_operating_time(&mut self) -> DResult<impl ResponseHandle<Ret = u64>> {
        long_read!(self, map::READ_OPERATING_TIME, parse_su64)
    }

    /// This command is not in the manual. It just gives you the time the motor
    /// operation started in the current time zone, similar to [`get_operating_time`][Motor::get_operating_time]
    pub fn get_operation_start(&mut self) -> DResult<impl ResponseHandle<Ret = DateTime<Local>>> {
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
    ) -> DResult<impl ResponseHandle<Ret = DigitalInputFunction>> {
        ensure!(i <= 8 && i >= 1, DriverError::InvalidArgument);
        let mut name = map::DIGITAL_INPUT_FUNCTION_PARTIAL.to_string();
        name.push(char::from_u32(60 + (i as u32)).unwrap());
        long_read!(self, name.as_str(), DigitalInputFunction::parse)
    }

    pub fn get_digital_output_function(
        &mut self,
        o: u8,
    ) -> DResult<impl ResponseHandle<Ret = DigitalOutputFunction>> {
        ensure!(o <= 8 && o >= 1, DriverError::InvalidArgument);
        let mut name = map::DIGITAL_OUTPUT_FUNCTION_PARTIAL.to_string();
        name.push(char::from_u32(60 + (o as u32)).unwrap());
        long_read!(self, name.as_str(), DigitalOutputFunction::parse)
    }

    // Not implementing Masking and demasking inputs since it is deprecated

    pub fn get_reverse_in_out_polarity(&mut self) -> DResult<impl ResponseHandle<Ret = u32>> {
        short_read!(self, map::REVERSE_IN_OUT_POLARITY, parse_su32)
    }

    pub fn get_input_debounce_time(&mut self) -> DResult<impl ResponseHandle<Ret = u8>> {
        short_read!(self, map::INPUT_DEBOUNCE_TIME, parse_su8)
    }

    // TODO read eeprom byte
    // TODO start bootloader

    pub fn get_reverse_clearance(&mut self) -> DResult<impl ResponseHandle<Ret = u16>> {
        short_read!(self, map::REVERSE_CLEARANCE, parse_su16)
    }

    pub fn get_ramp_type(&mut self) -> DResult<impl ResponseHandle<Ret = RampType>> {
        long_read!(self, map::RAMP_TYPE, RampType::parse)
    }

    pub fn get_brake_voltage_off_wait_time(&mut self) -> DResult<impl ResponseHandle<Ret = u16>> {
        long_read!(self, map::WAIT_TIME_BRAKE_VOLTAGE_OFF, parse_su16)
    }

    pub fn get_motor_movement_wait_time(&mut self) -> DResult<impl ResponseHandle<Ret = u16>> {
        long_read!(self, map::WAIT_TIME_MOTOR_MOVE, parse_su16)
    }

    pub fn get_motor_current_off_wait_time(&mut self) -> DResult<impl ResponseHandle<Ret = u16>> {
        long_read!(self, map::WAIT_TIME_MOTOR_CURRENT_OFF, parse_su16)
    }

    pub fn get_baud_rate(&mut self) -> DResult<impl ResponseHandle<Ret = BaudRate>> {
        long_read!(self, map::BAUD_RATE, BaudRate::parse)
    }

    // TODO set crc checksum
    // TODO set hall config

    pub fn get_temp_raw(&mut self) -> DResult<impl ResponseHandle<Ret = u16>> {
        short_read!(self, map::READ_TEMP, parse_su16)
    }

    pub fn get_temp(&mut self) -> DResult<impl ResponseHandle<Ret = f64>> {
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

    pub fn get_quickstop_ramp(&mut self) -> DResult<impl ResponseHandle<Ret = u16>> {
        short_read!(self, map::QUICKSTOP_RAMP, parse_su16)
    }

    pub fn get_quickstop_ramp_no_conversion(&mut self) -> DResult<impl ResponseHandle<Ret = u32>> {
        long_read!(self, map::QUICKSTOP_RAMP_NO_CONVERSION, parse_su32)
    }

    pub fn get_gearfactor_numerator(&mut self) -> DResult<impl ResponseHandle<Ret = u8>> {
        long_read!(self, map::GEAR_FACTOR_NUMERATOR, parse_su8)
    }

    pub fn get_gearfactor_denominator(&mut self) -> DResult<impl ResponseHandle<Ret = u8>> {
        long_read!(self, map::GEAR_FACTOR_DENOMINATOR, parse_su8)
    }

    /// This method doesn't return a ResponseHandle since it doesn't actually send
    /// a command from the motor but rather just returns a value stored within
    /// this struct. See also [`set_respond_mode`][Motor::set_respond_mode]
    pub fn get_respond_mode(&mut self) -> RespondMode {
        self.driver.borrow().get_respond_mode(self.address)
    }

    /// See [`set_respond_mode`][Motor::set_respond_mode]
    pub fn get_current_record(&mut self) -> DResult<impl ResponseHandle<Ret = Record>> {
        read!(
            self,
            preceded(tag(map::READ), Record::parse),
            format_args!("{}{}", map::READ, map::READ_CURRENT_RECORD)
        )
    }

    /// See [`set_respond_mode`][Motor::set_respond_mode]
    pub fn get_record(&mut self, n: u8) -> DResult<impl ResponseHandle<Ret = Record>> {
        ensure!(n <= 32, DriverError::InvalidArgument);
        read!(
            self,
            // FIXME concrete value instead of just parse_su8
            preceded(tuple((tag(map::READ), parse_su8)), Record::parse),
            format_args!("{}{}{}", map::READ, n, map::READ_CURRENT_RECORD)
        )
    }

    /// This command doesn't exist in the manual.
    /// [1.6.4 Reading out the current record](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)
    /// was so convoluted that it was split into multiple functions, namely this
    /// one, [`get_current_record`][Motor::get_current_record],
    /// [`get_record`][Motor::get_record], [`get_respond_mode`][Motor::get_respond_mode]
    /// and [`AllMotor::set_respond_mode`][super::all::AllMotor::set_respond_mode].
    /// This function is responsible for setting whether or not the firmware
    /// responds to most commands and the other 2 are responsible for actually
    /// reading out records.
    pub fn set_respond_mode(
        &mut self,
        mode: RespondMode,
    ) -> DResult<impl ResponseHandle<Ret = ()>> {
        self.driver
            .borrow_mut()
            .set_respond_mode(self.address, mode);
        self.short_write(map::READ_CURRENT_RECORD, mode)
    }

    pub fn get_positioning_mode(&mut self) -> DResult<impl ResponseHandle<Ret = PositioningMode>> {
        short_read!(self, map::POSITIONING_MODE, PositioningMode::parse)
    }

    pub fn get_travel_distance(&mut self) -> DResult<impl ResponseHandle<Ret = i32>> {
        short_read!(self, map::TRAVEL_DISTANCE, parse_i32)
    }

    pub fn get_min_frequency(&mut self) -> DResult<impl ResponseHandle<Ret = u32>> {
        short_read!(self, map::MIN_FREQUENCY, parse_su32)
    }

    pub fn get_max_frequency(&mut self) -> DResult<impl ResponseHandle<Ret = u32>> {
        short_read!(self, map::MAX_FREQUENCY, parse_su32)
    }

    pub fn get_max_frequency2(&mut self) -> DResult<impl ResponseHandle<Ret = u32>> {
        short_read!(self, map::MAX_FREQUENCY2, parse_su32)
    }

    pub fn get_accel_ramp(&mut self) -> DResult<impl ResponseHandle<Ret = u16>> {
        short_read!(self, map::ACCEL_RAMP, parse_su16)
    }

    pub fn get_accel_ramp_no_conversion(&mut self) -> DResult<impl ResponseHandle<Ret = u32>> {
        long_read!(self, map::ACCEL_RAMP_NO_CONVERSION, parse_su32)
    }

    pub fn get_brake_ramp(&mut self) -> DResult<impl ResponseHandle<Ret = u16>> {
        short_read!(self, map::BRAKE_RAMP, parse_su16)
    }

    pub fn get_brake_ramp_no_conversion(&mut self) -> DResult<impl ResponseHandle<Ret = u32>> {
        long_read!(self, map::BRAKE_RAMP_NO_CONVERSION, parse_su32)
    }

    pub fn get_rotation_direction(
        &mut self,
    ) -> DResult<impl ResponseHandle<Ret = RotationDirection>> {
        short_read!(self, map::ROTATION_DIRECTION, RotationDirection::parse)
    }

    pub fn get_rotation_direction_change(&mut self) -> DResult<impl ResponseHandle<Ret = bool>> {
        short_read!(
            self,
            map::ROTATION_DIRECTION_CHANGE,
            parse_su8.map(|n| n == 1)
        )
    }

    pub fn get_repetitions(&mut self) -> DResult<impl ResponseHandle<Ret = Repetitions>> {
        short_read!(self, map::REPETITIONS, Repetitions::parse)
    }

    pub fn get_record_pause(&mut self) -> DResult<impl ResponseHandle<Ret = u16>> {
        short_read!(self, map::RECORD_PAUSE, parse_su16)
    }

    pub fn get_continuation_record(&mut self) -> DResult<impl ResponseHandle<Ret = Option<u8>>> {
        short_read!(
            self,
            map::CONTINUATION_RECORD,
            parse_su8.map(|r| {
                if r == 0 {
                    None
                } else {
                    Some(r)
                }
            })
        )
    }

    pub fn get_max_accel_jerk(&mut self) -> DResult<impl ResponseHandle<Ret = u32>> {
        short_read!(self, map::MAX_ACCEL_JERK, parse_su32)
    }

    pub fn get_max_brake_jerk(&mut self) -> DResult<impl ResponseHandle<Ret = u32>> {
        short_read!(self, map::MAX_BRAKE_JERK, parse_su32)
    }
}

impl Motor<NoSendAutoStatus> {
    pub(in super::super) fn new(driver: Rc<RefCell<InnerDriver>>, address: u8) -> Self {
        Motor {
            driver,
            address,
            marker_as: PhantomData,
        }
    }

    /// Enables [1.5.33 Setting automatic sending of the status](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)
    ///
    /// This function consumes the motor and if the command was successful,
    /// returns a motor with type `Motor<I, SendAutoStatus>`. There, the
    /// return-type of [`start_motor`][Motor::start_motor] is adjusted to be then
    /// able to wait on the motor status, making it possble to know when the motor
    /// finished.
    ///
    /// # Errors
    /// If the command fails for any reason, a [`MotorMappingError`] is returned.
    /// It contains the actual [`DriverError`] and the original motor, making
    /// it possible to try again.
    pub fn start_sending_auto_status(
        self,
    ) -> Result<
        impl ResponseHandle<MotorMappingError<NoSendAutoStatus>, Ret = Motor<SendAutoStatus>>,
        MotorMappingError<NoSendAutoStatus>,
    > {
        let rh = self.short_write(map::AUTO_STATUS_SENDING, 1);
        let driver = Rc::clone(&self.driver);
        let address = self.address;
        match rh {
            Ok(h) => Ok(MotorMappingResponseHandle::new(self, h, move |m| unsafe {
                driver.borrow_mut().set_send_autostatus(address, true);
                // Safety:
                // Motor is repr(C), meaning we can assume the repr will be
                // the same way
                // PhantomData is a ZST, meaning it doesn't matter what we cast
                // it to
                //
                // Unfortunately there's not really a better way, at least not
                // one that won't really clutter code with unwraps and so on
                // FIXME ?
                mem::transmute::<Motor<NoSendAutoStatus>, Motor<SendAutoStatus>>(m)
            })),
            Err(e) => Err(MotorMappingError(self, e)),
        }
    }

    /// Starts the motor, also see [1.6.1 Starting a motor](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)
    ///
    /// This function is only used when [1.5.33 Setting automatic sending of the status](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)
    /// isn't enabled. See also [`start_sending_auto_status`][Motor::start_sending_auto_status]
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
    /// let mut driver = Driver::new(s).unwrap();
    /// let mut m = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    ///
    /// let handle = m.start_motor().unwrap().wait().unwrap();
    /// println!("started motor");
    /// ```
    pub fn start_motor(&mut self) -> DResult<impl ResponseHandle<Ret = ()>> {
        self.short_write(map::START_MOTOR, "")
    }
}

impl Motor<SendAutoStatus> {
    /// Disables [1.5.33 Setting automatic sending of the status](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)
    ///
    /// This function consumes the motor and if the command was successful,
    /// returns a motor with type `Motor<I, NoSendAutoStatus>`. There, the
    /// return-type of [`start_motor`][Motor::start_motor] is adjusted to not
    /// return anything.
    ///
    /// # Errors
    /// If the command fails for any reason, a [`MotorMappingError`] is returned.
    /// It contains the actual [`DriverError`] and the original motor, making
    /// it possible to try again.
    pub fn stop_sending_auto_status(
        self,
    ) -> Result<
        impl ResponseHandle<MotorMappingError<SendAutoStatus>, Ret = Motor<NoSendAutoStatus>>,
        MotorMappingError<SendAutoStatus>,
    > {
        let rh = self.short_write(map::AUTO_STATUS_SENDING, 0);
        let driver = Rc::clone(&self.driver);
        let address = self.address;
        match rh {
            Ok(h) => Ok(MotorMappingResponseHandle::new(self, h, move |m| unsafe {
                driver.borrow_mut().set_send_autostatus(address, false);
                // Safety:
                // Motor is repr(C), meaning we can assume the repr will be
                // the same way
                // PhantomData is a ZST, meaning it doesn't matter what we cast
                // it to
                //
                // Unfortunately there's not really a better way, at least not
                // one that won't really clutter code with unwraps and so on
                // FIXME ?
                mem::transmute::<Motor<SendAutoStatus>, Motor<NoSendAutoStatus>>(m)
            })),
            Err(e) => Err(MotorMappingError(self, e)),
        }
    }

    /// Starts the motor, also see [1.6.1 Starting a motor](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)
    ///
    /// This function is only used when [1.5.33 Setting automatic sending of the status](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)
    /// is enabled. This means that the first handle returns another handle that,
    /// when waited on, returns the status of the motor. See also
    /// [`start_sending_auto_status`][Motor::start_sending_auto_status]
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
    /// let mut driver = Driver::new(s).unwrap();
    /// let mut m = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    ///
    /// let mut m = m.start_sending_auto_status().unwrap().wait().unwrap();
    ///
    /// let handle = m.start_motor().unwrap().wait().unwrap();
    /// println!("started motor");
    /// let status = handle.wait().unwrap();
    /// println!("motor finished");
    /// ```
    pub fn start_motor(
        &mut self,
    ) -> DResult<impl ResponseHandle<Ret = impl ResponseHandle<Ret = MotorStatus>>> {
        let driver = Rc::clone(&self.driver);
        let address = self.address;
        self.short_write(map::START_MOTOR, "")
            .map(move |h| h.map(move |()| StatusResponseHandle::new(driver, address)))
    }
}

impl<AS: AutoStatusMode> Drop for Motor<AS> {
    /// Removes this motor from the driver.\
    /// Afterwards, a motor with this address can be added again by calling
    /// [`Driver::add_motor`][super::super::Driver::add_motor].
    ///
    /// See also [here][`Drop`]
    fn drop(&mut self) {
        self.driver.borrow_mut().drop_motor(&self.address)
    }
}
