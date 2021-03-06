#[cfg(test)]
mod tests;

pub mod cmd;
mod map;
pub mod motor;
mod parse;
pub mod responsehandle;

use self::{
    cmd::{
        frame::{MotorAddress, Msg, MsgWrap},
        payload::RespondMode,
    },
    motor::{AllMotor, Motor},
    parse::ParseError,
};
use crate::util::ensure;
use nom::Finish;
use std::{
    cell::RefCell,
    collections::{HashMap, VecDeque},
    fmt::Arguments,
    io::{self, BufRead, BufReader, Read, Write},
    rc::Rc,
};
use thiserror::Error;

// TODO implement logging

// unfortunately, due to rustfmt not having the blank_lines_upper_bound feature
// stable yet, we gotta put comments in between the different sections. otherwise
// its just too much

//

/// Errors returned by any part of the driver
// TODO improve error granularity
#[derive(Error, Debug)]
pub enum DriverError {
    /// Thrown by [`Driver::add_motor`] if the given address was out of bounds
    #[error("address must be 1 <= address <= 254, was {0}")]
    InvalidAddress(u8),
    /// Thrown by [`Driver::add_motor`] if a motor with that address already
    /// exists in that driver
    #[error("motor already exists: {0}")]
    AlreadyExists(MotorAddress),
    /// Thrown if any funtion in [`Motor`] receives a response that didn't match
    /// what was sent
    #[error("payloads from response and cmd don't match, response was {0:?}")]
    NonMatchingPayloads(Vec<u8>),
    /// Thrown by a function in [`Motor`] if the argument didn't match the
    /// requirements specified in the manual
    #[error("invalid value for command argument")]
    InvalidArgument,
    /// Thrown if a motor with an address the driver didn't know responded,
    /// meaning that `driver.add_motor(<address>)` wasn't called beforehand.
    #[error("unexpected motor with address {0} responded")]
    UnexpectedResponse(MotorAddress),
    /// Thrown by a function in [`Motor`] if the given motor is already waiting
    /// on a response
    #[error("motor is already waiting on a response and therefore not available")]
    NotAvailable,
    /// Thrown if the motor finds the command to be invalid
    #[error("{0:?}")]
    InvalidCmd(Vec<u8>),
    /// Wrapper around [`io::Error`]
    #[error(transparent)]
    IoError(#[from] io::Error),
    /// Wrapper arund [`nom::error::Error`]
    #[error("{0}")]
    ParsingError(ParseError<Vec<u8>>),
}

impl From<ParseError<&[u8]>> for DriverError {
    fn from(e: ParseError<&[u8]>) -> Self {
        Self::ParsingError(match e {
            ParseError::InvalidValue => ParseError::InvalidValue,
            ParseError::NomError(e) => ParseError::NomError(nom::error::Error {
                input: e.input.to_vec(),
                code: e.code,
            }),
        })
    }
}

//

#[derive(Debug)]
struct InnerMotor {
    available: bool,
    respond_mode: RespondMode,
    queue: VecDeque<Vec<u8>>,
}

#[derive(Debug)]
struct InnerDriver<I: Write + Read> {
    // TODO make interface also BufWriter so we make less syscalls
    interface: BufReader<I>,
    // TODO optimise
    // the u8 basically acts like a semaphore, once it reaches 0 we know we
    // received answers from all motors. it is initialized to the current motor
    // count in send_all
    all: Option<(u8, Vec<u8>)>,
    all_exists: bool,
    motors: HashMap<u8, InnerMotor>,
}

impl<I: Write + Read> InnerDriver<I> {
    // Should only be called by the drop function in Motor
    pub fn drop_motor(&mut self, address: &u8) {
        self.motors.remove(address);
    }

    // Should only be called by the drop function in Motor
    pub fn drop_all_motor(&mut self) {
        self.all_exists = false;
    }

    // check if the driver is currently waiting for replies to a command for all
    // motors
    fn is_waiting_all(&self) -> bool {
        self.all.as_ref().map_or(false, |a| a.0 != 0)
    }

    // receives a single msg from the interface and parses it into a Msg
    fn receive_msg(&mut self) -> Result<Msg, DriverError> {
        // size chosen more or less randomly, should fit most messages
        let mut buf = Vec::with_capacity(64);
        self.interface.read_until(b'\r', &mut buf)?;
        let (_, wrap) = MsgWrap::parse(&buf).finish()?;
        match wrap {
            MsgWrap::Valid(msg) => Ok(msg),
            MsgWrap::Invalid(msg) => Err(DriverError::InvalidCmd(msg.payload)),
        }
    }

    pub fn get_respond_mode(&self, address: u8) -> RespondMode {
        // shouldn't panic since it's only called from motor, where the address
        // is definitely in the driver
        self.motors.get(&address).unwrap().respond_mode
    }

    pub fn set_respond_mode(&mut self, address: u8, mode: RespondMode) {
        // shouldn't panic since it's only called from motor, where the address
        // is definitely in the driver
        self.motors.get_mut(&address).unwrap().respond_mode = mode;
    }

    pub fn set_respond_mode_all(&mut self, mode: RespondMode) {
        self.motors.values_mut().for_each(|m| m.respond_mode = mode);
    }

    // TODO error out if cmd for other motor is received and that motor wasn't waiting?
    pub fn receive_single(&mut self, address: u8) -> Result<Vec<u8>, DriverError> {
        // if there's one already in the buffer, return that
        // shouldn't panic because this shouldn't get called on an address that
        // doesn't already exist
        if let Some(p) = self.motors.get_mut(&address).unwrap().queue.pop_front() {
            Ok(p)
        } else {
            // if we're waiting for a response from a single motors, there shouldn't
            // be a response to an all command so we can massively
            // simplify the receiving process
            loop {
                let msg = self.receive_msg()?;
                if let MotorAddress::Single(a) = msg.address {
                    let imotor = self
                        .motors
                        .get_mut(&a)
                        // if motor address was unknown
                        .ok_or(DriverError::UnexpectedResponse(MotorAddress::Single(a)))?;
                    imotor.available = true;
                    if a == address {
                        return Ok(msg.payload);
                    } else {
                        imotor.queue.push_back(msg.payload);
                    }
                } else {
                    // if a motor responded to an all command
                    return Err(DriverError::UnexpectedResponse(MotorAddress::All));
                }
            }
        }
    }

    pub fn receive_all(&mut self) -> Result<Vec<u8>, DriverError> {
        if self.is_waiting_all() {
            // if we're waiting for a response from all motors, there shouldn't
            // be a response from one with a singular address so we can massively
            // simplify the receiving process
            for _ in 0..self.all.as_ref().unwrap().0 {
                let msg = self.receive_msg()?;
                ensure!(
                    msg.address.is_all(),
                    DriverError::UnexpectedResponse(msg.address)
                );
                // if were receiving there should be something in there
                let all = self.all.as_mut().unwrap();
                ensure!(
                    msg.payload == all.1,
                    DriverError::NonMatchingPayloads(msg.payload)
                );
                all.0 -= 1;
            }
        }
        Ok(self.all.take().unwrap().1)
    }

    // send_single split in 2 functions cause there are cases like reading values
    // where there will always be a response. this isn't the case with send_all
    // since there only write functions are allowed
    pub fn send_single_no_response(
        &mut self,
        address: u8,
        args: Arguments<'_>,
    ) -> Result<(), DriverError> {
        // if we are waiting for a response from all motors we can't send to
        // a singular motor because we don't know if it's ready yet. only when
        // all motors answered we can be sure
        ensure!(!self.is_waiting_all(), DriverError::NotAvailable);
        // shouldn't panic since the motor has to exist if a message was sent
        // to it
        let imotor = self.motors.get(&address).unwrap();
        ensure!(imotor.available, DriverError::NotAvailable);
        write!(self.interface.get_mut(), "#{}{}\r", address, args)?;
        Ok(())
    }

    pub fn send_single_with_response(
        &mut self,
        address: u8,
        args: Arguments<'_>,
    ) -> Result<(), DriverError> {
        self.send_single_no_response(address, args)?;
        self.motors.get_mut(&address).unwrap().available = false;
        Ok(())
    }

    pub fn send_all(&mut self, args: Arguments<'_>) -> Result<RespondMode, DriverError> {
        // since messages to all motors shouldn't happen that often it can take
        // a bit longer. if it's a problem a semaphore would be a fix for it
        ensure!(
            self.motors.values().all(|m| m.available) && !self.is_waiting_all(),
            DriverError::NotAvailable
        );
        let res_cnt = self
            .motors
            .values()
            .map(|m| m.respond_mode.is_responding() as u8)
            .fold(0, std::ops::Add::add);
        // TODO optimise sending by not copying message 2 times
        // size chosen more or less randomly, should fit most messages
        let iface = self.interface.get_mut();
        iface.write_all(b"#*")?;
        // if we gotta wait, we need to save what we sent, otherwise not
        if res_cnt > 0 {
            let mut sent = Vec::with_capacity(64);
            write!(sent, "{}", args)?;
            iface.write_all(&sent)?;
            self.all = Some((res_cnt, sent));
        } else {
            iface.write_fmt(args)?;
        }
        iface.write_all(b"\r")?;
        if res_cnt == 0 {
            Ok(RespondMode::Quiet)
        } else {
            Ok(RespondMode::NotQuiet)
        }
    }
}

//

/// Represents a single Rs485 connection with motors attached to it
///
/// `I` is the interface used to actually communicate with the motors, usually
/// a serialport. The driver itself doesn't really do much, to actually communicate
/// with a motor, a [`Motor`] is needed, which can be obtained by calling
/// [`add_motor`][Driver::add_motor].
///
/// Dropping the driver won't drop or close the interface. Only once the driver,
/// all [`Motor`]s and all [`ResponseHandle`][responsehandle::ResponseHandle]s
/// that originiated from the driver are dropped is the interface acually dropped,
/// but not closed.
#[derive(Debug)]
pub struct Driver<I: Write + Read> {
    inner: Rc<RefCell<InnerDriver<I>>>,
}

impl<I: Write + Read> Driver<I> {
    /// Returns new Driver. `I` is the interface used to actually communicate
    /// with the motors. Usually it's a serialport. Since the motors usually
    /// take a while to reply, especially when they're moving, the timeout of `I`
    /// (if there is one) should be set to something sensible and big.
    ///
    /// # Examples
    /// ```no_run
    /// # use nanotec_stepper_driver::Driver;
    /// use std::time::Duration;
    /// use serialport;
    ///
    /// let s = serialport::new("/dev/ttyUSB0", 115200)
    ///     .timeout(Duration::from_secs(1))
    ///     .open()
    ///     .unwrap();
    /// let driver = Driver::new(s);
    /// ```
    pub fn new(interface: I) -> Self {
        Driver {
            inner: Rc::new(RefCell::new(InnerDriver {
                // wrap into bufreader so receiving until '\r' is easier
                interface: BufReader::new(interface),
                // chosen more or less arbitrarily
                all: None,
                all_exists: false,
                // maximium number of motors
                motors: HashMap::with_capacity(254),
            })),
        }
    }

    /// Returns a motor of the given address. `respond_mode` ist the respond mode
    /// the motor is currently in (see also [1.6.4 Reading out the current record](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf#%5B%7B%22num%22%3A123%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C113%2C742%2Cnull%5D),
    /// as well as [`RespondMode`]). If this is wrong an error might occur while
    /// receiving the response. For more information see [`Motor`].
    ///
    /// A motor is removed from the driver simply by dropping it. See also
    /// [`Motor::drop`].
    ///
    /// # Errors
    /// Will return a [`DriverError::InvalidAddress`] if the given address is
    /// out of bounds. If a motor with this address already exists in this driver,
    /// [`DriverError::AlreadyExists`] is returned.
    ///
    /// # Examples
    /// ```no_run
    /// # use nanotec_stepper_driver::{Driver, RespondMode};
    /// use std::time::Duration;
    /// use serialport;
    ///
    /// let s = serialport::new("/dev/ttyUSB0", 115200)
    ///     .timeout(Duration::from_secs(1))
    ///     .open()
    ///     .unwrap();
    /// let mut driver = Driver::new(s);
    /// let mut m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    /// ```
    pub fn add_motor(
        &mut self,
        address: u8,
        respond_mode: RespondMode,
    ) -> Result<Motor<I>, DriverError> {
        ensure!(
            address >= 1 && address <= 254,
            DriverError::InvalidAddress(address)
        );
        let mut inner = self.inner.as_ref().borrow_mut();
        // Have to do it this way due to try_insert being nightly
        ensure!(
            !inner.motors.contains_key(&address),
            DriverError::AlreadyExists(MotorAddress::Single(address))
        );
        inner
            .motors
            // chosen more or less randomly, 4 should suffice though
            .insert(
                address,
                InnerMotor {
                    available: true,
                    respond_mode,
                    queue: VecDeque::with_capacity(4),
                },
            );
        Ok(Motor::new(
            self.inner.clone(),
            MotorAddress::Single(address),
        ))
    }

    /// Returns a motor that represents all motors, the so-called all-motor.
    /// That means that commands can be sent to all motors. That is only possible
    /// though while all motors are ready to receive and only possible on write
    /// commands, meaning mostly setters and things like starting a motor.
    ///
    /// A motor is removed from the driver simply by dropping it. See also
    /// [`Motor::drop`].
    ///
    /// # Errors
    /// If the all-motor already exists in this driver, [`DriverError::AlreadyExists`]
    /// is returned.
    ///
    /// # Examples
    /// ```no_run
    /// # use nanotec_stepper_driver::Driver;
    /// use std::time::Duration;
    /// use serialport;
    ///
    /// let s = serialport::new("/dev/ttyUSB0", 115200)
    ///     .timeout(Duration::from_secs(1))
    ///     .open()
    ///     .unwrap();
    /// let mut driver = Driver::new(s);
    /// let mut m1 = driver.add_all_motor().unwrap();
    /// ```
    pub fn add_all_motor(&mut self) -> Result<AllMotor<I>, DriverError> {
        let mut inner = self.inner.as_ref().borrow_mut();
        ensure!(
            !inner.all_exists,
            DriverError::AlreadyExists(MotorAddress::All)
        );
        inner.all_exists = true;
        Ok(Motor::new(self.inner.clone(), MotorAddress::All))
    }
}
