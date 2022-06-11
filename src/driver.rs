pub mod cmd;
mod map;
pub mod motor;
mod parse;
pub mod responsehandle;

use self::{cmd::Msg, motor::Motor};
use crate::util::ensure;
use nom::Finish;
use std::{
    cell::RefCell,
    collections::{HashMap, VecDeque},
    fmt::Arguments,
    io::{self, BufRead, BufReader, Read, Write},
    rc::Rc,
    str,
};
use thiserror::Error;

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
    AlreadyExists(u8),
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
    UnexpectedResponse(u8),
    /// Thrown by a function in [`Motor`] if the given motor is already waiting
    /// on a response
    #[error("motor is already waiting on a response and therefore not available")]
    NotAvailable,
    /// Wrapper around [`io::Error`]
    #[error(transparent)]
    IoError(#[from] io::Error),
    /// Wrapper arund [`nom::error::Error`]
    #[error("error {:?} while parsing message {:?}", .0.code, .0.input)]
    ParsingError(nom::error::Error<Vec<u8>>),
}

// dunno why, but derive won't work
impl From<nom::error::Error<Vec<u8>>> for DriverError {
    fn from(e: nom::error::Error<Vec<u8>>) -> Self {
        Self::ParsingError(e)
    }
}

impl From<nom::error::Error<&[u8]>> for DriverError {
    fn from(e: nom::error::Error<&[u8]>) -> Self {
        Self::ParsingError(nom::error::Error {
            input: e.input.to_vec(),
            code: e.code,
        })
    }
}

#[derive(Debug)]
struct InnerMotor {
    available: bool,
    queue: VecDeque<Vec<u8>>,
}

#[derive(Debug)]
struct InnerDriver<I: Write + Read> {
    interface: BufReader<I>,
    all: HashMap<Vec<u8>, u8>,
    motors: HashMap<u8, InnerMotor>,
}

impl<I: Write + Read> InnerDriver<I> {
    // Should only be called by the drop function in Motor
    fn drop_motor(&mut self, address: &u8) {
        self.motors.remove(address);
    }

    // receives a single msg from the interface and parses it into a Msg
    fn receive_msg(&mut self) -> Result<Msg, DriverError> {
        // size chosen more or less randomly, should fit most messages
        let mut buf = Vec::with_capacity(64);
        self.interface.read_until(b'\r', &mut buf)?;
        let (_, msg) = Msg::parse(&buf)
            .finish()
            // map is needed since value in error cant be reference
            .map_err(|b| nom::error::Error {
                input: b.input.to_vec(),
                code: b.code,
            })?;
        Ok(msg)
    }

    // inserts a payload into the self.all map or updates the value
    fn insert_count_all(&mut self, payload: Vec<u8>) -> () {
        if let Some(c) = self.all.get_mut(&payload) {
            *c += 1;
        } else {
            self.all.insert(payload, 1).unwrap();
        }
    }

    fn check_msg(&mut self, address: u8) -> Option<Vec<u8>> {
        // shouldn't panic since we know the address exists
        self.motors.get_mut(&address).unwrap().queue.pop_front()
    }

    // TODO adjust accordingly once receive_all is implemented
    // receives messages until one for address is found, which is then returned
    fn receive(&mut self, address: u8) -> Result<Vec<u8>, DriverError> {
        loop {
            let msg = self.receive_msg()?;
            match msg.address {
                Some(a) => {
                    if a == address {
                        // shouldn't panic since the motor has to exist if
                        // it was waited on
                        self.motors.get_mut(&a).unwrap().available = true;
                        return Ok(msg.payload);
                    } else {
                        let imotor = self
                            .motors
                            .get_mut(&a)
                            // if address is from unknown motor
                            .ok_or(DriverError::UnexpectedResponse(a))?;
                        imotor.queue.push_back(msg.payload);
                        imotor.available = true;
                    }
                }
                None => self.insert_count_all(msg.payload),
            }
        }
    }

    fn send_fmt(&mut self, address: u8, args: Arguments<'_>) -> Result<(), DriverError> {
        // shouldn't panic since the motor has to exist if a message was sent
        // to it
        let imotor = self.motors.get_mut(&address).unwrap();
        ensure!(imotor.available, DriverError::NotAvailable);
        imotor.available = false;
        write!(self.interface.get_mut(), "#{}{}\r", address, args).map_err(DriverError::from)
    }
}

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

// TODO implement different respondmodes, also in motor, which would make
// add_motor unsafe
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
                all: HashMap::with_capacity(16),
                // maximium number of motors
                motors: HashMap::with_capacity(254),
            })),
        }
    }

    /// Returns a motor of the given address.
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
    /// # use nanotec_stepper_driver::Driver;
    /// use std::time::Duration;
    /// use serialport;
    ///
    /// let s = serialport::new("/dev/ttyUSB0", 115200)
    ///     .timeout(Duration::from_secs(1))
    ///     .open()
    ///     .unwrap();
    /// let driver = Driver::new(s);
    /// let mut m1 = driver.add_motor(1).unwrap();
    /// ```
    pub fn add_motor(&self, address: u8) -> Result<Motor<I>, DriverError> {
        ensure!(
            address >= 1 && address <= 254,
            DriverError::InvalidAddress(address)
        );
        let mut inner = self.inner.as_ref().borrow_mut();
        // Have to do it this way due to try_insert being nightly
        ensure!(
            !inner.motors.contains_key(&address),
            DriverError::AlreadyExists(address)
        );
        inner
            .motors
            // chosen more or less randomly, 4 should suffice though
            .insert(
                address,
                InnerMotor {
                    available: true,
                    queue: VecDeque::with_capacity(4),
                },
            );
        Ok(Motor::new(self.inner.clone(), address))
    }
}
