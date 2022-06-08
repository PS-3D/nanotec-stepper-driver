pub mod cmd;
pub mod motor;
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

#[derive(Error, Debug)]
pub enum DriverError {
    #[error("address must be 1 <= address <= 254, was {0}")]
    InvalidAddress(u8),
    #[error("motor already exists: {0}")]
    AlreadyExists(u8),
    #[error("payloads from response and cmd don't match, response was {0:?}")]
    NonMatchingPayloads(Vec<u8>),
    #[error("invalid value for command argument")]
    InvalidArgument,
    #[error("unexpected motor with address {0} responded")]
    UnexpectedResponse(u8),
    #[error(transparent)]
    IoError(#[from] io::Error),
    #[error("error {:?} while parsing message {}", .0.code, str::from_utf8(&.0.input).unwrap())]
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
struct InnerDriver<I: Write + Read> {
    interface: BufReader<I>,
    all: HashMap<Vec<u8>, u8>,
    motors: HashMap<u8, VecDeque<Vec<u8>>>,
}

impl<I: Write + Read> InnerDriver<I> {
    // Should only be called by the drop function in Motor
    fn drop_motor(&mut self, address: &u8) {
        self.motors.remove(address);
    }

    fn receive_msg(&mut self) -> Result<Msg, DriverError> {
        let mut buf = Vec::with_capacity(32);
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

    fn insert_count_all(&mut self, payload: Vec<u8>) -> () {
        if let Some(c) = self.all.get_mut(&payload) {
            *c += 1;
        } else {
            self.all.insert(payload, 1).unwrap();
        }
    }

    fn check_msg(&mut self, address: u8) -> Option<Vec<u8>> {
        // shouldn't panic since we know the address exists
        self.motors.get_mut(&address).unwrap().pop_front()
    }

    // TODO change if receive all needs to happen
    fn receive(&mut self, address: u8) -> Result<Vec<u8>, DriverError> {
        loop {
            let msg = self.receive_msg()?;
            match msg.address {
                Some(a) => {
                    if a == address {
                        return Ok(msg.payload);
                    } else {
                        self.motors
                            .get_mut(&a)
                            .ok_or(DriverError::UnexpectedResponse(a))?
                            .push_back(msg.payload);
                    }
                }
                None => self.insert_count_all(msg.payload),
            }
        }
    }

    fn send_fmt(&mut self, args: Arguments<'_>) -> Result<(), DriverError> {
        self.interface
            .get_mut()
            .write_fmt(args)
            .map_err(DriverError::from)
    }

    fn send(&mut self, msg: &[u8]) -> Result<(), DriverError> {
        self.interface
            .get_mut()
            .write_all(msg)
            .map_err(DriverError::from)
    }
}

/// This type is not Threadsafe
#[derive(Debug)]
pub struct Driver<I: Write + Read> {
    inner: Rc<RefCell<InnerDriver<I>>>,
}

impl<I: Write + Read> Driver<I> {
    pub fn new(interface: I) -> Self {
        Driver {
            inner: Rc::new(RefCell::new(InnerDriver {
                interface: BufReader::new(interface),
                all: HashMap::with_capacity(16),
                motors: HashMap::with_capacity(255),
            })),
        }
    }

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
        // Shouldn't panic since we checked this before
        inner.motors.insert(address, VecDeque::new()).unwrap();
        Ok(Motor::new(self.inner.clone(), address))
    }
}
