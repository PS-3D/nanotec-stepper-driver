#[cfg(test)]
mod tests;

pub mod cmd;
pub mod estop;
mod map;
pub mod motor;
mod parse;
pub mod responsehandle;

use self::{
    cmd::{
        frame::{MotorAddress, Msg, MsgWrap},
        payload::{self, MotorStatus, RespondMode},
    },
    estop::EStop,
    motor::{AllMotor, Motor, NoSendAutoStatus},
    parse::ParseError,
};
use crate::util::ensure;
use nom::Finish;
use serialport::SerialPort;
use std::{
    collections::{HashMap, VecDeque},
    fmt::{Arguments, Debug},
    io::{self, BufRead, BufReader, BufWriter, Write},
    sync::{Arc, Mutex, MutexGuard},
    thread,
    time::Duration,
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
    /// Thrown if a motor with an address the driver didn't expect responded.
    /// That can be because the motor address isn't known to the driver or because
    /// that motor shouldn't send a message right now.
    #[error("motor with address {0} unexpectedly responded")]
    UnexpectedResponse(MotorAddress),
    /// Thrown if a motor that didn't have automatic status sending enabled
    /// sent an automatic status or if it sent a status but already had one
    /// waiting that wasn't fetched
    #[error("motor with address {0} sent an unexpected automatic status")]
    UnexpectedStatus(u8),
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
    /// Wrapper around [`serialport::Error`]
    #[error(transparent)]
    SerialPortError(#[from] serialport::Error),
}

impl From<ParseError<&[u8]>> for DriverError {
    fn from(e: ParseError<&[u8]>) -> Self {
        Self::ParsingError(match e {
            ParseError::InvalidValue => ParseError::InvalidValue,
            ParseError::NonEmptyRemainder => ParseError::NonEmptyRemainder,
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
    send_autostatus: bool,
    autostatus: Option<MotorStatus>,
    respond_mode: RespondMode,
    queue: VecDeque<Vec<u8>>,
}

#[derive(Debug)]
struct InnerDriverData {
    // TODO optimise
    // the u8 basically acts like a semaphore, once it reaches 0 we know we
    // received answers from all motors. it is initialized to the current motor
    // count in send_all
    all: Option<(u8, Vec<u8>)>,
    all_exists: bool,
    motors: HashMap<u8, InnerMotor>,
}

impl InnerDriverData {
    fn drop_motor(&mut self, address: &u8) {
        self.motors.remove(address);
    }

    fn drop_all_motor(&mut self) {
        self.all_exists = false;
    }

    fn is_waiting_all(&self) -> bool {
        self.all.as_ref().map_or(false, |a| a.0 != 0)
    }

    fn handle_status_msg(
        &mut self,
        address: MotorAddress,
        status: MotorStatus,
    ) -> Result<(), DriverError> {
        // having the check for all motors in the handler is suboptimal but the
        // easiest solution for now, maybe we'll change it in the future
        let saddr = address.single_or(DriverError::UnexpectedResponse(address))?;
        let imotor = self
            .motors
            .get_mut(&saddr)
            // if motor address was unknown
            .ok_or(DriverError::UnexpectedResponse(address))?;
        ensure!(
            imotor.send_autostatus && imotor.autostatus.is_none(),
            DriverError::UnexpectedStatus(saddr)
        );
        // ignore since there isn't a value, we checked it above
        // we do it like this instead of using replace to keep the original
        // value if there is one
        let _ = imotor.autostatus.insert(status);
        Ok(())
    }

    fn handle_general_msg(&mut self, msg: Msg) -> Result<(), DriverError> {
        // having the check for all motors in the handler is suboptimal but the
        // easiest solution for now, maybe we'll change it in the future
        let saddr = msg
            .address
            .single_or(DriverError::UnexpectedResponse(msg.address))?;
        ensure!(
            self.all.is_none(),
            DriverError::UnexpectedResponse(msg.address)
        );
        let imotor = self
            .motors
            .get_mut(&saddr)
            .ok_or(DriverError::UnexpectedResponse(msg.address))?;
        ensure!(
            !imotor.available,
            DriverError::UnexpectedResponse(msg.address)
        );
        imotor.available = true;
        imotor.queue.push_back(msg.payload);
        Ok(())
    }
}

// IMPORTANT
// 1. never lock read and data at the same time
//    otherwise the emergency stop will break
// 2. to lock write and data at the same time, first lock data, then write
//    drop them in reverse
struct InnerDriver {
    // TODO make interface also BufWriter so we make less syscalls
    read_interface: Mutex<BufReader<Box<dyn SerialPort>>>,
    write_interface: Mutex<BufWriter<Box<dyn SerialPort>>>,
    data: Mutex<InnerDriverData>,
}

impl InnerDriver {
    // Should only be called by the drop function in Motor
    pub fn drop_motor(&self, address: &u8) {
        let mut data = self.data.lock().unwrap();
        data.drop_motor(address);
    }

    // Should only be called by the drop function in Motor
    pub fn drop_all_motor(&self) {
        let mut data = self.data.lock().unwrap();
        data.drop_all_motor();
    }

    // LOCKS READ
    // -> data needs to be unlocked
    // receives a single msg from the interface and parses it into a Msg
    fn receive_msg(&self) -> Result<Msg, DriverError> {
        // size chosen more or less randomly, should fit most messages
        let mut buf = Vec::with_capacity(64);
        let mut read_interface = self.read_interface.lock().unwrap();
        read_interface.read_until(b'\r', &mut buf)?;
        // there can't be a remainder since we only read till the first '\r'
        let (_, wrap) = MsgWrap::parse(&buf).finish()?;
        match wrap {
            MsgWrap::Valid(msg) => Ok(msg),
            MsgWrap::Invalid(msg) => Err(DriverError::InvalidCmd(msg.payload)),
        }
    }

    // LOCKS DATA and LOCKS READ
    // -> both have to be unlocked
    // TODO write tests
    // receives a general message. if the message returned by receive_msg
    // isnt a general message, e.g. an automatic status, it is handled appropriately
    fn receive_general_msg(&self) -> Result<Msg, DriverError> {
        loop {
            // read gets locked (and unlocked) here
            let msg = self.receive_msg()?;
            // ignore error since error means that parser didn't match and
            // so we have to assume the message wasn't a status message
            if let Some(status) =
                parse::finish_no_remainder(&msg.payload, payload::parse_auto_status_payload).ok()
            {
                let mut data = self.data.lock().unwrap();
                data.handle_status_msg(msg.address, status)?;
            } else {
                return Ok(msg);
            }
        }
    }

    // LOCKS DATA and LOCKS READ
    // -> both have to be unlocked
    fn receive_status_msg(&self) -> Result<(u8, MotorStatus), DriverError> {
        loop {
            // read gets locked (and unlocked) here
            let msg = self.receive_msg()?;
            if let Some(status) =
                parse::finish_no_remainder(&msg.payload, payload::parse_auto_status_payload).ok()
            {
                let saddr = msg
                    .address
                    .single_or(DriverError::UnexpectedResponse(msg.address))?;
                return Ok((saddr, status));
            } else {
                let mut data = self.data.lock().unwrap();
                data.handle_general_msg(msg)?;
            }
        }
    }

    // LOCKS DATA
    pub fn get_respond_mode(&self, address: u8) -> RespondMode {
        let data = self.data.lock().unwrap();
        // shouldn't panic since it's only called from motor, where the address
        // is definitely in the driver
        data.motors.get(&address).unwrap().respond_mode
    }

    // LOCKS DATA
    pub fn set_respond_mode(&self, address: u8, mode: RespondMode) {
        let mut data = self.data.lock().unwrap();
        // shouldn't panic since it's only called from motor, where the address
        // is definitely in the driver
        data.motors.get_mut(&address).unwrap().respond_mode = mode;
    }

    // LOCKS DATA
    pub fn set_respond_mode_all(&self, mode: RespondMode) {
        let mut data = self.data.lock().unwrap();
        data.motors.values_mut().for_each(|m| m.respond_mode = mode);
    }

    // LOCKS DATA and LOCKS READ
    // TODO error out if cmd for other motor is received and that motor wasn't waiting?
    pub fn receive_single(&self, address: u8) -> Result<Vec<u8>, DriverError> {
        // data gets locked and unlocked here
        let qfront = {
            let mut data = self.data.lock().unwrap();
            // shouldn't panic because this shouldn't get called on an address that
            // doesn't already exist
            data.motors.get_mut(&address).unwrap().queue.pop_front()
        };
        // if there's one already in the buffer, return that
        if let Some(p) = qfront {
            Ok(p)
        } else {
            // if we're waiting for a response from a single motors, there shouldn't
            // be a response to an all command so we can massively
            // simplify the receiving process
            loop {
                // read and data get locked and unlocked here
                let msg = self.receive_general_msg()?;
                // data gets locked and unlocked here
                if let MotorAddress::Single(a) = msg.address {
                    let mut data = self.data.lock().unwrap();
                    let imotor = data
                        .motors
                        .get_mut(&a)
                        // if motor address was unknown
                        .ok_or(DriverError::UnexpectedResponse(MotorAddress::Single(a)))?;
                    // FIXME error if motor is already available
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

    pub fn receive_all(&self) -> Result<Vec<u8>, DriverError> {
        if self.data.lock().unwrap().is_waiting_all() {
            // if we're waiting for a response from all motors, there shouldn't
            // be a response from one with a singular address so we can massively
            // simplify the receiving process
            let allcnt = self.data.lock().unwrap().all.as_ref().unwrap().0;
            for _ in 0..allcnt {
                let msg = self.receive_general_msg()?;
                ensure!(
                    msg.address.is_all(),
                    DriverError::UnexpectedResponse(msg.address)
                );
                {
                    let mut data = self.data.lock().unwrap();
                    // if were receiving there should be something in there
                    let all = data.all.as_mut().unwrap();
                    ensure!(
                        msg.payload == all.1,
                        DriverError::NonMatchingPayloads(msg.payload)
                    );
                    all.0 -= 1;
                }
            }
        }
        Ok(self.data.lock().unwrap().all.take().unwrap().1)
    }

    // LOCKS DATA and LOCKS READ
    pub fn receive_status(&self, address: u8) -> Result<MotorStatus, DriverError> {
        // locks and unlocks data
        let autostatus = {
            let mut data = self.data.lock().unwrap();
            // shouldn't panic since this method should only be called with addresses
            // that already exist
            let imotor = data.motors.get_mut(&address).unwrap();
            imotor.autostatus
        };
        if let Some(status) = autostatus {
            Ok(status)
        } else {
            loop {
                // locks and unlocks data and read
                let status = self.receive_status_msg()?;
                if status.0 == address {
                    return Ok(status.1);
                } else {
                    // locks and unlocks data
                    let mut data = self.data.lock().unwrap();
                    // for now we have to wrap the address into a MotorAddress
                    // again, in the future we probably have to make a type for
                    // Msg Payloads
                    // TODO make Msg Payload type
                    data.handle_status_msg(status.0.into(), status.1)?;
                }
            }
        }
    }

    // LOCKS DATA and LOCKS WRITE
    // send_single split in 2 functions cause there are cases like reading values
    // where there will always be a response. this isn't the case with send_all
    // since there only write functions are allowed
    pub fn send_single_no_response(
        &self,
        address: u8,
        args: Arguments<'_>,
    ) -> Result<(), DriverError> {
        let data = self.data.lock().unwrap();
        // if we are waiting for a response from all motors we can't send to
        // a singular motor because we don't know if it's ready yet. only when
        // all motors answered we can be sure
        ensure!(!data.is_waiting_all(), DriverError::NotAvailable);
        // shouldn't panic since the motor has to exist if a message was sent
        // to it
        let imotor = data.motors.get(&address).unwrap();
        ensure!(imotor.available, DriverError::NotAvailable);
        let mut write_interface = self.write_interface.lock().unwrap();
        write!(write_interface, "#{}{}\r", address, args)?;
        write_interface.flush()?;
        Ok(())
    }

    // LOCKS DATA and LOCKS WRITE
    pub fn send_single_with_response(
        &self,
        address: u8,
        args: Arguments<'_>,
    ) -> Result<(), DriverError> {
        // locks and unlocks data and write
        self.send_single_no_response(address, args)?;
        let mut data = self.data.lock().unwrap();
        data.motors.get_mut(&address).unwrap().available = false;
        Ok(())
    }

    // LOCKS DATA and LOCKS WRITE
    pub fn send_all(&self, args: Arguments<'_>) -> Result<RespondMode, DriverError> {
        let mut data = self.data.lock().unwrap();
        // since messages to all motors shouldn't happen that often it can take
        // a bit longer. if it's a problem a semaphore would be a fix for it
        ensure!(
            data.motors.values().all(|m| m.available) && !data.is_waiting_all(),
            DriverError::NotAvailable
        );
        let res_cnt = data
            .motors
            .values()
            .map(|m| m.respond_mode.is_responding() as u8)
            .fold(0, std::ops::Add::add);
        // TODO optimise sending by not copying message 2 times
        // size chosen more or less randomly, should fit most messages
        let iface = &mut self.write_interface.lock().unwrap();
        iface.write_all(b"#*")?;
        // if we gotta wait, we need to save what we sent, otherwise not
        if res_cnt > 0 {
            let mut sent = Vec::with_capacity(64);
            write!(sent, "{}", args)?;
            iface.write_all(&sent)?;
            data.all = Some((res_cnt, sent));
        } else {
            iface.write_fmt(args)?;
        }
        iface.write_all(b"\r")?;
        iface.flush()?;
        if res_cnt == 0 {
            Ok(RespondMode::Quiet)
        } else {
            Ok(RespondMode::NotQuiet)
        }
    }

    // LOCKS WRITE
    // will cause invalid state
    // for now there is no way to recover, so only way is to restart
    pub fn estop(&self, millis: u64) -> Result<(), DriverError> {
        fn send_stop(
            i: &mut MutexGuard<BufWriter<Box<dyn SerialPort>>>,
        ) -> Result<(), DriverError> {
            write!(i, "#*{}0\r", map::STOP_MOTOR)?;
            i.flush()?;
            Ok(())
        }
        let mut write_interface = self.write_interface.lock().unwrap();
        send_stop(&mut write_interface)?;
        for _ in 0..millis {
            thread::sleep(Duration::from_millis(1));
            send_stop(&mut write_interface)?;
        }
        Ok(())
    }
}

impl Debug for InnerDriver {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "InnerDriver {{ read_interface: _, write_interface: _, data: {:?} }}",
            self.data,
        )
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
pub struct Driver {
    inner: Arc<InnerDriver>,
}

impl Driver {
    /// Returns new Driver. `I` is the interface used to actually communicate
    /// with the motors. Usually it's a serialport. Since the motors usually
    /// take a while to reply, especially when they're moving, the timeout of `I`
    /// (if there is one) should be set to something sensible and big.
    ///
    /// # Errors
    /// Be aware that this function will call [`SerialPort::try_clone`], meaning
    /// that the serialport must be cloneable (for example, in linux one can mark
    /// a serial port as exclusive). If this operation fails, a [`DriverError::SerialPortError`]
    /// containing the actual error will be returned.
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
    /// let driver = Driver::new(s).unwrap();
    /// ```
    pub fn new(interface: Box<dyn SerialPort>) -> Result<Self, DriverError> {
        Ok(Driver {
            inner: Arc::new(InnerDriver {
                // wrap into bufreader so receiving until '\r' is easier
                read_interface: Mutex::new(BufReader::new(interface.try_clone()?)),
                write_interface: Mutex::new(BufWriter::new(interface)),
                data: Mutex::new(InnerDriverData {
                    all: None,
                    all_exists: false,
                    // maximium number of motors
                    motors: HashMap::with_capacity(254),
                }),
            }),
        })
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
    /// let mut driver = Driver::new(s).unwrap();
    /// let mut m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    /// ```
    pub fn add_motor(
        &mut self,
        address: u8,
        respond_mode: RespondMode,
    ) -> Result<Motor<NoSendAutoStatus>, DriverError> {
        ensure!(
            address >= 1 && address <= 254,
            DriverError::InvalidAddress(address)
        );
        let inner = &self.inner;
        let mut data = inner.data.lock().unwrap();
        // Have to do it this way due to try_insert being nightly
        ensure!(
            !data.motors.contains_key(&address),
            DriverError::AlreadyExists(MotorAddress::Single(address))
        );
        data.motors
            // chosen more or less randomly, 4 should suffice though
            .insert(
                address,
                InnerMotor {
                    available: true,
                    send_autostatus: false,
                    autostatus: None,
                    respond_mode,
                    queue: VecDeque::with_capacity(4),
                },
            );
        Ok(Motor::new(
            Arc::clone(&self.inner),
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
    /// let mut driver = Driver::new(s).unwrap();
    /// let mut m1 = driver.add_all_motor().unwrap();
    /// ```
    pub fn add_all_motor(&mut self) -> Result<AllMotor<NoSendAutoStatus>, DriverError> {
        let inner = &self.inner;
        let mut data = inner.data.lock().unwrap();
        ensure!(
            !data.all_exists,
            DriverError::AlreadyExists(MotorAddress::All)
        );
        data.all_exists = true;
        Ok(Motor::new(Arc::clone(&self.inner), MotorAddress::All))
    }

    /// Returns a new [`EStop`]
    pub fn new_estop(&self) -> EStop {
        EStop::new(Arc::clone(&self.inner))
    }
}
