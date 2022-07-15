use super::{cmd::frame::MotorAddress, DriverError, InnerDriver};
use std::{
    cell::RefCell,
    fmt::Debug,
    io::{Read, Write},
    marker::PhantomData,
    rc::Rc,
};
use thiserror::Error;

// unfortunately, due to rustfmt not having the blank_lines_upper_bound feature
// stable yet, we gotta put comments in between the different sections. otherwise
// its just too much

//

/// Holds a recoverable DriverError
#[derive(Error)]
pub struct RecoverableResponseError<H: ResponseHandle<T>, T> {
    pub handle: H,
    pub error: DriverError,
    pub marker: PhantomData<T>,
}

impl<H: ResponseHandle<T>, T> RecoverableResponseError<H, T> {
    // maps the handle to anther handle using f
    fn map_handle<O, F>(self, f: F) -> RecoverableResponseError<O, T>
    where
        F: FnOnce(H) -> O,
        O: ResponseHandle<T>,
    {
        RecoverableResponseError {
            handle: f(self.handle),
            error: self.error,
            marker: PhantomData,
        }
    }
}

// have to do it manually because we don't really want ResponseHandle to require
// Debug
impl<H: ResponseHandle<T>, T> Debug for RecoverableResponseError<H, T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        Debug::fmt(&self.error, f)
    }
}

//

/// Error that contains ResponseHandle if error is recoverable.
///
/// Some DriverErrors are still recoverable. For example for
/// [`DriverError::UnexpectedResponse`], we can still continue to wait for the
/// actual result. For that, [`RecoverableResponseError`] contains another
/// ResponseHandle that waits for the result, as well as the original error.
#[derive(Error)]
pub enum ResponseError<H: ResponseHandle<T>, T> {
    #[error(transparent)]
    Recoverable(RecoverableResponseError<H, T>),
    #[error(transparent)]
    Fatal(DriverError),
}

impl<H: ResponseHandle<T>, T> ResponseError<H, T> {
    fn from_driver_error(handle: H, error: DriverError) -> Self {
        let error = match error {
            DriverError::UnexpectedResponse(r) => DriverError::UnexpectedResponse(r),
            _ => return Self::Fatal(error),
        };
        Self::Recoverable(RecoverableResponseError {
            handle,
            error,
            marker: PhantomData,
        })
    }

    // maps the handle of RecoverableResponseERror to another handle using f
    fn map_handle<O, F>(self, f: F) -> ResponseError<O, T>
    where
        F: FnOnce(H) -> O,
        O: ResponseHandle<T>,
    {
        match self {
            ResponseError::Recoverable(e) => ResponseError::Recoverable(e.map_handle(f)),
            ResponseError::Fatal(e) => ResponseError::Fatal(e),
        }
    }
}

// have to do it manually, somehow derive fucks it up
impl<H: ResponseHandle<T>, T> Debug for ResponseError<H, T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ResponseError::Recoverable(e) => Debug::fmt(e, f),
            ResponseError::Fatal(e) => Debug::fmt(e, f),
        }
    }
}

// TODO maybe some sort of ignore method for Result<T, ResponseError> that loops
// until it succeeds or a fatal error occurs

//

/// Handle to wait for a motor response
pub trait ResponseHandle<T> {
    /// Check the response for correctness and return the result contained in the
    /// response, if there was one. Usually only getters have a returnvalue. If
    /// the response only needs to be checked for correctness but no value is
    /// contained in it, wait only checks for correctness. This function blocks
    /// until a response is received.
    ///
    /// # Errors
    /// Usually returns [`DriverError::NonMatchingPayloads`] if the response didn't
    /// match what was expected or a [`DriverError::ParsingError`].
    /// If an IO error occured, [`DriverError::IoError`] is returned.
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
    /// let handle = m1.start_motor().unwrap();
    /// println!("started motor");
    /// handle.wait().unwrap();
    /// println!("motor stopped");
    /// ```
    /// A [`ResponseHandle`] can also be used to send commands to 2 motors at
    /// once:
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
    fn wait(self) -> Result<T, ResponseError<Self, T>>
    where
        Self: Sized;
}

//

#[derive(Debug)]
pub(super) struct ReadResponseHandle<I: Write + Read, T, P>
where
    P: Fn(&[u8]) -> Result<T, DriverError>,
{
    driver: Rc<RefCell<InnerDriver<I>>>,
    address: u8,
    // should parse the payload of the message (command without #<address> and \r)
    // and return a T from it
    parser: P,
    markert: PhantomData<T>,
}

// Implementation for read commands
// implementations for read and write were split so we don't need to parse as much
// since for write commands we only need to check if the payload matched what we
// sent and it also makes WriteResponseHandle::wait a bit faster
impl<I: Write + Read, T, P> ReadResponseHandle<I, T, P>
where
    P: Fn(&[u8]) -> Result<T, DriverError>,
{
    pub fn new(driver: Rc<RefCell<InnerDriver<I>>>, address: u8, parser: P) -> Self {
        Self {
            driver,
            address,
            parser,
            markert: PhantomData,
        }
    }
}

impl<I: Write + Read, T, P> ResponseHandle<T> for ReadResponseHandle<I, T, P>
where
    P: Fn(&[u8]) -> Result<T, DriverError>,
{
    // the whole match error and drop shenannigans are needed to statisfy the
    // borrow checker
    fn wait(self) -> Result<T, ResponseError<Self, T>> {
        let mut driver = self.driver.as_ref().borrow_mut();
        let payload = match driver.receive_single(self.address) {
            Ok(p) => p,
            Err(e) => {
                drop(driver);
                return Err(ResponseError::from_driver_error(self, e));
            }
        };
        match (self.parser)(&payload) {
            Ok(p) => Ok(p),
            Err(e) => {
                drop(driver);
                Err(ResponseError::from_driver_error(self, e))
            }
        }
    }
}

//

// Implementation for write commands
// implementations for read and write were split so we don't need to parse as much
// since for write commands we only need to check if the payload matched what we
// sent and it also makes WriteResponseHandle::wait a bit faster
#[derive(Debug)]
pub(super) struct WriteResponseHandle<I: Write + Read> {
    driver: Rc<RefCell<InnerDriver<I>>>,
    address: MotorAddress,
    // payload we sent
    sent: Vec<u8>,
}

impl<I: Write + Read> WriteResponseHandle<I> {
    pub fn new(driver: Rc<RefCell<InnerDriver<I>>>, address: MotorAddress, sent: Vec<u8>) -> Self {
        Self {
            driver,
            address,
            sent,
        }
    }
}

impl<I: Write + Read> ResponseHandle<()> for WriteResponseHandle<I> {
    // the whole match error and drop shenannigans are needed to statisfy the
    // borrow checker
    fn wait(self) -> Result<(), ResponseError<Self, ()>> {
        let mut driver = self.driver.as_ref().borrow_mut();
        let payload = match self.address {
            MotorAddress::Single(a) => driver.receive_single(a),
            MotorAddress::All => driver.receive_all(),
        };
        let payload = match payload {
            Ok(p) => p,
            Err(e) => {
                drop(driver);
                return Err(ResponseError::from_driver_error(self, e));
            }
        };
        if self.sent == payload {
            Ok(())
        } else {
            drop(driver);
            Err(ResponseError::from_driver_error(
                self,
                DriverError::NonMatchingPayloads(payload),
            ))
        }
    }
}

//

// Responsehandle for motors with RespondMode::Quiet
#[derive(Debug)]
pub(super) struct DummyResponseHandle();

impl DummyResponseHandle {
    pub fn new() -> Self {
        Self()
    }
}

impl ResponseHandle<()> for DummyResponseHandle {
    fn wait(self) -> Result<(), ResponseError<Self, ()>> {
        Ok(())
    }
}

//

// Wrapper so we can return Write and Dummy, read is not needed since read
// commands always return a value
#[derive(Debug)]
pub(super) enum WrapperResponseHandle<I: Read + Write> {
    Write(WriteResponseHandle<I>),
    Dummy(DummyResponseHandle),
}

impl<I: Write + Read> ResponseHandle<()> for WrapperResponseHandle<I> {
    fn wait(self) -> Result<(), ResponseError<Self, ()>> {
        use WrapperResponseHandle::*;
        // the maps are required to map from ResponseError<WriteResponseHandle> or
        // ResponseError<DummyResponseHandle> to ResponseError<WrapperResponseHandle>
        match self {
            Write(h) => h.wait().map_err(|e| e.map_handle(|h| Write(h))),
            Dummy(h) => h.wait().map_err(|e| e.map_handle(|h| Dummy(h))),
        }
    }
}
