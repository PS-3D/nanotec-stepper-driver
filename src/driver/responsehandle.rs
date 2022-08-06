use super::{
    cmd::frame::MotorAddress,
    motor::{AutoStatusMode, Motor},
    DriverError, InnerDriver,
};
use serialport::SerialPort;
use std::{cell::RefCell, error::Error, fmt::Debug, marker::PhantomData, rc::Rc};
use thiserror::Error;

// unfortunately, due to rustfmt not having the blank_lines_upper_bound feature
// stable yet, we gotta put comments in between the different sections. otherwise
// its just too much

//

// Unfortunately we need the fatal error type even tho this is the recoverable
// error, because handle.wait could then in theory throw a fatal error again
/// Holds a recoverable error
///
/// Sometimes an error returned by motor might still be recoverable, for example
/// [`DriverError::UnexpectedResponse`], in which case we need the [`ResponseHandle`]
/// back to call [`wait`][ResponseHandle::wait] again. This type does exactly
/// that.
///
/// For more information see also [`ResponseError`].
#[derive(Error)]
pub struct RecoverableResponseError<H, T, EF, ER>
where
    H: ResponseHandle<T, EF, ER>,
    EF: Error + Into<DriverError>,
    ER: Error + Into<DriverError>,
{
    pub handle: H,
    pub error: ER,
    pub markert: PhantomData<T>,
    pub markeref: PhantomData<EF>,
}

impl<H, T, EF, ER> RecoverableResponseError<H, T, EF, ER>
where
    H: ResponseHandle<T, EF, ER>,
    EF: Error + Into<DriverError>,
    ER: Error + Into<DriverError>,
{
    // maps the handle to anther handle using f
    fn map_handle<OH, OT, OEF, F>(self, f: F) -> RecoverableResponseError<OH, OT, OEF, ER>
    where
        OH: ResponseHandle<OT, OEF, ER>,
        OEF: Error + Into<DriverError>,
        F: FnOnce(H) -> OH,
    {
        RecoverableResponseError {
            handle: f(self.handle),
            error: self.error,
            markert: PhantomData,
            markeref: PhantomData,
        }
    }
}

// have to do it manually because we don't really want ResponseHandle to require
// Debug
impl<H, T, EF, ER> Debug for RecoverableResponseError<H, T, EF, ER>
where
    H: ResponseHandle<T, EF, ER>,
    EF: Error + Into<DriverError>,
    ER: Error + Into<DriverError>,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        Debug::fmt(&self.error, f)
    }
}

impl<H, T, EF, ER> From<RecoverableResponseError<H, T, EF, ER>> for DriverError
where
    H: ResponseHandle<T, EF, ER>,
    EF: Error + Into<DriverError>,
    ER: Error + Into<DriverError>,
{
    fn from(e: RecoverableResponseError<H, T, EF, ER>) -> Self {
        e.error.into()
    }
}

//

// Needs different types for fatal and recoverable errors because, depending on the
// response handle because in some handles (e.g. MotorMappingResponseHandle) a
// fatal error might need to return additional information, which a recoverable
// error might not need because it is still recoverable (in the case of
// MotorMappingResponseHandle a fatal error needs to return the original motor)
//
// In theory the recoverable error could also be DriverError instead of generic
// but maybe we need it in the future.
/// Error that contains ResponseHandle if error is recoverable.
///
/// This error is usually returned by [`ResponseHandle::wait`], to differentiate
/// erors that can still be recovered from ones that are fatal.
///
/// Some DriverErrors are still recoverable, meaning the handle could still exist
/// and wait could be called again. For example for
/// [`DriverError::UnexpectedResponse`], we can still continue to wait for the
/// actual result, since a reply from an unexpected motor isn't really a reason
/// not to continue waiting, but we still might want to handle that type of error.
/// For that, [`RecoverableResponseError`] contains another ResponseHandle that
/// waits for the result, as well as the original error.
///
/// In some cases the fatal error needs to be a different type than the recoverable
/// error, for example when calling [`Motor::start_sending_auto_status`], where
/// the original motor needs to be returned on a fatal error.
#[derive(Error)]
pub enum ResponseError<H, T, EF, ER = DriverError>
where
    H: ResponseHandle<T, EF, ER>,
    EF: Error + Into<DriverError>,
    ER: Error + Into<DriverError>,
{
    #[error(transparent)]
    Fatal(EF),
    #[error(transparent)]
    Recoverable(RecoverableResponseError<H, T, EF, ER>),
}

impl<H, T, EF, ER> ResponseError<H, T, EF, ER>
where
    H: ResponseHandle<T, EF, ER>,
    EF: Error + Into<DriverError>,
    ER: Error + Into<DriverError>,
{
    // maps the handle of RecoverableResponseERror to another handle using f
    fn map_handle<OH, OT, F>(self, f: F) -> ResponseError<OH, OT, EF, ER>
    where
        OH: ResponseHandle<OT, EF, ER>,
        F: FnOnce(H) -> OH,
    {
        match self {
            Self::Fatal(e) => ResponseError::Fatal(e),
            Self::Recoverable(e) => ResponseError::Recoverable(e.map_handle(f)),
        }
    }

    /// Returns `true` if the error is fatal
    pub fn is_fatal(&self) -> bool {
        match self {
            Self::Fatal(_) => true,
            _ => false,
        }
    }

    /// Returns `true` if the error is recoverable
    pub fn is_recoverable(&self) -> bool {
        match self {
            Self::Recoverable(_) => true,
            _ => false,
        }
    }

    /// Returns the fatal error, if this error is fatal.
    ///
    /// # Panics
    /// If the error is not fatal
    pub fn fatal(self) -> EF {
        match self {
            Self::Fatal(e) => e,
            _ => panic!("fatal() was called on a non-fatal error"),
        }
    }

    /// Returns the fatal recoverable, if this error is recoverable.
    ///
    /// # Panics
    /// If the error is not recoverable
    pub fn recoverable(self) -> RecoverableResponseError<H, T, EF, ER> {
        match self {
            ResponseError::Recoverable(e) => e,
            _ => panic!("recoverable() was called on a non-recoverable error"),
        }
    }
}

impl<H: ResponseHandle<T>, T> ResponseError<H, T, DriverError> {
    fn from_driver_error(handle: H, error: DriverError) -> Self {
        let error = match error {
            DriverError::UnexpectedResponse(r) => DriverError::UnexpectedResponse(r),
            _ => return Self::Fatal(error),
        };
        Self::Recoverable(RecoverableResponseError {
            handle,
            error,
            markert: PhantomData,
            markeref: PhantomData,
        })
    }
}

// have to implement debug manually, somehow derive fucks it up
impl<H, T, EF, ER> Debug for ResponseError<H, T, EF, ER>
where
    H: ResponseHandle<T, EF, ER>,
    EF: Error + Into<DriverError>,
    ER: Error + Into<DriverError>,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ResponseError::")?;
        match self {
            ResponseError::Fatal(e) => {
                write!(f, "Fatal(")?;
                Debug::fmt(e, f)?;
            }
            ResponseError::Recoverable(e) => {
                write!(f, "Recoverable(")?;
                Debug::fmt(e, f)?;
            }
        }
        write!(f, ")")
    }
}

impl<H, T, EF, ER> From<ResponseError<H, T, EF, ER>> for DriverError
where
    H: ResponseHandle<T, EF, ER>,
    EF: Error + Into<DriverError>,
    ER: Error + Into<DriverError>,
{
    fn from(e: ResponseError<H, T, EF, ER>) -> Self {
        match e {
            ResponseError::Fatal(f) => f.into(),
            ResponseError::Recoverable(r) => r.into(),
        }
    }
}

// TODO maybe some sort of ignore method for Result<T, ResponseError> that loops
// until it succeeds or a fatal error occurs

//

// for explanation on the two errors see ResponseError and maybe also
// MotorMapResponseHandle
/// Handle to wait for a motor response
pub trait ResponseHandle<T, EF = DriverError, ER = DriverError>
where
    EF: Error + Into<DriverError>,
    ER: Error + Into<DriverError>,
{
    /// Check the response for correctness and return the result contained in the
    /// response, if there was one. Usually only getters have a returnvalue. If
    /// the response only needs to be checked for correctness but no value is
    /// contained in it, wait only checks for correctness. This function blocks
    /// until a response is received.
    ///
    /// # Errors
    ///
    /// Some errors are recoverable, meaning that they don't affect the waiting
    /// on the current response, but need to be reported and maybe handled
    /// nevertheless. For that reason [`ResponseError`] is returned, which can
    /// distinguish between recoverable and fatal errors. Recoverable errors return
    /// the error and the responsehandle again, making it possible to handle the
    /// error and then continue waiting.
    ///
    /// ## Example:
    /// ```no_run
    /// # use nanotec_stepper_driver::{Driver, ResponseHandle, RespondMode, ResponseError};
    /// use std::time::Duration;
    /// use serialport;
    ///
    /// let s = serialport::new("/dev/ttyUSB0", 115200)
    ///     .timeout(Duration::from_secs(1))
    ///     .open_native()
    ///     .unwrap();
    /// let mut driver = Driver::new(s);
    /// let mut m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    ///
    /// let mut handle = m1.start_motor().unwrap();
    /// println!("started motor");
    /// while let Err(e) = handle.wait() {
    ///     if let ResponseError::Recoverable(r) = e {
    ///         println!("Recoverable error occured: {}", r.error);
    ///         handle = r.handle;
    ///     } else {
    ///         println!("error was fatal!");
    ///         break;
    ///     }
    /// }
    /// ```
    ///
    /// Usually returns:
    /// * [`DriverError::NonMatchingPayloads`] if the response didn't
    /// match what was expected
    /// * [`DriverError::ParsingError`] if an error occured while parsing (this
    /// usually also means that the response didn't match what was expected)
    /// * [`DriverError::IoError`] if an IO error occurs
    /// * [`DriverError::UnexpectedResponse`] if an unkown motor responds
    ///
    /// # Examples
    /// ```no_run
    /// # use nanotec_stepper_driver::{Driver, ResponseHandle, RespondMode};
    /// use std::time::Duration;
    /// use serialport;
    ///
    /// let s = serialport::new("/dev/ttyUSB0", 115200)
    ///     .timeout(Duration::from_secs(1))
    ///     .open_native()
    ///     .unwrap();
    /// let mut driver = Driver::new(s);
    /// let mut m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    ///
    /// let handle = m1.start_motor().unwrap();
    /// println!("started motor");
    /// handle.wait().unwrap();
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
    ///     .open_native()
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
    /// ```
    fn wait(self) -> Result<T, ResponseError<Self, T, EF, ER>>
    where
        Self: Sized;
}

//

#[derive(Debug)]
pub(super) struct ReadResponseHandle<I: SerialPort, T, P>
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
impl<I: SerialPort, T, P> ReadResponseHandle<I, T, P>
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

impl<I: SerialPort, T, P> ResponseHandle<T> for ReadResponseHandle<I, T, P>
where
    P: Fn(&[u8]) -> Result<T, DriverError>,
{
    // the whole match error and drop shenannigans are needed to statisfy the
    // borrow checker
    fn wait(self) -> Result<T, ResponseError<Self, T, DriverError>> {
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
pub(super) struct WriteResponseHandle<I: SerialPort> {
    driver: Rc<RefCell<InnerDriver<I>>>,
    address: MotorAddress,
    // payload we sent
    sent: Vec<u8>,
}

impl<I: SerialPort> WriteResponseHandle<I> {
    pub fn new(driver: Rc<RefCell<InnerDriver<I>>>, address: MotorAddress, sent: Vec<u8>) -> Self {
        Self {
            driver,
            address,
            sent,
        }
    }
}

impl<I: SerialPort> ResponseHandle<()> for WriteResponseHandle<I> {
    // the whole match error and drop shenannigans are needed to statisfy the
    // borrow checker
    fn wait(self) -> Result<(), ResponseError<Self, (), DriverError>> {
        let mut driver = self.driver.as_ref().borrow_mut();
        let payload = match self.address {
            MotorAddress::Single(a) => driver.receive_single(a),
            MotorAddress::All => driver.receive_all(),
        };
        // unfortunately we can't just map the error because we have to
        // drop the driver before we can return the error
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
    fn wait(self) -> Result<(), ResponseError<Self, (), DriverError>> {
        Ok(())
    }
}

//

// Wrapper so we can return Write and Dummy, read is not needed since read
// commands always return a value
//
// In theory doing this would also be possible by making 2 different implementations
// for Motor, like with AutoStatusMode, but this would only be
// a lot of work, since the automatic status sending affects only a few methods
// while whether or not the motor answers affects each and every command.
// (Tho in the api we limit it to write commands only, since read commands without
// an answer would be sortof stupid)
#[derive(Debug)]
pub(super) enum WrapperResponseHandle<I: SerialPort> {
    Write(WriteResponseHandle<I>),
    Dummy(DummyResponseHandle),
}

impl<I: SerialPort> ResponseHandle<()> for WrapperResponseHandle<I> {
    fn wait(self) -> Result<(), ResponseError<Self, (), DriverError>> {
        use WrapperResponseHandle::*;
        // the maps are required to map from ResponseError<WriteResponseHandle> or
        // ResponseError<DummyResponseHandle> to ResponseError<WrapperResponseHandle>
        match self {
            Write(h) => h.wait().map_err(|e| e.map_handle(|h| Write(h))),
            Dummy(h) => h.wait().map_err(|e| e.map_handle(|h| Dummy(h))),
        }
    }
}

//

// more or less just a helper error to be able to return a motor and
// a drivererror in case the writeresponsehandle in motormappingresponsehandle
// fails fatally. This is so the Motor can still be used, since there's no reason
// why it shouldn't be usable.
/// Returned by [`ResponseHandle::wait`] if the error of a motor mapping command
/// was fatal
///
/// This means that the operation didn't succeed and there is no way to recover
/// from it. This error holds the actual error that occured as well as the original
/// motor, not the one with the new type, since it's still usable. Such a
/// command would be [`Motor::start_sending_auto_status`].
#[derive(Error)]
#[error("{}", .1)]
pub struct MotorMappingError<I: SerialPort, AS: AutoStatusMode>(
    pub Motor<I, AS>,
    #[source] pub DriverError,
);

impl<I: SerialPort, AS: AutoStatusMode> From<MotorMappingError<I, AS>> for DriverError {
    fn from(e: MotorMappingError<I, AS>) -> Self {
        e.1
    }
}

// have to implement manually because we can't print a debug repr of Motor<I, AS>
// if we don't restrict SerialPort and we don't really want to do that
impl<I: SerialPort, AS: AutoStatusMode> Debug for MotorMappingError<I, AS> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "MotorMappingError(_, {:?})", self.1)
    }
}

//

// responsehandle for mapping a whole motor
// it uses an underlying writeresponsehandle in order to set the sending
// of various things, for example automatically sending the status.
// if the writeresponsehandle fails and is fatal, we return the original motor so
// the motor can continue to be used and the operation maybe tried again (there
// is no reason why it shouldn't be usable).
// If it fails and is recoverable, we return a more or less copy of this handle
// so wait can be called again and the operation might succed after all.
// If it succeds we apply the mapping function and return the new motor.
#[derive(Debug)]
pub(super) struct MotorMappingResponseHandle<I, ASI, ASO, F>
where
    I: SerialPort,
    ASI: AutoStatusMode + Debug,
    ASO: AutoStatusMode + Debug,
    F: FnOnce(Motor<I, ASI>) -> Motor<I, ASO>,
{
    motor: Motor<I, ASI>,
    handle: WrapperResponseHandle<I>,
    mapper: F,
    markermo: PhantomData<ASO>,
}

impl<I, ASI, ASO, F> MotorMappingResponseHandle<I, ASI, ASO, F>
where
    I: SerialPort,
    ASI: AutoStatusMode + Debug,
    ASO: AutoStatusMode + Debug,
    F: FnOnce(Motor<I, ASI>) -> Motor<I, ASO>,
{
    pub(super) fn new(motor: Motor<I, ASI>, handle: WrapperResponseHandle<I>, f: F) -> Self {
        Self {
            motor,
            handle,
            mapper: f,
            markermo: PhantomData,
        }
    }
}

impl<I, ASI, ASO, F> ResponseHandle<Motor<I, ASO>, MotorMappingError<I, ASI>>
    for MotorMappingResponseHandle<I, ASI, ASO, F>
where
    I: SerialPort,
    ASI: AutoStatusMode + Debug,
    ASO: AutoStatusMode + Debug,
    F: FnOnce(Motor<I, ASI>) -> Motor<I, ASO>,
{
    fn wait(
        self,
    ) -> Result<Motor<I, ASO>, ResponseError<Self, Motor<I, ASO>, MotorMappingError<I, ASI>>>
    where
        Self: Sized,
    {
        // we have to do it like this instead of mapping, otherwise we will move
        // self.motor twice, once for fatal and once for recoverable
        if let Err(e) = self.handle.wait() {
            return Err(match e {
                ResponseError::Fatal(f) => ResponseError::Fatal(MotorMappingError(self.motor, f)),
                ResponseError::Recoverable(r) => {
                    ResponseError::Recoverable(r.map_handle(|h| Self {
                        motor: self.motor,
                        handle: h,
                        mapper: self.mapper,
                        markermo: PhantomData,
                    }))
                }
            });
        }
        Ok((self.mapper)(self.motor))
    }
}
