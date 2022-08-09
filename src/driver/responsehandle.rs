pub(crate) mod map;
pub(crate) mod read;
pub(crate) mod write;

use super::DriverError;
use std::{error::Error, fmt::Debug, marker::PhantomData};
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
    H: ResponseHandle<EF, ER, Ret = T>,
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
    H: ResponseHandle<EF, ER, Ret = T>,
    EF: Error + Into<DriverError>,
    ER: Error + Into<DriverError>,
{
    // maps the handle to anther handle using f
    fn map_handle<OH, OT, OEF, F>(self, f: F) -> RecoverableResponseError<OH, OT, OEF, ER>
    where
        OH: ResponseHandle<OEF, ER, Ret = OT>,
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
    H: ResponseHandle<EF, ER, Ret = T>,
    EF: Error + Into<DriverError>,
    ER: Error + Into<DriverError>,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        Debug::fmt(&self.error, f)
    }
}

impl<H, T, EF, ER> From<RecoverableResponseError<H, T, EF, ER>> for DriverError
where
    H: ResponseHandle<EF, ER, Ret = T>,
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
/// error, for example when calling [`Motor::start_sending_auto_status`][super::motor::Motor::start_sending_auto_status],
/// where the original motor needs to be returned on a fatal error.
#[derive(Error)]
pub enum ResponseError<H, T, EF, ER = DriverError>
where
    H: ResponseHandle<EF, ER, Ret = T>,
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
    H: ResponseHandle<EF, ER, Ret = T>,
    EF: Error + Into<DriverError>,
    ER: Error + Into<DriverError>,
{
    // maps the handle of RecoverableResponseERror to another handle using f
    fn map_handle<OH, OT, F>(self, f: F) -> ResponseError<OH, OT, EF, ER>
    where
        OH: ResponseHandle<EF, ER, Ret = OT>,
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

impl<H: ResponseHandle<Ret = T>, T> ResponseError<H, T, DriverError> {
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
    H: ResponseHandle<EF, ER, Ret = T>,
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
    H: ResponseHandle<EF, ER, Ret = T>,
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

// FIXME some sort of drop impl that the receiving of messages wont break
// if responsehandle is dropped. An intermediate fix could be to panic
// if responsehandle gets dropped or something similar
//
// for explanation on the two errors see ResponseError and maybe also
// MotorMapResponseHandle
/// Handle to wait for a motor response
pub trait ResponseHandle<EF = DriverError, ER = DriverError>
where
    EF: Error + Into<DriverError>,
    ER: Error + Into<DriverError>,
{
    type Ret;

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
    fn wait(self) -> Result<Self::Ret, ResponseError<Self, Self::Ret, EF, ER>>
    where
        Self: Sized;
}
