use super::{
    super::{
        cmd::frame::MotorAddress,
        motor::{AutoStatusMode, Motor},
        DriverError, InnerDriver,
    },
    ResponseError, ResponseHandle,
};
use std::{fmt::Debug, marker::PhantomData, sync::Arc};
use thiserror::Error;

//

// Implementation for write commands
// implementations for read and write were split so we don't need to parse as much
// since for write commands we only need to check if the payload matched what we
// sent and it also makes WriteResponseHandle::wait a bit faster
#[derive(Debug)]
pub(in super::super) struct WriteResponseHandle {
    driver: Arc<InnerDriver>,
    address: MotorAddress,
    // payload we sent
    sent: Vec<u8>,
}

impl WriteResponseHandle {
    pub fn new(driver: Arc<InnerDriver>, address: MotorAddress, sent: Vec<u8>) -> Self {
        Self {
            driver,
            address,
            sent,
        }
    }
}

impl ResponseHandle for WriteResponseHandle {
    type Ret = ();

    // the whole match error and drop shenannigans are needed to statisfy the
    // borrow checker
    fn wait(self) -> Result<(), ResponseError<Self, (), DriverError>> {
        let driver = &self.driver;
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
pub(in super::super) struct DummyResponseHandle();

impl DummyResponseHandle {
    pub fn new() -> Self {
        Self()
    }
}

impl ResponseHandle for DummyResponseHandle {
    type Ret = ();

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
pub(in super::super) enum WrapperResponseHandle {
    Write(WriteResponseHandle),
    Dummy(DummyResponseHandle),
}

impl ResponseHandle for WrapperResponseHandle {
    type Ret = ();

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
pub struct MotorMappingError<AS: AutoStatusMode>(pub Motor<AS>, #[source] pub DriverError);

impl<AS: AutoStatusMode> From<MotorMappingError<AS>> for DriverError {
    fn from(e: MotorMappingError<AS>) -> Self {
        e.1
    }
}

// have to implement manually because we can't print a debug repr of Motor<I, AS>
// if we don't restrict SerialPort and we don't really want to do that
impl<AS: AutoStatusMode> Debug for MotorMappingError<AS> {
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
pub(in super::super) struct MotorMappingResponseHandle<ASI, ASO, F>
where
    ASI: AutoStatusMode + Debug,
    ASO: AutoStatusMode + Debug,
    F: FnOnce(Motor<ASI>) -> Motor<ASO>,
{
    motor: Motor<ASI>,
    handle: WrapperResponseHandle,
    mapper: F,
    markermo: PhantomData<ASO>,
}

impl<ASI, ASO, F> MotorMappingResponseHandle<ASI, ASO, F>
where
    ASI: AutoStatusMode + Debug,
    ASO: AutoStatusMode + Debug,
    F: FnOnce(Motor<ASI>) -> Motor<ASO>,
{
    pub fn new(motor: Motor<ASI>, handle: WrapperResponseHandle, f: F) -> Self {
        Self {
            motor,
            handle,
            mapper: f,
            markermo: PhantomData,
        }
    }
}

impl<ASI, ASO, F> ResponseHandle<MotorMappingError<ASI>> for MotorMappingResponseHandle<ASI, ASO, F>
where
    ASI: AutoStatusMode + Debug,
    ASO: AutoStatusMode + Debug,
    F: FnOnce(Motor<ASI>) -> Motor<ASO>,
{
    type Ret = Motor<ASO>;

    fn wait(self) -> Result<Motor<ASO>, ResponseError<Self, Motor<ASO>, MotorMappingError<ASI>>>
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
