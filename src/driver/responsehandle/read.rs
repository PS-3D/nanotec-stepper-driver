use crate::driver::cmd::payload::MotorStatus;

use super::{
    super::{DriverError, InnerDriver},
    ResponseError, ResponseHandle,
};
use std::{cell::RefCell, fmt::Debug, marker::PhantomData, rc::Rc};

//

#[derive(Debug)]
pub(in super::super) struct ReadResponseHandle<T, P>
where
    P: Fn(&[u8]) -> Result<T, DriverError>,
{
    driver: Rc<RefCell<InnerDriver>>,
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
impl<T, P> ReadResponseHandle<T, P>
where
    P: Fn(&[u8]) -> Result<T, DriverError>,
{
    pub fn new(driver: Rc<RefCell<InnerDriver>>, address: u8, parser: P) -> Self {
        Self {
            driver,
            address,
            parser,
            markert: PhantomData,
        }
    }
}

impl<T, P> ResponseHandle for ReadResponseHandle<T, P>
where
    P: Fn(&[u8]) -> Result<T, DriverError>,
{
    type Ret = T;

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

#[derive(Debug)]
pub(in super::super) struct StatusResponseHandle {
    driver: Rc<RefCell<InnerDriver>>,
    address: u8,
}

impl StatusResponseHandle {
    pub fn new(driver: Rc<RefCell<InnerDriver>>, address: u8) -> Self {
        Self { driver, address }
    }
}

impl ResponseHandle for StatusResponseHandle {
    type Ret = MotorStatus;

    fn wait(self) -> Result<MotorStatus, ResponseError<Self, MotorStatus, DriverError>> {
        let mut driver = self.driver.as_ref().borrow_mut();
        match driver.receive_status(self.address) {
            Ok(s) => Ok(s),
            Err(e) => {
                drop(driver);
                Err(ResponseError::from_driver_error(self, e))
            }
        }
    }
}
