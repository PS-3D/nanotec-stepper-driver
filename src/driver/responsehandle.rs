use super::{DriverError, InnerDriver};
use std::{
    cell::RefCell,
    io::{Read, Write},
    marker::PhantomData,
    rc::Rc,
};

pub trait ResponseHandle<T> {
    fn wait(self) -> Result<T, DriverError>;
}

pub(super) struct ReadResponseHandle<I: Write + Read, T, P>
where
    P: Fn(&[u8]) -> Result<T, DriverError>,
{
    driver: Rc<RefCell<InnerDriver<I>>>,
    address: u8,
    parser: P,
    markert: PhantomData<T>,
}

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
    fn wait(self) -> Result<T, DriverError> {
        let mut driver = self.driver.as_ref().borrow_mut();
        let payload = match driver.check_msg(self.address) {
            Some(p) => p,
            None => driver.receive(self.address)?,
        };
        (self.parser)(&payload)
    }
}

pub(super) struct WriteResponseHandle<I: Write + Read> {
    driver: Rc<RefCell<InnerDriver<I>>>,
    address: u8,
    sent: Vec<u8>,
}

impl<I: Write + Read> WriteResponseHandle<I> {
    pub fn new(driver: Rc<RefCell<InnerDriver<I>>>, address: u8, sent: Vec<u8>) -> Self {
        Self {
            driver,
            address,
            sent,
        }
    }
}

impl<I: Write + Read> ResponseHandle<()> for WriteResponseHandle<I> {
    fn wait(self) -> Result<(), DriverError> {
        let mut driver = self.driver.as_ref().borrow_mut();
        let payload = match driver.check_msg(self.address) {
            Some(p) => p,
            None => driver.receive(self.address)?,
        };
        if self.sent == payload {
            Ok(())
        } else {
            Err(DriverError::NonMatchingPayloads(payload))
        }
    }
}
