use super::{DriverError, InnerDriver};
use std::{
    cell::RefCell,
    io::{Read, Write},
    marker::PhantomData,
    rc::Rc,
};

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
    /// # use nanotec_stepper_driver::{Driver, ResponseHandle};
    /// use std::time::Duration;
    /// use serialport;
    ///
    /// let s = serialport::new("/dev/ttyUSB0", 115200)
    ///     .timeout(Duration::from_secs(1))
    ///     .open()
    ///     .unwrap();
    /// let mut driver = Driver::new(s);
    /// let mut m1 = driver.add_motor(1).unwrap();
    ///
    /// let handle = m1.start_motor().unwrap();
    /// println!("started motor");
    /// handle.wait().unwrap();
    /// println!("motor stopped");
    /// ```
    /// A [`ResponseHandle`] can also be used to send commands to 2 motors at
    /// once:
    /// ```no_run
    /// # use nanotec_stepper_driver::{Driver, ResponseHandle};
    /// use std::time::Duration;
    /// use serialport;
    ///
    /// let s = serialport::new("/dev/ttyUSB0", 115200)
    ///     .timeout(Duration::from_secs(1))
    ///     .open()
    ///     .unwrap();
    /// let mut driver = Driver::new(s);
    /// let mut m1 = driver.add_motor(1).unwrap();
    /// let mut m2 = driver.add_motor(2).unwrap();
    ///
    /// let handle1 = m1.start_motor().unwrap();
    /// let handle2 = m2.start_motor().unwrap();
    /// println!("started motors");
    /// handle1.wait().unwrap();
    /// handle2.wait().unwrap();
    /// println!("motors stopped");
    /// ```
    fn wait(self) -> Result<T, DriverError>;
}

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
    fn wait(self) -> Result<T, DriverError> {
        let mut driver = self.driver.as_ref().borrow_mut();
        let payload = driver.receive_single(self.address)?;
        (self.parser)(&payload)
    }
}

// Implementation for write commands
// implementations for read and write were split so we don't need to parse as much
// since for write commands we only need to check if the payload matched what we
// sent and it also makes WriteResponseHandle::wait a bit faster
pub(super) struct WriteResponseHandle<I: Write + Read> {
    driver: Rc<RefCell<InnerDriver<I>>>,
    address: Option<u8>,
    // payload we sent
    sent: Vec<u8>,
}

impl<I: Write + Read> WriteResponseHandle<I> {
    pub fn new(driver: Rc<RefCell<InnerDriver<I>>>, address: Option<u8>, sent: Vec<u8>) -> Self {
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
        let payload = match self.address {
            Some(a) => driver.receive_single(a)?,
            None => driver.receive_all()?,
        };
        if self.sent == payload {
            Ok(())
        } else {
            Err(DriverError::NonMatchingPayloads(payload))
        }
    }
}
