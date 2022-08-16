use super::{DriverError, InnerDriver};
use std::sync::Arc;

// in theory it should be possible to not use mutexes and instead clone
// serialport again and wrap in in a BufWriter. that would/could in theory only
// yield one actual write per message which should then not result 2 threads
// writing at once. this might probably not be the best idea tho since BufWriter
// does afaik not make any guarantees about flushing
/// "Non-emergency emergency" stop
#[derive(Debug)]
pub struct EStop(Arc<InnerDriver>);

impl EStop {
    pub(super) fn new(driver: Arc<InnerDriver>) -> Self {
        EStop(driver)
    }

    /// Stops all motors in a "non-emergency emergency"
    ///
    /// This function sends a stop command with the quickstop ramp to all motors,
    /// stopping them (see also [1.6.2 Stopping a motor](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)).
    /// To achieve this the command is just sent every millisecond for `millis`
    /// milliseconds. At first this might seem like a stupid and unelegant solution
    /// and in a way it sortof is. But consider that doing it this way is dead-simple
    /// and short from an IO error or another thread holding and not releasing the
    /// lock on the internal write-mutex there is not really mutch to go wrong.
    /// If `millis` is chosen big enough there is no reason why any motor should
    /// not stop (any value than 1000 should *probably* be fine). If it won't stop
    /// then, it won't stop any other way anyways.\
    /// Nevertheless:
    ///
    /// ***DO NOT USE THIS AS AN ACTUAL EMERGENCY STOP***
    ///
    /// Things go wrong. Software has bugs. Hardware breaks. Both can be used
    /// incorrectly. If you really need things to stop, you should consider another
    /// method anyways, no matter how good this implementation is. This mostly
    /// exists because it is less violent than cutting the power or attaching
    /// a brake to the motor to overpower it.\
    /// Additionally, this would never statisfy most safety standards anyways.
    ///
    /// After calling this function, the internal state of the driver is invalid.
    /// At the moment there is no way to softly reset the state, so you more or
    /// less need to drop the driver and everything associated with it and replace
    /// it with a new one.
    ///
    /// # Errors
    /// Even though this throws a DriverError, the only actual error than could
    /// be thrown is a [`DriverError::IoError`].
    ///
    /// # Example
    /// ```no_run
    /// # use nanotec_stepper_driver::{Driver, ResponseHandle, RespondMode};
    /// use std::time::Duration;
    /// use serialport;
    ///
    /// let s = serialport::new("/dev/ttyUSB0", 115200)
    ///     .timeout(Duration::from_secs(1))
    ///     .open()
    ///     .unwrap();
    /// let mut driver = Driver::new(s).unwrap();
    /// let mut estop = driver.new_estop();
    /// let mut m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    ///
    /// let handle1 = m1.start_motor().unwrap();
    /// println!("started motors");
    /// estop.estop(1000).unwrap();
    /// println!("motors stopped");
    /// ```
    pub fn estop(&mut self, millis: u64) -> Result<(), DriverError> {
        self.0.estop(millis)
    }
}
