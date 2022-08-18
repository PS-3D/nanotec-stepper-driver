use super::{
    super::{
        cmd::{
            frame::MotorAddress,
            payload::{
                BaudRate, DigitalInputFunction, DigitalOutputFunction, ErrorCorrectionMode,
                LimitSwitchBehavior, MotorStatus, MotorStop, MotorType, PositioningMode, RampType,
                Repetitions, RespondMode, RotationDirection, StepMode,
            },
        },
        map,
        responsehandle::{
            map::ResponseHandleMap,
            read::StatusResponseHandle,
            write::{DummyResponseHandle, WrapperResponseHandle, WriteResponseHandle},
            ResponseHandle,
        },
        DriverError, InnerDriver, Map,
    },
    motor_common_functions, DResult,
};
use crate::util::ensure;
use std::{
    cell::RefCell,
    fmt::{Arguments, Debug, Display},
    io::Write,
    rc::Rc,
};

/// Controls all motors at once
///
/// Unlike [`Motor`][super::single::Motor], the AllMotor only contains setters
/// since it isn't really possible to send a read-command to all motors because
/// they also would all answer with `"*"` as the address, making it impossible
/// to differentiate between who actually sent what.
///
/// Like with [`Motor`][super::single::Motor], the functions don't return values
/// directly but rather a [`ResponseHandle`] on which [`ResponseHandle::wait`] can
/// then be called to wait for the response of the motor. Since the AllMotor only
/// has setters none of the handles actually return a value, they only exist to
/// confirm the answer of the motor.
///
/// # Errors
/// If a value doesn't match the specifications of the corresponding command
/// in the manual, [`DriverError::InvalidArgument`] is returned.
/// A [`DriverError::NotAvailable`] is returned if a command is sent to all
/// motors while some are still waiting for a response. Or if a command is sent
/// to a single motors while not all motors have responded to a command for all
/// motors yet, since it isn't possible to distinguish the responses from single
/// motors in that case.
/// A [`DriverError::IoError`] is also possible, if there was an error sending the
/// command.
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
/// let mut driver = Driver::new(s).unwrap();
/// let m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
/// let m2 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
/// let mut all = driver.add_all_motor().unwrap();
///
/// all.set_travel_distance(42000).unwrap().wait().unwrap();
/// let map = all.start_motor().unwrap().wait().unwrap();
///
/// println!("driving 42000 steps");
/// assert!(map.is_empty());
/// ```
#[derive(Debug)]
pub struct AllMotor(Rc<RefCell<InnerDriver>>);

impl AllMotor {
    pub(in super::super) fn new(driver: Rc<RefCell<InnerDriver>>) -> Self {
        Self(driver)
    }

    fn write(&self, args: Arguments<'_>) -> Result<WrapperResponseHandle, DriverError> {
        let rm = self.0.borrow_mut().send_all(args)?;
        Ok(match rm {
            RespondMode::NotQuiet => {
                // unfortunately there isn't a better way rn.
                // value chosen sorta random, 64 bytes should be enough for nearly all
                // commands tho
                let mut sent = Vec::with_capacity(64);
                sent.write_fmt(args)?;
                WrapperResponseHandle::Write(WriteResponseHandle::new(
                    Rc::clone(&self.0),
                    MotorAddress::All,
                    sent,
                ))
            }
            RespondMode::Quiet => WrapperResponseHandle::Dummy(DummyResponseHandle::new(())),
        })
    }

    motor_common_functions!(AllMotor::write);

    /// This command doesn't exist in the manual.
    /// [1.6.4 Reading out the current record](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf)
    /// was so convoluted that it was split into multiple functions, namely this
    /// one, [`Motor::get_current_record`][super::single::Motor::get_current_record]
    /// [`Motor::get_record`][super::single::Motor::get_record],
    /// [`Motor::get_respond_mode`][super::single::Motor::get_respond_mode]
    /// and [`Motor::set_respond_mode`][super::single::Motor::set_respond_mode].
    /// This function is responsible for setting whether or not the firmware
    /// responds to most commands and the other 2 are responsible for actually
    /// reading out records.
    pub fn set_respond_mode(
        &mut self,
        mode: RespondMode,
    ) -> DResult<impl ResponseHandle<Ret = ()>> {
        self.0.borrow_mut().set_respond_mode_all(mode);
        self.short_write(map::READ_CURRENT_RECORD, mode)
    }

    // TODO maybe change returntype to
    // DResult<impl ResponseHandle<Ret = Option<BTreeMap<u8, impl ResponseHandle<Ret = MotorStatus>>>>>
    // and impliment ResponseHandle<Ret = BTreeMap<u8, T>> for BTreeMap<u8, impl ResponseHandle<Ret = T>>
    // then the user could decide to wait in a specific order or just ignore it
    // NOTE this is sortof difficult due to the whole recoverable error thing
    /// Starts all motors at once.
    ///
    /// # Returns
    /// This function returns a nested [`ResponseHandle`]. The reason for that
    /// is that some motors might automatically send the status when they're done
    /// moving and so the map returned by the first handle contains another handle
    /// to receive the status for each motor that is set to automatically send it.
    /// The first handle is just for the response confirming the command, like with
    /// any other command.
    ///
    /// # Examples
    /// ```no_run
    /// # use nanotec_stepper_driver::{Driver, ResponseHandle, RespondMode, MotorStatus};
    /// use std::time::Duration;
    /// use serialport;
    ///
    /// let s = serialport::new("/dev/ttyUSB0", 115200)
    ///     .timeout(Duration::from_secs(1))
    ///     .open()
    ///     .unwrap();
    /// let mut driver = Driver::new(s).unwrap();
    /// let m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    /// let m2 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    /// let mut all = driver.add_all_motor().unwrap();
    /// let m1 = m1.start_sending_auto_status().unwrap().wait().unwrap();
    /// let m2 = m2.start_sending_auto_status().unwrap().wait().unwrap();
    ///
    /// let map = all.start_motor().unwrap().wait().unwrap();
    /// println!("started all motors");
    ///
    /// for handle in map.into_iter().map(|t| t.1) {
    ///     let status = handle.wait().unwrap();
    ///     assert_eq!(status, MotorStatus::Ready);
    /// }
    /// println!("finished driving on all motors");
    /// ```
    pub fn start_motor(
        &mut self,
    ) -> DResult<impl ResponseHandle<Ret = Map<u8, impl ResponseHandle<Ret = MotorStatus>>>> {
        let res_modes = self.short_write(map::START_MOTOR, "");
        let send_status = self
            .0
            .borrow()
            .get_send_autostatus_all()
            .into_iter()
            .filter(|kv| kv.1)
            .map(|(a, _)| (a, StatusResponseHandle::new(Rc::clone(&self.0), a)))
            .collect();
        res_modes.map(move |h| h.map(move |_| send_status))
    }
}

impl Drop for AllMotor {
    /// Removes this motor from the driver.\
    /// Afterwards, a AllMotor can be added again by calling
    /// [`Driver::add_motor`][super::super::Driver::add_all_motor].
    ///
    /// See also [here][`Drop`]
    fn drop(&mut self) {
        self.0.borrow_mut().drop_all_motor()
    }
}
