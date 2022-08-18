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

#[derive(Debug)]
pub struct AllMotor(Rc<RefCell<InnerDriver>>);

// TODO docs
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
