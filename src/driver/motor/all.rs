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
            write::{DummyResponseHandle, WrapperResponseHandle, WriteResponseHandle},
            ResponseHandle,
        },
        DriverError, InnerDriver, Map,
    },
    motor_common_functions, DResult,
};
use crate::util::ensure;
use std::{
    fmt::{Arguments, Debug, Display},
    io::Write,
    sync::Arc,
};

#[derive(Debug)]
pub struct AllMotor(Arc<InnerDriver>);

// TODO docs
impl AllMotor {
    pub(in super::super) fn new(driver: Arc<InnerDriver>) -> Self {
        Self(driver)
    }

    fn write(&self, args: Arguments<'_>) -> Result<WrapperResponseHandle, DriverError> {
        let rm = self.0.send_all(args)?;
        Ok(match rm {
            RespondMode::NotQuiet => {
                // unfortunately there isn't a better way rn.
                // value chosen sorta random, 64 bytes should be enough for nearly all
                // commands tho
                let mut sent = Vec::with_capacity(64);
                sent.write_fmt(args)?;
                WrapperResponseHandle::Write(WriteResponseHandle::new(
                    Arc::clone(&self.0),
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
        // FIXME TOC/TOU race condition
        self.0.as_ref().set_respond_mode_all(mode);
        // FIXME state corruption possible on fail of this write (even more so if
        // you consider that for setting the respendmode to NotQuiet, the command
        // could have worked but then, while reading the response)
        self.short_write(map::READ_CURRENT_RECORD, mode)
    }

    // TODO maybe change returntype to
    // DResult<impl ResponseHandle<Ret = Option<BTreeMap<u8, impl ResponseHandle<Ret = MotorStatus>>>>>
    // and impliment ResponseHandle<Ret = BTreeMap<u8, T>> for BTreeMap<u8, impl ResponseHandle<Ret = T>>
    // then the user could decide to wait in a specific order or just ignore it
    // TODO maybe implement ResponseHandle<Ret = Option<T>> for Option<impl ResponseHandle<T>>
    // pub fn start_motor(
    //     &mut self,
    // ) -> DResult<impl ResponseHandle<Ret = Map<u8, impl ResponseHandle<Ret = MotorStatus>>>> {
    //     let res_modes = self.short_write(map::START_MOTOR, "");
    //     if res_modes.values().any(RespondMode::is_responding) {
    //         Some()
    //     } else {
    //     }
    //     todo!()
    // }
}

impl Drop for AllMotor {
    /// Removes this motor from the driver.\
    /// Afterwards, a AllMotor can be added again by calling
    /// [`Driver::add_motor`][super::super::Driver::add_all_motor].
    ///
    /// See also [here][`Drop`]
    fn drop(&mut self) {
        self.0.as_ref().drop_all_motor()
    }
}
