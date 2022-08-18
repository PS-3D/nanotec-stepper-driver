pub mod all;
pub mod single;

use super::DriverError;

//

type DResult<T> = Result<T, DriverError>;

//

macro_rules! motor_common_functions {
    ($write:expr) => {
        fn short_write<D: Display>(
            &self,
            mnemonic: &str,
            data: D,
        ) -> Result<WrapperResponseHandle, DriverError> {
            $write(self, format_args!("{}{}", mnemonic, data))
        }

        fn long_write<D: Display>(
            &self,
            mnemonic: &str,
            data: D,
        ) -> Result<WrapperResponseHandle, DriverError> {
            $write(self, format_args!("{}={}", mnemonic, data))
        }

        pub fn set_motor_type(&mut self, t: MotorType) -> DResult<impl ResponseHandle<Ret = ()>> {
            self.long_write(map::MOTOR_TYPE, t)
        }

        pub fn set_phase_current(&mut self, c: u8) -> DResult<impl ResponseHandle<Ret = ()>> {
            ensure!(c <= 150, DriverError::InvalidArgument);
            self.short_write(map::PHASE_CURRENT, c)
        }

        pub fn set_standstill_phase_current(
            &mut self,
            c: u8,
        ) -> DResult<impl ResponseHandle<Ret = ()>> {
            ensure!(c <= 150, DriverError::InvalidArgument);
            self.short_write(map::STANDSTILL_PHASE_CURRENT, c)
        }

        pub fn set_bldc_peak_current(&mut self, c: u8) -> DResult<impl ResponseHandle<Ret = ()>> {
            ensure!(c <= 150, DriverError::InvalidArgument);
            self.long_write(map::BLDC_PEAK_CURRENT, c)
        }

        pub fn set_bldc_current_time_constant(
            &mut self,
            t: u16,
        ) -> DResult<impl ResponseHandle<Ret = ()>> {
            self.long_write(map::BLDC_CURRENT_TIME_CONSTANT, t)
        }

        pub fn set_step_mode(&mut self, m: StepMode) -> DResult<impl ResponseHandle<Ret = ()>> {
            self.short_write(map::STEP_MODE, m)
        }

        pub fn set_drive_address(&mut self, a: u8) -> DResult<impl ResponseHandle<Ret = ()>> {
            ensure!(a >= 1 && a <= 254, DriverError::InvalidArgument);
            self.short_write(map::DRIVE_ADDRESS, a)
        }

        pub fn set_motor_id(&mut self, id: u32) -> DResult<impl ResponseHandle<Ret = ()>> {
            ensure!(id <= 2147483647, DriverError::InvalidArgument);
            self.long_write(map::MOTOR_ID, id)
        }

        pub fn set_limit_switch_behavior(
            &mut self,
            l: LimitSwitchBehavior,
        ) -> DResult<impl ResponseHandle<Ret = ()>> {
            self.short_write(map::LIMIT_SWITCH_BEHAVIOR, l)
        }

        pub fn set_error_correction_mode(
            &mut self,
            m: ErrorCorrectionMode,
        ) -> DResult<impl ResponseHandle<Ret = ()>> {
            self.short_write(map::ERROR_CORRECTION_MODE, m)
        }

        pub fn set_auto_correction_record(
            &mut self,
            r: u8,
        ) -> DResult<impl ResponseHandle<Ret = ()>> {
            ensure!(r <= 32, DriverError::InvalidArgument);
            self.short_write(map::AUTO_CORRECTION_RECORD, r)
        }

        // TODO encoder direction

        pub fn set_swing_out_time(&mut self, st: u8) -> DResult<impl ResponseHandle<Ret = ()>> {
            ensure!(st <= 250, DriverError::InvalidArgument);
            self.short_write(map::SWING_OUT_TIME, st)
        }

        pub fn set_max_encoder_deviation(
            &mut self,
            d: u8,
        ) -> DResult<impl ResponseHandle<Ret = ()>> {
            ensure!(d <= 250, DriverError::InvalidArgument);
            self.short_write(map::MAX_ENCODER_DEVIATION, d)
        }

        pub fn set_feedrate_numerator(&mut self, n: u32) -> DResult<impl ResponseHandle<Ret = ()>> {
            ensure!(n <= 2147483647, DriverError::InvalidArgument);
            self.long_write(map::FEED_RATE_NUMERATOR, n)
        }

        pub fn set_feedrate_denominator(
            &mut self,
            d: u32,
        ) -> DResult<impl ResponseHandle<Ret = ()>> {
            ensure!(d <= 2147483647, DriverError::InvalidArgument);
            self.long_write(map::FEED_RATE_DENOMINATOR, d)
        }

        pub fn reset_position_error(&mut self, p: i32) -> DResult<impl ResponseHandle<Ret = ()>> {
            ensure!(
                p >= -100_000_000 && p <= 100_000_000,
                DriverError::InvalidArgument
            );
            self.short_write(map::RESET_POS_ERR, p)
        }

        pub fn set_digital_input_function(
            &mut self,
            i: u8,
            f: DigitalInputFunction,
        ) -> DResult<impl ResponseHandle<Ret = ()>> {
            ensure!(i <= 8 && i >= 1, DriverError::InvalidArgument);
            let mut name = map::DIGITAL_INPUT_FUNCTION_PARTIAL.to_string();
            name.push(char::from_u32(60 + (i as u32)).unwrap());
            self.long_write(name.as_str(), f)
        }

        pub fn set_digital_output_function(
            &mut self,
            o: u8,
            f: DigitalOutputFunction,
        ) -> DResult<impl ResponseHandle<Ret = ()>> {
            ensure!(o <= 8 && o >= 1, DriverError::InvalidArgument);
            let mut name = map::DIGITAL_OUTPUT_FUNCTION_PARTIAL.to_string();
            name.push(char::from_u32(60 + (o as u32)).unwrap());
            self.long_write(name.as_str(), f)
        }

        // Not implementing Masking and demasking inputs since it is deprecated

        pub fn set_reverse_in_out_polarity(
            &mut self,
            b: u32,
        ) -> DResult<impl ResponseHandle<Ret = ()>> {
            ensure!(b & 0x1ff00ff == b, DriverError::InvalidArgument);
            self.short_write(map::REVERSE_IN_OUT_POLARITY, b)
        }

        pub fn set_input_debounce_time(&mut self, t: u8) -> DResult<impl ResponseHandle<Ret = ()>> {
            ensure!(t <= 250, DriverError::InvalidArgument);
            self.short_write(map::INPUT_DEBOUNCE_TIME, t)
        }

        // TODO set outputs

        pub fn reset_eeprom(&mut self) -> DResult<impl ResponseHandle<Ret = ()>> {
            self.short_write(map::RESET_EEPROM, "")
        }

        // TODO start bootloader

        pub fn set_reverse_clearance(&mut self, c: u16) -> DResult<impl ResponseHandle<Ret = ()>> {
            ensure!(c <= 9999, DriverError::InvalidArgument);
            self.short_write(map::REVERSE_CLEARANCE, c)
        }

        pub fn set_ramp_type(&mut self, t: RampType) -> DResult<impl ResponseHandle<Ret = ()>> {
            self.long_write(map::RAMP_TYPE, t)
        }

        pub fn set_brake_voltage_off_wait_time(
            &mut self,
            t: u16,
        ) -> DResult<impl ResponseHandle<Ret = ()>> {
            self.long_write(map::WAIT_TIME_BRAKE_VOLTAGE_OFF, t)
        }

        pub fn set_motor_movement_wait_time(
            &mut self,
            t: u16,
        ) -> DResult<impl ResponseHandle<Ret = ()>> {
            self.long_write(map::WAIT_TIME_MOTOR_MOVE, t)
        }

        pub fn set_motor_current_off_wait_time(
            &mut self,
            t: u16,
        ) -> DResult<impl ResponseHandle<Ret = ()>> {
            self.long_write(map::WAIT_TIME_MOTOR_CURRENT_OFF, t)
        }

        pub fn set_baud_rate(&mut self, br: BaudRate) -> DResult<impl ResponseHandle<Ret = ()>> {
            self.long_write(map::BAUD_RATE, br)
        }

        // TODO set crc checksum
        // TODO set hall config

        pub fn set_quickstop_ramp(&mut self, s: u16) -> DResult<impl ResponseHandle<Ret = ()>> {
            ensure!(s <= 8000, DriverError::InvalidArgument);
            self.short_write(map::QUICKSTOP_RAMP, s)
        }

        pub fn set_quickstop_ramp_no_conversion(
            &mut self,
            r: u32,
        ) -> DResult<impl ResponseHandle<Ret = ()>> {
            ensure!(r <= 3_000_000, DriverError::InvalidArgument);
            self.long_write(map::QUICKSTOP_RAMP_NO_CONVERSION, r)
        }

        pub fn set_gearfactor_numerator(
            &mut self,
            n: u8,
        ) -> DResult<impl ResponseHandle<Ret = ()>> {
            self.long_write(map::GEAR_FACTOR_NUMERATOR, n)
        }

        pub fn set_gearfactor_denominator(
            &mut self,
            d: u8,
        ) -> DResult<impl ResponseHandle<Ret = ()>> {
            self.long_write(map::GEAR_FACTOR_DENOMINATOR, d)
        }

        pub fn stop_motor(&mut self, stop: MotorStop) -> DResult<impl ResponseHandle<Ret = ()>> {
            self.short_write(map::STOP_MOTOR, stop)
        }

        pub fn load_record(&mut self, n: u8) -> DResult<impl ResponseHandle<Ret = ()>> {
            ensure!(n >= 1 && n <= 32, DriverError::InvalidArgument);
            self.short_write(map::LOAD_RECORD, n)
        }

        pub fn save_record(&mut self, n: u8) -> DResult<impl ResponseHandle<Ret = ()>> {
            ensure!(n >= 1 && n <= 32, DriverError::InvalidArgument);
            self.short_write(map::SAVE_RECORD, n)
        }

        pub fn set_positioning_mode(
            &mut self,
            mode: PositioningMode,
        ) -> DResult<impl ResponseHandle<Ret = ()>> {
            self.short_write(map::POSITIONING_MODE, mode)
        }

        pub fn set_travel_distance(
            &mut self,
            distance: i32,
        ) -> DResult<impl ResponseHandle<Ret = ()>> {
            ensure!(
                distance >= -100_000_000 && distance <= 100_000_000,
                DriverError::InvalidArgument
            );
            self.short_write(map::TRAVEL_DISTANCE, distance)
        }

        pub fn set_min_frequency(
            &mut self,
            frequency: u32,
        ) -> DResult<impl ResponseHandle<Ret = ()>> {
            ensure!(
                frequency >= 1 && frequency <= 160_000,
                DriverError::InvalidArgument
            );
            self.short_write(map::MIN_FREQUENCY, frequency)
        }

        pub fn set_max_frequency(
            &mut self,
            frequency: u32,
        ) -> DResult<impl ResponseHandle<Ret = ()>> {
            ensure!(
                frequency >= 1 && frequency <= 1_000_000,
                DriverError::InvalidArgument
            );
            self.short_write(map::MAX_FREQUENCY, frequency)
        }

        pub fn set_max_frequency2(
            &mut self,
            frequency: u32,
        ) -> DResult<impl ResponseHandle<Ret = ()>> {
            ensure!(
                frequency >= 1 && frequency <= 1_000_000,
                DriverError::InvalidArgument
            );
            self.short_write(map::MAX_FREQUENCY2, frequency)
        }

        pub fn set_accel_ramp(&mut self, n: u16) -> DResult<impl ResponseHandle<Ret = ()>> {
            ensure!(n >= 1, DriverError::InvalidArgument);
            self.short_write(map::ACCEL_RAMP, n)
        }

        pub fn set_accel_ramp_no_conversion(
            &mut self,
            n: u32,
        ) -> DResult<impl ResponseHandle<Ret = ()>> {
            ensure!(n >= 1 && n <= 3_000_000, DriverError::InvalidArgument);
            self.long_write(map::ACCEL_RAMP_NO_CONVERSION, n)
        }

        pub fn set_brake_ramp(&mut self, n: u16) -> DResult<impl ResponseHandle<Ret = ()>> {
            self.short_write(map::BRAKE_RAMP, n)
        }

        pub fn set_brake_ramp_no_conversion(
            &mut self,
            n: u32,
        ) -> DResult<impl ResponseHandle<Ret = ()>> {
            ensure!(n <= 3_000_000, DriverError::InvalidArgument);
            self.long_write(map::BRAKE_RAMP_NO_CONVERSION, n)
        }

        pub fn set_rotation_direction(
            &mut self,
            direction: RotationDirection,
        ) -> DResult<impl ResponseHandle<Ret = ()>> {
            self.short_write(map::ROTATION_DIRECTION, direction)
        }

        pub fn set_rotation_direction_change(
            &mut self,
            change: bool,
        ) -> DResult<impl ResponseHandle<Ret = ()>> {
            self.short_write(map::ROTATION_DIRECTION_CHANGE, change as u8)
        }

        pub fn set_repetitions(
            &mut self,
            n: Repetitions,
        ) -> DResult<impl ResponseHandle<Ret = ()>> {
            if let Repetitions::N(r) = n {
                ensure!(r <= 254, DriverError::InvalidArgument);
            }
            self.short_write(map::REPETITIONS, n)
        }

        pub fn set_record_pause(&mut self, n: u16) -> DResult<impl ResponseHandle<Ret = ()>> {
            self.short_write(map::RECORD_PAUSE, n)
        }

        pub fn set_continuation_record(
            &mut self,
            n: Option<u8>,
        ) -> DResult<impl ResponseHandle<Ret = ()>> {
            let r = match n {
                Some(r) => {
                    ensure!(r <= 32 && r >= 1, DriverError::InvalidArgument);
                    r
                }
                None => 0,
            };
            self.short_write(map::CONTINUATION_RECORD, r)
        }

        pub fn set_max_accel_jerk(&mut self, n: u32) -> DResult<impl ResponseHandle<Ret = ()>> {
            ensure!(n >= 1 && n <= 100_000_000, DriverError::InvalidArgument);
            self.short_write(map::MAX_ACCEL_JERK, n)
        }

        pub fn set_max_brake_jerk(&mut self, n: u32) -> DResult<impl ResponseHandle<Ret = ()>> {
            ensure!(n >= 1 && n <= 100_000_000, DriverError::InvalidArgument);
            self.short_write(map::MAX_BRAKE_JERK, n)
        }
    };
}

pub(self) use motor_common_functions;
