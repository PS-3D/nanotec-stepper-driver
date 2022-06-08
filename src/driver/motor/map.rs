//! Provides Bindings for the individual commands

macro_rules! makepl {
    ($($name:ident, $val:literal),*) => {
        $(pub const $name: &str = $val;)*
    };
}

// symbols of the commands in the order they appear in the manual
#[rustfmt::skip]
makepl!(
    READ, "Z",
    START_MOTOR, "A",
    STOP_MOTOR, "S",
    LOAD_RECORD, "y",
    READ_CURRENT_RECORD, "|",
    SAVE_RECORD, ">",
    POSITIONING_MODE, "p",
    TRAVEL_DISTANCE, "s",
    MIN_FREQUENCY, "u",
    MAX_FREQUENCY, "o",
    MAX_FREQUENCY2, "n",
    ACCEL_RAMP, "b",
    ACCEL_RAMP_NO_CONVERSION, ":accel",
    BRAKE_RAMP, "B",
    BRAKE_RAMP_NO_CONVERSION, ":decel",
    ROTATION_DIRECTION, "d",
    ROTATION_DIRECTION_CHANGE, "t",
    REPETITIONS, "W",
    RECORD_PAUSE, "P",
    CONTINUATION_RECORD, "N",
    MAX_ACCEL_JERK, ":b",
    MAX_BRAKE_JERK, ":B"
);
