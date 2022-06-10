use nom::Finish;

use super::{MotorStop, MotorType, Msg, PositioningMode, Record, RespondMode, RotationDirection};

#[test]
fn motortype_parse() {
    let (_, t) = MotorType::parse(b"+0").finish().unwrap();
    let expected = MotorType::Stepper;
    assert_eq!(t, expected)
}

#[test]
#[should_panic]
fn motortype_parse_oob() {
    let (_, _) = MotorType::parse(b"+3").finish().unwrap();
}

#[test]
#[should_panic]
fn motortype_parse_garbage() {
    let (_, _) = MotorType::parse(b"asdf").finish().unwrap();
}

#[test]
fn motortype_display() {
    let s = format!("{}", MotorType::BLDCHall);
    let expected = "1";
    assert_eq!(expected, s)
}

#[test]
fn motorstop_display() {
    let s = format!("{}", MotorStop::QuickStop);
    let expected = "0";
    assert_eq!(expected, s)
}

#[test]
fn respondmode_parse() {
    let (_, m) = RespondMode::parse(b"+0").finish().unwrap();
    let expected = RespondMode::Quiet;
    assert_eq!(m, expected)
}

#[test]
#[should_panic]
fn respondmode_parse_oob() {
    let (_, _) = RespondMode::parse(b"+3").finish().unwrap();
}

#[test]
#[should_panic]
fn respondmode_parse_garbage() {
    let (_, _) = RespondMode::parse(b"asdf").finish().unwrap();
}

#[test]
fn respondmode_display() {
    let s = format!("{}", RespondMode::NotQuiet);
    let expected = "1";
    assert_eq!(expected, s)
}

#[test]
fn positioningmode_parse() {
    let (_, m) = PositioningMode::parse(b"+7").finish().unwrap();
    let expected = PositioningMode::ClockDirectionManualLeft;
    assert_eq!(m, expected)
}

#[test]
#[should_panic]
fn positioningmode_parse_oob() {
    let (_, _) = PositioningMode::parse(b"+0").finish().unwrap();
}

#[test]
#[should_panic]
fn positioningmode_parse_garbage() {
    let (_, _) = PositioningMode::parse(b"asdf").finish().unwrap();
}

#[test]
fn positioningmode_display() {
    let s = format!("{}", PositioningMode::Absolute);
    let expected = "2";
    assert_eq!(expected, s)
}

#[test]
fn rotationdirection_parse() {
    let (_, d) = RotationDirection::parse(b"+1").finish().unwrap();
    let expected = RotationDirection::Right;
    assert_eq!(d, expected)
}

#[test]
#[should_panic]
fn rotationdirection_parse_oob() {
    let (_, _) = RotationDirection::parse(b"+2").finish().unwrap();
}

#[test]
#[should_panic]
fn rotationdirection_parse_garbage() {
    let (_, _) = RotationDirection::parse(b"asdf").finish().unwrap();
}

#[test]
fn rotationdirection_display() {
    let s = format!("{}", RotationDirection::Left);
    let expected = "0";
    assert_eq!(expected, s)
}

#[test]
fn record_parse() {
    let (_, r) = Record::parse(b"p+1s+400u+400o+1000n+1000b+2364B+0d+0t+0W+1P+0N+0:b+1:B+0")
        .finish()
        .unwrap();
    let expected = Record {
        positioning_mode: PositioningMode::Relative,
        travel_distance: 400,
        min_frequency: 400,
        max_frequency: 1000,
        max_frequency2: 1000,
        accel_ramp: 2364,
        brake_ramp: 0,
        rotation_direction: RotationDirection::Left,
        rotation_direction_change: false,
        repetitions: 1,
        record_pause: 0,
        continuation_record: 0,
    };
    assert_eq!(r, expected);
}

#[test]
fn msg_parse_long_write() {
    let (_, m) = Msg::parse(b"1:Keyword=1337\r").finish().unwrap();
    let expected = Msg {
        address: Some(1),
        payload: b":Keyword=1337".to_vec(),
    };
    assert_eq!(m, expected)
}

#[test]
fn msg_parse_long_read() {
    let (_, m) = Msg::parse(b"1:CL_motor_type+1\r").finish().unwrap();
    let expected = Msg {
        address: Some(1),
        payload: b":CL_motor_type+1".to_vec(),
    };
    assert_eq!(m, expected)
}

#[test]
fn msg_parse_short_no_arg() {
    let (_, m) = Msg::parse(b"1A\r").finish().unwrap();
    let expected = Msg {
        address: Some(1),
        payload: b"A".to_vec(),
    };
    assert_eq!(m, expected)
}

#[test]
fn msg_parse_all() {
    let (_, m) = Msg::parse(b"*S+1\r").finish().unwrap();
    let expected = Msg {
        address: None,
        payload: b"S+1".to_vec(),
    };
    assert_eq!(m, expected)
}

#[test]
#[should_panic]
fn msg_parse_no_return() {
    let (_, _) = Msg::parse(b"*S+1").finish().unwrap();
}

#[test]
#[should_panic]
fn msg_parse_no_addr() {
    let (_, _) = Msg::parse(b"S+1\r").finish().unwrap();
}

#[test]
#[should_panic]
fn msg_parse_addr_oob() {
    let (_, _) = Msg::parse(b"1337S+1\r").finish().unwrap();
}

#[test]
#[should_panic]
fn msg_parse_no_payload() {
    let (_, _) = Msg::parse(b"1\r").finish().unwrap();
}

#[test]
#[should_panic]
fn msg_parse_garbage() {
    let (_, _) = Msg::parse(b"gsrceitng").finish().unwrap();
}
