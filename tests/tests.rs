use nanotec_stepper_driver::{
    Driver, MotorStatus, PositioningMode, Record, Repetitions, RespondMode, ResponseHandle,
    RotationDirection,
};
use nanotec_stepper_driver_test::Interface;

#[test]
fn single() {
    let mut interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface.clone())).unwrap();
    let mut m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    interface.add_cmd_echo(b"#1p2\r");
    m1.set_positioning_mode(PositioningMode::Absolute)
        .unwrap()
        .wait()
        .unwrap();
    interface.add_cmd_echo(b"#1s1337\r");
    m1.set_travel_distance(1337).unwrap().wait().unwrap();
    interface.add_cmd_echo(b"#1u1\r");
    m1.set_min_frequency(1).unwrap().wait().unwrap();
    interface.add_cmd_echo(b"#1o42\r");
    m1.set_max_frequency(42).unwrap().wait().unwrap();
    interface.add_cmd_echo(b"#1b69\r");
    m1.set_accel_ramp(69).unwrap().wait().unwrap();
    interface.add_cmd_echo(b"#1B69\r");
    m1.set_brake_ramp(69).unwrap().wait().unwrap();
    interface.add_cmd_echo(b"#1d0\r");
    m1.set_rotation_direction(RotationDirection::Left)
        .unwrap()
        .wait()
        .unwrap();
    interface.add_cmd_echo(b"#1t0\r");
    m1.set_rotation_direction_change(false)
        .unwrap()
        .wait()
        .unwrap();
    interface.add_cmd_echo(b"#1W1\r");
    m1.set_repetitions(Repetitions::N(1))
        .unwrap()
        .wait()
        .unwrap();
    interface.add_cmd_echo(b"#1P0\r");
    m1.set_record_pause(0).unwrap().wait().unwrap();
    interface.add_cmd_echo(b"#1N0\r");
    m1.set_continuation_record(None).unwrap().wait().unwrap();
    interface.add_cmd_echo(b"#1:b17\r");
    m1.set_max_accel_jerk(17).unwrap().wait().unwrap();
    interface.add_cmd_echo(b"#1:B17\r");
    m1.set_max_brake_jerk(17).unwrap().wait().unwrap();
    interface.add_cmd_echo(b"#1A\r");
    m1.start_motor().unwrap().wait().unwrap();
}

#[test]
fn concurrent() {
    let mut interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface.clone())).unwrap();
    let m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    let m2 = driver.add_motor(2, RespondMode::NotQuiet).unwrap();

    interface.add_cmd_echo(b"#1J1\r");
    let mut m1 = m1.start_sending_auto_status().unwrap().wait().unwrap();
    interface.add_cmd_echo(b"#2J1\r");
    let mut m2 = m2.start_sending_auto_status().unwrap().wait().unwrap();

    interface.add_write(b"#1Z|\r");
    interface.add_write(b"#2y3\r");
    let h1 = m1.get_current_record().unwrap();
    let h2 = m2.load_record(3).unwrap();
    // we should be able to reverse the order the messages arrive in
    interface.add_read(b"2y3\r");
    interface
        .add_read(b"1Zp+1s-15000u+10o+3000n+1b+1000B+500d+1t+1W+50P+5000N+3:b+10000:B+10000\r");
    let expected_record = Record {
        positioning_mode: PositioningMode::Relative,
        travel_distance: -15000,
        min_frequency: 10,
        max_frequency: 3000,
        max_frequency2: 1,
        accel_ramp: 1000,
        brake_ramp: 500,
        rotation_direction: RotationDirection::Right,
        rotation_direction_change: true,
        repetitions: 50,
        record_pause: 5000,
        continuation_record: 3,
        max_accel_jerk: 10000,
        max_brake_jerk: 10000,
    };
    let r = h1.wait().unwrap();
    assert_eq!(r, expected_record);
    h2.wait().unwrap();

    interface.add_write(b"#1p2\r");
    let h1 = m1.set_positioning_mode(PositioningMode::Absolute).unwrap();
    interface.add_cmd_echo(b"#2s1337\r");
    let h2 = m2.set_travel_distance(1337).unwrap();
    interface.add_read(b"1p2\r");
    h1.wait().unwrap();
    h2.wait().unwrap();

    interface.add_write(b"#2Zo\r");
    let h2 = m2.get_max_frequency().unwrap();
    interface.add_cmd_echo(b"#1s1337\r");
    let h1 = m1.set_travel_distance(1337).unwrap();
    interface.add_read(b"2Zo42\r");
    assert_eq!(h2.wait().unwrap(), 42);
    h1.wait().unwrap();

    interface.add_cmd_echo(b"#1B69\r");
    let h1 = m1.set_brake_ramp(69).unwrap();
    interface.add_cmd_echo(b"#2o69\r");
    let h2 = m2.set_max_frequency(69).unwrap();
    h1.wait().unwrap();
    h2.wait().unwrap();

    interface.add_cmd_echo(b"#1d0\r");
    let h1 = m1.set_rotation_direction(RotationDirection::Left).unwrap();
    interface.add_cmd_echo(b"#2t0\r");
    let h2 = m2.set_rotation_direction_change(false).unwrap();
    h1.wait().unwrap();
    h2.wait().unwrap();

    interface.add_cmd_echo(b"#2A\r");
    let h2 = m2.start_motor().unwrap().wait().unwrap();
    interface.add_cmd_echo(b"#1A\r");
    let h1 = m1.start_motor().unwrap().wait().unwrap();
    interface.add_read(b"2j161\r");
    let s2 = h2.wait().unwrap();
    interface.add_read(b"1j161\r");
    let s1 = h1.wait().unwrap();

    assert_eq!(s1, MotorStatus::Ready);
    assert_eq!(s2, MotorStatus::Ready);
}

#[test]
fn all() {
    let mut interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface.clone())).unwrap();
    let mut m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    let mut m2 = driver.add_motor(2, RespondMode::NotQuiet).unwrap();
    let mut all = driver.add_all_motor().unwrap();

    interface.add_cmd_echo(b"#*s1337\r");
    interface.add_read(b"*s1337\r");
    all.set_travel_distance(1337).unwrap().wait().unwrap();

    interface.add_cmd_echo(b"#1d0\r");
    let h1 = m1.set_rotation_direction(RotationDirection::Left).unwrap();
    interface.add_cmd_echo(b"#2d1\r");
    let h2 = m2.set_rotation_direction(RotationDirection::Right).unwrap();
    h1.wait().unwrap();
    h2.wait().unwrap();

    interface.add_cmd_echo(b"#*A\r");
    interface.add_read(b"*A\r");
    all.start_motor().unwrap().wait().unwrap();
}

#[test]
fn quiet() {
    let mut interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface.clone())).unwrap();
    let mut m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    let mut m2 = driver.add_motor(2, RespondMode::NotQuiet).unwrap();
    let mut all = driver.add_all_motor().unwrap();

    interface.add_write(b"#*|0\r");
    all.set_respond_mode(RespondMode::Quiet)
        .unwrap()
        .wait()
        .unwrap();

    interface.add_write(b"#1s1337\r");
    m1.set_travel_distance(1337).unwrap().wait().unwrap();
    interface.add_write(b"#2s4269\r");
    m2.set_travel_distance(4269).unwrap().wait().unwrap();

    interface.add_write(b"#*p2\r");
    all.set_positioning_mode(PositioningMode::Absolute)
        .unwrap()
        .wait()
        .unwrap();

    interface.add_write(b"#2d1\r");
    m2.set_rotation_direction(RotationDirection::Right)
        .unwrap()
        .wait()
        .unwrap();
    interface.add_write(b"#1d0\r");
    m1.set_rotation_direction(RotationDirection::Left)
        .unwrap()
        .wait()
        .unwrap();

    interface.add_cmd_echo(b"#*|1\r");
    interface.add_read(b"*|1\r");
    all.set_respond_mode(RespondMode::NotQuiet)
        .unwrap()
        .wait()
        .unwrap();

    interface.add_cmd_echo(b"#*A\r");
    interface.add_read(b"*A\r");
    all.start_motor().unwrap().wait().unwrap();
}
