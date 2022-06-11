use nanotec_stepper_driver::{Driver, PositioningMode, Record, ResponseHandle, RotationDirection};
use nanotec_stepper_driver_test::Interface;

#[test]
fn test_single() {
    let mut interface = Interface::new();
    let driver = Driver::new(interface.clone());
    let mut m1 = driver.add_motor(1).unwrap();
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
    m1.set_repetitions(1).unwrap().wait().unwrap();
    interface.add_cmd_echo(b"#1P0\r");
    m1.set_record_pause(0).unwrap().wait().unwrap();
    interface.add_cmd_echo(b"#1N0\r");
    m1.set_continuation_record(0).unwrap().wait().unwrap();
    interface.add_cmd_echo(b"#1:b17\r");
    m1.set_max_accel_jerk(17).unwrap().wait().unwrap();
    interface.add_cmd_echo(b"#1:B17\r");
    m1.set_max_brake_jerk(17).unwrap().wait().unwrap();
    interface.add_cmd_echo(b"#1A\r");
    m1.start_motor().unwrap().wait().unwrap();
}

#[test]
fn test_concurrent() {
    let mut interface = Interface::new();
    let driver = Driver::new(interface.clone());
    let mut m1 = driver.add_motor(1).unwrap();
    let mut m2 = driver.add_motor(2).unwrap();
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
}
