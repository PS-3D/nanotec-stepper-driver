use super::{
    cmd::{frame::MotorAddress, payload::RespondMode},
    Driver, DriverError,
};
use nanotec_stepper_driver_test::Interface;

#[test]
fn receive_single() {
    let mut interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface.clone())).unwrap();
    let _m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    let mut inner = driver.inner.borrow_mut();
    inner.motors.get_mut(&1).unwrap().available = false;
    interface.add_read(b"1A\r");

    let r = inner.receive_single(1).unwrap();

    assert!(interface.is_empty());
    assert_eq!(&r, b"A");
    assert!(inner.motors.get(&1).unwrap().available);
    assert!(inner.motors.get(&1).unwrap().queue.is_empty());
}

#[test]
fn receive_single_multiple() {
    let mut interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface.clone())).unwrap();
    let _m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    let _m2 = driver.add_motor(2, RespondMode::NotQuiet).unwrap();
    let mut inner = driver.inner.borrow_mut();
    inner.motors.get_mut(&1).unwrap().available = false;
    inner.motors.get_mut(&2).unwrap().available = false;
    interface.add_read(b"2A\r");
    interface.add_read(b"1A\r");

    let r = inner.receive_single(1).unwrap();

    assert!(interface.is_empty());
    assert_eq!(&r, b"A");
    assert!(inner.motors.get(&2).unwrap().available);
    assert_eq!(inner.motors.get(&2).unwrap().queue.front().unwrap(), b"A");
}

#[test]
fn receive_single_other_motor() {
    let mut interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface.clone())).unwrap();
    let _m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    let mut inner = driver.inner.borrow_mut();
    interface.add_read(b"2A\r");

    let r = inner.receive_single(1);

    assert!(matches!(
        r,
        Err(DriverError::UnexpectedResponse(MotorAddress::Single(2)))
    ));
    assert!(interface.is_empty());
    assert!(inner.motors.get(&1).unwrap().available);
    assert!(inner.motors.get(&1).unwrap().queue.is_empty());
}

#[test]
fn receive_single_all() {
    let mut interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface.clone())).unwrap();
    let _m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    let mut inner = driver.inner.borrow_mut();
    interface.add_read(b"*A\r");

    let r = inner.receive_single(1);

    assert!(matches!(
        r,
        Err(DriverError::UnexpectedResponse(MotorAddress::All))
    ));
    assert!(interface.is_empty());
    assert!(inner.motors.get(&1).unwrap().available);
    assert!(inner.motors.get(&1).unwrap().queue.is_empty());
}

#[test]
fn receive_all() {
    let mut interface = Interface::new();
    let driver = Driver::new(Box::new(interface.clone())).unwrap();
    let mut inner = driver.inner.borrow_mut();
    interface.add_read(b"*A\r");
    interface.add_read(b"*A\r");
    inner.all = Some((2, b"A".to_vec()));

    let r = inner.receive_all().unwrap();

    assert!(interface.is_empty());
    assert_eq!(&r, b"A");
    assert!(inner.all.is_none());
}

#[test]
fn receive_all_single() {
    let mut interface = Interface::new();
    let driver = Driver::new(Box::new(interface.clone())).unwrap();
    let mut inner = driver.inner.borrow_mut();
    interface.add_read(b"1A\r");
    inner.all = Some((1, b"A".to_vec()));

    let r = inner.receive_all();

    assert!(matches!(
        r,
        Err(DriverError::UnexpectedResponse(MotorAddress::Single(1)))
    ));
    assert!(interface.is_empty());
    assert_eq!(inner.all.as_ref().unwrap().0, 1);
    assert_eq!(inner.all.as_ref().unwrap().1, b"A".to_vec());
}

#[test]
fn send_single_no_response() {
    let mut interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface.clone())).unwrap();
    let _m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    let inner = driver.inner.borrow_mut();
    interface.add_write(b"#1A\r");

    inner.send_single_no_response(1, format_args!("A")).unwrap();

    assert!(inner.motors.get(&1).unwrap().available);
}

#[test]
fn send_single_no_response_waiting_all() {
    let interface = Interface::new();
    let driver = Driver::new(Box::new(interface.clone())).unwrap();
    let mut inner = driver.inner.borrow_mut();
    inner.all = Some((1, b"A".to_vec()));

    let r = inner.send_single_no_response(1, format_args!("test"));

    assert!(matches!(r, Err(DriverError::NotAvailable)));
}

#[test]
fn send_single_no_response_waiting() {
    let interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface.clone())).unwrap();
    let _m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    let mut inner = driver.inner.borrow_mut();
    inner.motors.get_mut(&1).unwrap().available = false;

    let r = inner.send_single_no_response(1, format_args!("test"));

    assert!(matches!(r, Err(DriverError::NotAvailable)));
    assert!(!inner.motors.get(&1).unwrap().available);
}

#[test]
fn send_single_with_response() {
    let mut interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface.clone())).unwrap();
    let _m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    let mut inner = driver.inner.borrow_mut();
    interface.add_write(b"#1A\r");

    inner
        .send_single_with_response(1, format_args!("A"))
        .unwrap();

    assert!(!inner.motors.get(&1).unwrap().available);
}

#[test]
fn send_single_with_response_waiting_all() {
    let interface = Interface::new();
    let driver = Driver::new(Box::new(interface.clone())).unwrap();
    let mut inner = driver.inner.borrow_mut();
    inner.all = Some((1, b"A".to_vec()));

    let r = inner.send_single_with_response(1, format_args!("test"));

    assert!(matches!(r, Err(DriverError::NotAvailable)));
}

#[test]
fn send_single_with_response_waiting() {
    let interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface.clone())).unwrap();
    let _m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    let mut inner = driver.inner.borrow_mut();
    inner.motors.get_mut(&1).unwrap().available = false;

    let r = inner.send_single_with_response(1, format_args!("test"));

    assert!(matches!(r, Err(DriverError::NotAvailable)));
    assert!(!inner.motors.get(&1).unwrap().available);
}

#[test]
fn send_all_not_quiet() {
    let mut interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface.clone())).unwrap();
    let _m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    let _m2 = driver.add_motor(2, RespondMode::Quiet).unwrap();
    let mut inner = driver.inner.borrow_mut();
    interface.add_write(b"#*A\r");

    let r = inner.send_all(format_args!("A")).unwrap();

    assert_eq!(r, RespondMode::NotQuiet);
    assert_eq!(inner.all.as_ref().unwrap().0, 1);
    assert_eq!(inner.all.as_ref().unwrap().1, b"A".to_vec());
}

#[test]
fn send_all_quiet() {
    let mut interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface.clone())).unwrap();
    let _m1 = driver.add_motor(1, RespondMode::Quiet).unwrap();
    let mut inner = driver.inner.borrow_mut();
    interface.add_write(b"#*A\r");

    let r = inner.send_all(format_args!("A")).unwrap();

    assert_eq!(r, RespondMode::Quiet);
    assert!(inner.all.is_none());
}

#[test]
fn send_all_waiting() {
    let interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface)).unwrap();
    let _m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    let _m2 = driver.add_motor(2, RespondMode::Quiet).unwrap();
    let mut inner = driver.inner.borrow_mut();
    inner.motors.get_mut(&1).unwrap().available = false;

    let r = inner.send_all(format_args!("test"));

    assert!(matches!(r, Err(DriverError::NotAvailable)));
    assert!(inner.all.is_none());
}

#[test]
fn send_all_all_waiting() {
    let interface = Interface::new();
    let driver = Driver::new(Box::new(interface)).unwrap();
    let mut inner = driver.inner.borrow_mut();
    inner.all = Some((1, b"test".to_vec()));

    let r = inner.send_all(format_args!("test"));

    assert!(matches!(r, Err(DriverError::NotAvailable)));
}

#[test]
fn add_motor() {
    let interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface)).unwrap();

    let _m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();

    let inner = driver.inner.borrow();
    let im1 = inner.motors.get(&1).unwrap();
    assert!(im1.available);
    assert_eq!(im1.respond_mode, RespondMode::NotQuiet);
    assert!(im1.queue.is_empty());
}

#[test]
fn add_motor_already_exists() {
    let interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface)).unwrap();

    let _m1 = driver.add_motor(1, RespondMode::Quiet).unwrap();
    let r = driver.add_motor(1, RespondMode::NotQuiet);

    assert!(matches!(
        r,
        Err(DriverError::AlreadyExists(MotorAddress::Single(1)))
    ));
    assert_eq!(driver.inner.borrow().motors.len(), 1);
}

#[test]
fn add_motor_multiple_times() {
    let interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface)).unwrap();

    let m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    // this is artificial, in the real world this would only happen if motor
    // got dropped
    driver.inner.borrow_mut().drop_motor(&1);
    assert!(driver.inner.borrow().motors.is_empty());
    let m2 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    assert!(driver.inner.borrow().motors.contains_key(&1));

    // prevent from being dropped...
    println!("{:?}", m1);
    println!("{:?}", m2);
}

#[test]
fn add_all_motor() {
    let interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface)).unwrap();

    let _a1 = driver.add_all_motor().unwrap();

    let inner = driver.inner.borrow();
    assert!(inner.all_exists);
    assert!(inner.all.is_none());
}

#[test]
fn add_all_motor_already_exists() {
    let interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface)).unwrap();

    let _a1 = driver.add_all_motor().unwrap();
    let r = driver.add_all_motor();

    assert!(matches!(
        r,
        Err(DriverError::AlreadyExists(MotorAddress::All))
    ));
    assert!(driver.inner.borrow().all_exists);
}

#[test]
fn add_all_motor_multiple_times() {
    let interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface)).unwrap();

    let a1 = driver.add_all_motor().unwrap();
    // this is artificial, in the real world this would only happen if motor
    // got dropped
    driver.inner.borrow_mut().drop_all_motor();
    assert!(!driver.inner.borrow().all_exists);
    let a2 = driver.add_all_motor().unwrap();
    assert!(driver.inner.borrow().all_exists);

    // prevent from being dropped...
    println!("{:?}", a1);
    println!("{:?}", a2);
}
