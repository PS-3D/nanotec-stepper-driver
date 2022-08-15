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
    let inner = &driver.inner;
    {
        let mut data = inner.data.lock().unwrap();
        data.motors.get_mut(&1).unwrap().available = false;
    }
    interface.add_read(b"1A\r");

    let r = inner.receive_single(1).unwrap();

    assert!(interface.is_empty());
    assert_eq!(&r, b"A");
    {
        let data = inner.data.lock().unwrap();
        assert!(data.motors.get(&1).unwrap().available);
        assert!(data.motors.get(&1).unwrap().queue.is_empty());
    }
}

#[test]
fn receive_single_multiple() {
    let mut interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface.clone())).unwrap();
    let _m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    let _m2 = driver.add_motor(2, RespondMode::NotQuiet).unwrap();
    let inner = &driver.inner;
    {
        let mut data = inner.data.lock().unwrap();
        data.motors.get_mut(&1).unwrap().available = false;
        data.motors.get_mut(&2).unwrap().available = false;
    }
    interface.add_read(b"2A\r");
    interface.add_read(b"1A\r");

    let r = inner.receive_single(1).unwrap();

    assert!(interface.is_empty());
    assert_eq!(&r, b"A");
    {
        let data = inner.data.lock().unwrap();
        assert!(data.motors.get(&2).unwrap().available);
        assert_eq!(data.motors.get(&2).unwrap().queue.front().unwrap(), b"A");
    }
}

#[test]
fn receive_single_other_motor() {
    let mut interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface.clone())).unwrap();
    let _m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    let inner = &driver.inner;
    interface.add_read(b"2A\r");

    let r = inner.receive_single(1);

    assert!(matches!(
        r,
        Err(DriverError::UnexpectedResponse(MotorAddress::Single(2)))
    ));
    assert!(interface.is_empty());
    {
        let data = inner.data.lock().unwrap();
        assert!(data.motors.get(&1).unwrap().available);
        assert!(data.motors.get(&1).unwrap().queue.is_empty());
    }
}

#[test]
fn receive_single_all() {
    let mut interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface.clone())).unwrap();
    let _m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    let inner = &driver.inner;
    interface.add_read(b"*A\r");

    let r = inner.receive_single(1);

    assert!(matches!(
        r,
        Err(DriverError::UnexpectedResponse(MotorAddress::All))
    ));
    assert!(interface.is_empty());
    {
        let data = inner.data.lock().unwrap();
        assert!(data.motors.get(&1).unwrap().available);
        assert!(data.motors.get(&1).unwrap().queue.is_empty());
    }
}

#[test]
fn receive_all() {
    let mut interface = Interface::new();
    let driver = Driver::new(Box::new(interface.clone())).unwrap();
    let inner = &driver.inner;
    interface.add_read(b"*A\r");
    interface.add_read(b"*A\r");
    {
        let mut data = inner.data.lock().unwrap();
        data.all = Some((2, b"A".to_vec()));
    }

    let r = inner.receive_all().unwrap();

    assert!(interface.is_empty());
    assert_eq!(&r, b"A");
    {
        let data = inner.data.lock().unwrap();
        assert!(data.all.is_none());
    }
}

#[test]
fn receive_all_single() {
    let mut interface = Interface::new();
    let driver = Driver::new(Box::new(interface.clone())).unwrap();
    let inner = &driver.inner;
    interface.add_read(b"1A\r");
    {
        let mut data = inner.data.lock().unwrap();
        data.all = Some((1, b"A".to_vec()));
    }

    let r = inner.receive_all();

    assert!(matches!(
        r,
        Err(DriverError::UnexpectedResponse(MotorAddress::Single(1)))
    ));
    assert!(interface.is_empty());
    {
        let data = inner.data.lock().unwrap();
        assert_eq!(data.all.as_ref().unwrap().0, 1);
        assert_eq!(data.all.as_ref().unwrap().1, b"A".to_vec());
    }
}

#[test]
fn send_single_no_response() {
    let mut interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface.clone())).unwrap();
    let _m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    let inner = &driver.inner;
    interface.add_write(b"#1A\r");

    inner.send_single_no_response(1, format_args!("A")).unwrap();

    {
        let data = inner.data.lock().unwrap();
        assert!(data.motors.get(&1).unwrap().available);
    }
}

#[test]
fn send_single_no_response_waiting_all() {
    let interface = Interface::new();
    let driver = Driver::new(Box::new(interface.clone())).unwrap();
    let inner = &driver.inner;
    {
        let mut data = inner.data.lock().unwrap();
        data.all = Some((1, b"A".to_vec()));
    }

    let r = inner.send_single_no_response(1, format_args!("test"));

    assert!(matches!(r, Err(DriverError::NotAvailable)));
}

#[test]
fn send_single_no_response_waiting() {
    let interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface.clone())).unwrap();
    let _m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    let inner = &driver.inner;
    {
        let mut data = inner.data.lock().unwrap();
        data.motors.get_mut(&1).unwrap().available = false;
    }

    let r = inner.send_single_no_response(1, format_args!("test"));

    assert!(matches!(r, Err(DriverError::NotAvailable)));
    {
        let data = inner.data.lock().unwrap();
        assert!(!data.motors.get(&1).unwrap().available);
    }
}

#[test]
fn send_single_with_response() {
    let mut interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface.clone())).unwrap();
    let _m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    let inner = &driver.inner;
    interface.add_write(b"#1A\r");

    inner
        .send_single_with_response(1, format_args!("A"))
        .unwrap();

    {
        let data = inner.data.lock().unwrap();
        assert!(!data.motors.get(&1).unwrap().available);
    }
}

#[test]
fn send_single_with_response_waiting_all() {
    let interface = Interface::new();
    let driver = Driver::new(Box::new(interface.clone())).unwrap();
    let inner = &driver.inner;
    {
        let mut data = inner.data.lock().unwrap();
        data.all = Some((1, b"A".to_vec()));
    }

    let r = inner.send_single_with_response(1, format_args!("test"));

    assert!(matches!(r, Err(DriverError::NotAvailable)));
}

#[test]
fn send_single_with_response_waiting() {
    let interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface.clone())).unwrap();
    let _m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    let inner = &driver.inner;
    {
        let mut data = inner.data.lock().unwrap();
        data.motors.get_mut(&1).unwrap().available = false;
    }

    let r = inner.send_single_with_response(1, format_args!("test"));

    assert!(matches!(r, Err(DriverError::NotAvailable)));
    {
        let data = inner.data.lock().unwrap();
        assert!(!data.motors.get(&1).unwrap().available);
    }
}

#[test]
fn send_all_not_quiet() {
    let mut interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface.clone())).unwrap();
    let _m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    let _m2 = driver.add_motor(2, RespondMode::Quiet).unwrap();
    let inner = &driver.inner;
    interface.add_write(b"#*A\r");

    let r = inner.send_all(format_args!("A")).unwrap();

    assert_eq!(r, RespondMode::NotQuiet);
    {
        let data = inner.data.lock().unwrap();
        assert_eq!(data.all.as_ref().unwrap().0, 1);
        assert_eq!(data.all.as_ref().unwrap().1, b"A".to_vec());
    }
}

#[test]
fn send_all_quiet() {
    let mut interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface.clone())).unwrap();
    let _m1 = driver.add_motor(1, RespondMode::Quiet).unwrap();
    let inner = &driver.inner;
    interface.add_write(b"#*A\r");

    let r = inner.send_all(format_args!("A")).unwrap();

    assert_eq!(r, RespondMode::Quiet);
    {
        let data = inner.data.lock().unwrap();
        assert!(data.all.is_none());
    }
}

#[test]
fn send_all_waiting() {
    let interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface)).unwrap();
    let _m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();
    let _m2 = driver.add_motor(2, RespondMode::Quiet).unwrap();
    let inner = &driver.inner;
    {
        let mut data = inner.data.lock().unwrap();
        data.motors.get_mut(&1).unwrap().available = false;
    }

    let r = inner.send_all(format_args!("test"));

    assert!(matches!(r, Err(DriverError::NotAvailable)));
    {
        let data = inner.data.lock().unwrap();
        assert!(data.all.is_none());
    }
}

#[test]
fn send_all_all_waiting() {
    let interface = Interface::new();
    let driver = Driver::new(Box::new(interface)).unwrap();
    let inner = &driver.inner;
    {
        let mut data = inner.data.lock().unwrap();
        data.all = Some((1, b"test".to_vec()));
    }

    let r = inner.send_all(format_args!("test"));

    assert!(matches!(r, Err(DriverError::NotAvailable)));
}

#[test]
fn add_motor() {
    let interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface)).unwrap();

    let _m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();

    let inner = &driver.inner;
    let data = inner.data.lock().unwrap();
    let im1 = data.motors.get(&1).unwrap();
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
    let inner = &driver.inner;
    let data = inner.data.lock().unwrap();
    assert_eq!(data.motors.len(), 1);
}

#[test]
fn add_motor_multiple_times() {
    let interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface)).unwrap();

    let m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();

    {
        let inner = &driver.inner;

        // this is artificial, in the real world this would only happen if motor
        // got dropped
        inner.drop_motor(&1);

        let data = inner.data.lock().unwrap();
        assert!(data.motors.is_empty());
    }

    let m2 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();

    {
        let inner = &driver.inner;
        let data = inner.data.lock().unwrap();
        assert!(data.motors.contains_key(&1));
    }

    // prevent from being dropped...
    println!("{:?}", m1);
    println!("{:?}", m2);
}

#[test]
fn add_all_motor() {
    let interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface)).unwrap();

    let _a1 = driver.add_all_motor().unwrap();

    let inner = &driver.inner;
    let data = inner.data.lock().unwrap();
    assert!(data.all_exists);
    assert!(data.all.is_none());
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
    let inner = &driver.inner;
    let data = inner.data.lock().unwrap();
    assert!(data.all_exists);
}

#[test]
fn add_all_motor_multiple_times() {
    let interface = Interface::new();
    let mut driver = Driver::new(Box::new(interface)).unwrap();

    let a1 = driver.add_all_motor().unwrap();

    {
        let inner = &driver.inner;

        // this is artificial, in the real world this would only happen if motor
        // got dropped
        inner.drop_all_motor();

        let data = inner.data.lock().unwrap();
        assert!(!data.all_exists);
    }

    let a2 = driver.add_all_motor().unwrap();

    {
        let inner = &driver.inner;
        let data = inner.data.lock().unwrap();
        assert!(data.all_exists);
    }

    // prevent from being dropped...
    println!("{:?}", a1);
    println!("{:?}", a2);
}
