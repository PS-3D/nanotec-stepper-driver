use super::{MotorAddress, Msg, MsgWrap};
use nom::Finish;

// unfortunately, due to rustfmt not having the blank_lines_upper_bound feature
// stable yet, we gotta put comments in between the different sections. otherwise
// its just too much

//

#[test]
fn msg_parse_long_write() {
    let (_, m) = Msg::parse(b"1:Keyword=1337\r").finish().unwrap();
    let expected = Msg {
        address: MotorAddress::Single(1),
        payload: b":Keyword=1337".to_vec(),
    };
    assert_eq!(m, expected)
}

#[test]
fn msg_parse_long_read() {
    let (_, m) = Msg::parse(b"1:CL_motor_type+1\r").finish().unwrap();
    let expected = Msg {
        address: MotorAddress::Single(1),
        payload: b":CL_motor_type+1".to_vec(),
    };
    assert_eq!(m, expected)
}

#[test]
fn msg_parse_short_no_arg() {
    let (_, m) = Msg::parse(b"1A\r").finish().unwrap();
    let expected = Msg {
        address: MotorAddress::Single(1),
        payload: b"A".to_vec(),
    };
    assert_eq!(m, expected)
}

#[test]
fn msg_parse_all() {
    let (_, m) = Msg::parse(b"*S+1\r").finish().unwrap();
    let expected = Msg {
        address: MotorAddress::All,
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

//

#[test]
fn msgwrap_parse_valid() {
    let (_, w) = MsgWrap::parse(b"1A\r").finish().unwrap();
    assert_eq!(
        w,
        MsgWrap::Valid(Msg {
            address: MotorAddress::Single(1),
            payload: b"A".to_vec(),
        })
    )
}

#[test]
fn msgwrap_parse_invalid() {
    let (_, w) = MsgWrap::parse(b"1A?\r").finish().unwrap();
    assert_eq!(
        w,
        MsgWrap::Invalid(Msg {
            address: MotorAddress::Single(1),
            payload: b"A?".to_vec(),
        })
    )
}
