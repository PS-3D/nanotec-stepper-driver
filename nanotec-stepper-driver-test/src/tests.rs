use super::Interface;
use std::io::{Read, Write};

#[test]
fn read_small_buffer() {
    let mut i = Interface::new();
    let mut buf = [0u8; 8];
    let expected = b"thisisatest";
    i.add_read(expected);
    assert_eq!(i.read(&mut buf).unwrap(), 8);
    assert_eq!(buf, expected[..8]);
    assert_eq!(i.read(&mut buf).unwrap(), expected.len() - 8);
    assert_eq!(buf[..expected.len() - 8], expected[8..]);
}

#[test]
fn read_big_buffer() {
    let mut i = Interface::new();
    let mut buf = [0u8; 64];
    let expected = b"thisisatest";
    i.add_read(expected);
    assert_eq!(i.read(&mut buf).unwrap(), expected.len());
    assert_eq!(&buf[..expected.len()], expected);
}

#[test]
#[should_panic]
fn read_empty() {
    let mut i = Interface::new();
    let mut buf = [0u8; 32];
    let _ = i.read(&mut buf);
}

#[test]
fn write_small_buffer() {
    let mut i = Interface::new();
    let expected = b"thisisatest";
    i.add_write(expected);
    assert_eq!(i.write(&expected[..8]).unwrap(), 8);
    assert_eq!(i.write(&expected[8..]).unwrap(), expected.len() - 8);
}

#[test]
fn write() {
    let mut i = Interface::new();
    let expected = b"thisisatest";
    i.add_write(expected);
    assert_eq!(i.write(expected).unwrap(), expected.len());
}

#[test]
#[should_panic]
fn write_wrong_buf() {
    let mut i = Interface::new();
    i.add_write(b"thisisatest");
    let _ = i.write(b"anothertest");
}

#[test]
#[should_panic]
fn write_empty() {
    let mut i = Interface::new();
    let _ = i.write(b"thisisatest");
}
