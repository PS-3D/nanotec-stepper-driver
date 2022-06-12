#[cfg(test)]
mod tests;

use std::{
    cell::RefCell,
    io::{Read, Write},
    rc::Rc,
};

// used to mock an interface to test the stepper driver
// the idea is that read contains bytes which can be read and write bytes
// that can be written. once they are written/read they are removed from the
// corresponding buffer. if the buffers were emtpy we panic and if the content
// in write didnt match what is being written we panic as well.
// add_read and add_write add bytes to the corresponding buffers
//
// don't be alarmed if you think it's slow or inefficient or anything, it doesn't
// need to be fast nor pretty nor efficient, its just for testing. it needs to be
// easy
pub struct Interface {
    read: Rc<RefCell<Vec<u8>>>,
    write: Rc<RefCell<Vec<u8>>>,
}

impl Read for Interface {
    fn read(&mut self, mut buf: &mut [u8]) -> std::io::Result<usize> {
        let mut q = self.read.as_ref().borrow_mut();
        if q.is_empty() {
            panic!("read was emtpy")
        } else if buf.len() > q.len() {
            buf.write_all(&*q)?;
            let res = q.len();
            q.drain(..);
            Ok(res)
        } else {
            buf.write_all(&q[..buf.len()])?;
            q.drain(..buf.len());
            Ok(buf.len())
        }
    }
}

impl Write for Interface {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let mut q = self.write.as_ref().borrow_mut();
        if q.is_empty() {
            panic!("write was emtpy")
        } else if q.starts_with(buf) {
            q.drain(..buf.len());
            Ok(buf.len())
        } else {
            panic!("write didn't start with {:?}, write was {:?}", buf, q)
        }
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

impl Clone for Interface {
    fn clone(&self) -> Self {
        Interface {
            read: self.read.clone(),
            write: self.write.clone(),
        }
    }
}

impl Interface {
    pub fn new() -> Self {
        Interface {
            read: Rc::new(RefCell::new(Vec::new())),
            write: Rc::new(RefCell::new(Vec::new())),
        }
    }
    pub fn add_read(&mut self, buf: &[u8]) {
        self.read.as_ref().borrow_mut().extend_from_slice(buf)
    }

    pub fn add_write(&mut self, buf: &[u8]) {
        self.write.as_ref().borrow_mut().extend_from_slice(buf)
    }

    pub fn add_cmd_echo(&mut self, buf: &[u8]) {
        self.add_read(&buf[1..]);
        self.add_write(buf);
    }
}
