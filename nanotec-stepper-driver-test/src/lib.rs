#[cfg(test)]
mod tests;

use std::{
    io::{Read, Write},
    sync::{Arc, RwLock},
};

use serialport::SerialPort;

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
#[derive(Debug)]
pub struct Interface {
    read: Arc<RwLock<Vec<u8>>>,
    write: Arc<RwLock<Vec<u8>>>,
}

impl Read for Interface {
    fn read(&mut self, mut buf: &mut [u8]) -> std::io::Result<usize> {
        let mut q = self.read.as_ref().write().unwrap();
        if q.is_empty() {
            panic!("read was emtpy")
        } else if buf.len() > q.len() {
            buf.write_all(&*q)?;
            let res = q.len();
            q.drain(..);
            Ok(res)
        } else {
            buf.copy_from_slice(&q[..buf.len()]);
            q.drain(..buf.len());
            Ok(buf.len())
        }
    }
}

impl Write for Interface {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let mut q = self.write.as_ref().write().unwrap();
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

#[allow(unused_variables)]
impl SerialPort for Interface {
    fn name(&self) -> Option<String> {
        unimplemented!()
    }

    fn baud_rate(&self) -> serialport::Result<u32> {
        unimplemented!()
    }

    fn data_bits(&self) -> serialport::Result<serialport::DataBits> {
        unimplemented!()
    }

    fn flow_control(&self) -> serialport::Result<serialport::FlowControl> {
        unimplemented!()
    }

    fn parity(&self) -> serialport::Result<serialport::Parity> {
        unimplemented!()
    }

    fn stop_bits(&self) -> serialport::Result<serialport::StopBits> {
        unimplemented!()
    }

    fn timeout(&self) -> std::time::Duration {
        unimplemented!()
    }

    fn set_baud_rate(&mut self, baud_rate: u32) -> serialport::Result<()> {
        unimplemented!()
    }

    fn set_data_bits(&mut self, data_bits: serialport::DataBits) -> serialport::Result<()> {
        unimplemented!()
    }

    fn set_flow_control(
        &mut self,
        flow_control: serialport::FlowControl,
    ) -> serialport::Result<()> {
        unimplemented!()
    }

    fn set_parity(&mut self, parity: serialport::Parity) -> serialport::Result<()> {
        unimplemented!()
    }

    fn set_stop_bits(&mut self, stop_bits: serialport::StopBits) -> serialport::Result<()> {
        unimplemented!()
    }

    fn set_timeout(&mut self, timeout: std::time::Duration) -> serialport::Result<()> {
        unimplemented!()
    }

    fn write_request_to_send(&mut self, level: bool) -> serialport::Result<()> {
        unimplemented!()
    }

    fn write_data_terminal_ready(&mut self, level: bool) -> serialport::Result<()> {
        unimplemented!()
    }

    fn read_clear_to_send(&mut self) -> serialport::Result<bool> {
        unimplemented!()
    }

    fn read_data_set_ready(&mut self) -> serialport::Result<bool> {
        unimplemented!()
    }

    fn read_ring_indicator(&mut self) -> serialport::Result<bool> {
        unimplemented!()
    }

    fn read_carrier_detect(&mut self) -> serialport::Result<bool> {
        unimplemented!()
    }

    fn bytes_to_read(&self) -> serialport::Result<u32> {
        unimplemented!()
    }

    fn bytes_to_write(&self) -> serialport::Result<u32> {
        unimplemented!()
    }

    fn clear(&self, buffer_to_clear: serialport::ClearBuffer) -> serialport::Result<()> {
        unimplemented!()
    }

    fn try_clone(&self) -> serialport::Result<Box<dyn SerialPort>> {
        Ok(Box::new(self.clone()))
    }

    fn set_break(&self) -> serialport::Result<()> {
        unimplemented!()
    }

    fn clear_break(&self) -> serialport::Result<()> {
        unimplemented!()
    }
}

impl Interface {
    pub fn new() -> Self {
        Interface {
            read: Arc::new(RwLock::new(Vec::new())),
            write: Arc::new(RwLock::new(Vec::new())),
        }
    }
    pub fn add_read(&mut self, buf: &[u8]) {
        self.read.as_ref().write().unwrap().extend_from_slice(buf)
    }

    pub fn add_write(&mut self, buf: &[u8]) {
        self.write.as_ref().write().unwrap().extend_from_slice(buf)
    }

    pub fn add_cmd_echo(&mut self, buf: &[u8]) {
        self.add_read(&buf[1..]);
        self.add_write(buf);
    }

    pub fn is_empty(&self) -> bool {
        self.read.read().unwrap().is_empty() && self.write.read().unwrap().is_empty()
    }
}
