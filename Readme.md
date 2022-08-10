# nanotec-stepper-driver

This crate lets you talk to your [Nanotec](https://en.nanotec.com/) stepper-motor controllers with rust.\
It is written after the [Programming Manual V2.7](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf) and compatible with
firmware versions 25.01.2013 and newer.

## Usage

Controling your motors is easy, just like you would from Java:
```rust

use nanotec_stepper_driver::{Driver, ResponseHandle, RespondMode};
use std::time::Duration;
use serialport;

let s = serialport::new("/dev/ttyUSB0", 115200)
    .timeout(Duration::from_secs(1))
    .open()
    .unwrap();
let mut driver = Driver::new(s).unwrap();
let mut m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();

m1.load_record(3).unwrap().wait().unwrap();
m1.set_continuation_record(None).unwrap().wait().unwrap();
m1.start_motor().unwrap().wait().unwrap();

let steps = m1.get_travel_distance().unwrap().wait().unwrap();
let max_freq = m1.get_max_frequency().unwrap().wait().unwrap();
println!("Driving {} steps with a top speed of {} steps/min", steps, max_freq);
```

Some commands like [1.6.4 Reading out the current record](https://en.nanotec.com/fileadmin/files/Handbuecher/Programmierung/Programming_Manual_V2.7.pdf) were simplified or split to make using them easier and more intuitive:
```rust
use nanotec_stepper_driver::{Driver, ResponseHandle, RespondMode};
use std::time::Duration;
use serialport;

let s = serialport::new("/dev/ttyUSB0", 115200)
    .timeout(Duration::from_secs(1))
    .open()
    .unwrap();
let mut driver = Driver::new(s).unwrap();
let mut m1 = driver.add_motor(1, RespondMode::NotQuiet).unwrap();

let cur_record = m1.get_current_record().unwrap().wait().unwrap();
println!("Current record: {:?}", record);

let record_3 = m1.get_record(3).unwrap().wait().unwrap();
println!("Record 3: {:?}", record_3);

m1.set_respond_mode(RespondMode::Quiet).unwrap().wait().unwrap();
println!("Now the motor won't respond to commands anymore");
```

## Release

This library is not really released yet, this is a **prerelease**.
There are a lot of commands that aren't implemented yet, as well as some other features we would like to implement.
And the library could use a bit of optimisation in the back as well.
The public interface should be somewhat final though, but don't count on it.

If you desperately need this library to be finished or if you need a specific feature, consider contributing or make an issue.

