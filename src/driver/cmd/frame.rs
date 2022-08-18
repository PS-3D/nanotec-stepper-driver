#[cfg(test)]
mod tests;

use super::super::parse::ParseError;
use nom::{self, character::complete::u8 as parse_u8, IResult, Parser};
use std::{
    error::Error,
    fmt::{Debug, Display},
};

// unfortunately, due to rustfmt not having the blank_lines_upper_bound feature
// stable yet, we gotta put comments in between the different sections. otherwise
// its just too much

//

/// Holds the address of a motor
///
/// Used to disscern whether something is addressed to a specific motor or to all
/// motors.
#[derive(Debug, PartialEq, Eq, Hash, Copy, Clone)]
pub enum MotorAddress {
    All,
    Single(u8),
}

impl MotorAddress {
    pub(crate) fn parse(s: &[u8]) -> IResult<&[u8], Self, ParseError<&[u8]>> {
        use nom::{branch::alt, bytes::complete::tag};
        alt((
            tag("*").map(|_| Self::All),
            parse_u8.map(|a| Self::Single(a)),
        ))(s)
    }

    pub fn is_all(&self) -> bool {
        matches!(self, Self::All)
    }

    pub fn is_single(&self) -> bool {
        matches!(self, Self::Single(_))
    }

    pub fn single(self) -> u8 {
        match self {
            Self::All => panic!("single() called on All"),
            Self::Single(a) => a,
        }
    }

    pub fn single_or<E: Error>(self, error: E) -> Result<u8, E> {
        match self {
            Self::All => Err(error),
            Self::Single(a) => Ok(a),
        }
    }

    pub fn single_expect(self, msg: &str) -> u8 {
        match self {
            Self::All => panic!("{}", msg),
            Self::Single(a) => a,
        }
    }
}

impl Display for MotorAddress {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::All => write!(f, "*"),
            Self::Single(a) => write!(f, "{}", a),
        }
    }
}

impl From<Option<u8>> for MotorAddress {
    fn from(a: Option<u8>) -> Self {
        match a {
            Some(a) => Self::Single(a),
            None => Self::All,
        }
    }
}

impl From<MotorAddress> for Option<u8> {
    fn from(a: MotorAddress) -> Self {
        match a {
            MotorAddress::Single(a) => Some(a),
            MotorAddress::All => None,
        }
    }
}

impl From<u8> for MotorAddress {
    fn from(a: u8) -> Self {
        Self::Single(a)
    }
}

//

#[derive(Debug, PartialEq, Eq, Hash, Clone)]
pub(crate) struct Msg {
    pub address: MotorAddress,
    pub payload: Vec<u8>,
}

impl Msg {
    pub fn parse(s: &[u8]) -> IResult<&[u8], Self, ParseError<&[u8]>> {
        use nom::{
            bytes::complete::{tag, take_until1},
            sequence::{terminated, tuple},
        };
        // don't parse hashtags cause the motors dont send hashtags
        tuple((
            MotorAddress::parse,
            terminated(take_until1(b"\r" as &[u8]), tag(b"\r")),
        ))
        .map(|(a, p)| Self {
            address: a,
            payload: p.to_vec(),
        })
        .parse(s)
        .map_err(|e| nom::Err::convert(e))
    }
}

//

#[derive(Debug, PartialEq, Eq, Hash, Clone)]
pub(crate) enum MsgWrap {
    Valid(Msg),
    Invalid(Msg),
}

impl MsgWrap {
    pub fn parse(s: &[u8]) -> IResult<&[u8], Self, ParseError<&[u8]>> {
        Msg::parse
            .map(|msg| {
                if msg.payload.ends_with(b"?") {
                    Self::Invalid(msg)
                } else {
                    Self::Valid(msg)
                }
            })
            .parse(s)
    }
}
