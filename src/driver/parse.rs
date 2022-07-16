use nom::{
    bytes::complete::tag,
    character::complete::{u16 as parse_u16, u32 as parse_u32, u64 as parse_u64, u8 as parse_u8},
    combinator::opt,
    sequence::preceded,
    IResult, Parser,
};
use std::fmt::Debug;
use thiserror::Error;

/// Gets thrown when there is an error while parsing the various enums which
/// represent values for the commands
#[derive(Error, Debug)]
pub enum ParseError<I: Debug> {
    /// Gets thrown when a a value of a command doesn't have a matching enum variant,
    /// which usually means it's too big.
    #[error("Invalid Value while Parsing, probably too big")]
    InvalidValue,
    /// Wrapper around [`nom::error::Error`]
    #[error("nom error: {0:?}")]
    NomError(nom::error::Error<I>),
}

impl<I: Debug> nom::error::ParseError<I> for ParseError<I> {
    fn from_error_kind(input: I, kind: nom::error::ErrorKind) -> Self {
        Self::NomError(nom::error::Error::from_error_kind(input, kind))
    }

    /// basically copied from nom::error::Error::append
    fn append(_: I, _: nom::error::ErrorKind, other: Self) -> Self {
        other
    }
}

impl<I: Debug> From<nom::error::Error<I>> for ParseError<I> {
    fn from(e: nom::error::Error<I>) -> Self {
        Self::NomError(e)
    }
}

fn parse_s<'a, P, T, E>(p: P) -> impl FnMut(&'a [u8]) -> IResult<&'a [u8], T, E>
where
    P: Parser<&'a [u8], T, E>,
    E: nom::error::ParseError<&'a [u8]>,
{
    preceded(opt(tag(b"+")), p)
}

pub(super) fn parse_su8<'a, E>(s: &'a [u8]) -> IResult<&'a [u8], u8, E>
where
    E: nom::error::ParseError<&'a [u8]>,
{
    parse_s(parse_u8)(s)
}
pub(super) fn parse_su16<'a, E>(s: &'a [u8]) -> IResult<&'a [u8], u16, E>
where
    E: nom::error::ParseError<&'a [u8]>,
{
    parse_s(parse_u16)(s)
}

pub(super) fn parse_su32<'a, E>(s: &'a [u8]) -> IResult<&'a [u8], u32, E>
where
    E: nom::error::ParseError<&'a [u8]>,
{
    parse_s(parse_u32)(s)
}

pub(super) fn parse_su64<'a, E>(s: &'a [u8]) -> IResult<&'a [u8], u64, E>
where
    E: nom::error::ParseError<&'a [u8]>,
{
    parse_s(parse_u64)(s)
}

#[inline]
pub(super) fn parse_enum_value<'a, P, C, O, O2>(
    s: &'a [u8],
    parser: P,
    constructor: C,
) -> IResult<&'a [u8], O2, ParseError<&'a [u8]>>
where
    P: Fn(&'a [u8]) -> IResult<&'a [u8], O, nom::error::Error<&'a [u8]>>,
    C: Fn(O) -> Option<O2>,
{
    let (rem, res) = parser(s).map_err(|e| nom::Err::convert(e))?;
    Ok((
        rem,
        constructor(res).ok_or(nom::Err::Error(ParseError::InvalidValue))?,
    ))
}
