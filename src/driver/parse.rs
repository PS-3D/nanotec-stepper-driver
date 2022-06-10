use nom::{
    bytes::complete::tag,
    character::complete::{u16 as parse_u16, u32 as parse_u32, u8 as parse_u8},
    combinator::opt,
    sequence::preceded,
    IResult, Parser,
};

fn parse_s<'a, P, T, E>(p: P) -> impl FnMut(&'a [u8]) -> IResult<&'a [u8], T, E>
where
    P: Parser<&'a [u8], T, E>,
    E: nom::error::ParseError<&'a [u8]>,
{
    preceded(opt(tag(b"+")), p)
}

pub fn parse_su8<'a, E>(s: &'a [u8]) -> IResult<&'a [u8], u8, E>
where
    E: nom::error::ParseError<&'a [u8]>,
{
    parse_s(parse_u8)(s)
}
pub fn parse_su16<'a, E>(s: &'a [u8]) -> IResult<&'a [u8], u16, E>
where
    E: nom::error::ParseError<&'a [u8]>,
{
    parse_s(parse_u16)(s)
}

pub fn parse_su32<'a, E>(s: &'a [u8]) -> IResult<&'a [u8], u32, E>
where
    E: nom::error::ParseError<&'a [u8]>,
{
    parse_s(parse_u32)(s)
}
