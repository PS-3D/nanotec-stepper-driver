use super::{super::DriverError, ResponseError, ResponseHandle};
use std::{error::Error, marker::PhantomData};

pub(in super::super) struct MapResponseHandle<H, T, EF, ER, F, OT>
where
    H: ResponseHandle<EF, ER, Ret = T>,
    EF: Error + Into<DriverError>,
    ER: Error + Into<DriverError>,
    F: FnOnce(T) -> OT,
{
    handle: H,
    f: F,
    markert: PhantomData<T>,
    markeref: PhantomData<EF>,
    markerer: PhantomData<ER>,
    markerof: PhantomData<OT>,
}

impl<H, T, EF, ER, F, OT> MapResponseHandle<H, T, EF, ER, F, OT>
where
    H: ResponseHandle<EF, ER, Ret = T>,
    EF: Error + Into<DriverError>,
    ER: Error + Into<DriverError>,
    F: FnOnce(T) -> OT,
{
    pub fn new(handle: H, f: F) -> Self {
        Self {
            handle,
            f,
            markert: PhantomData,
            markeref: PhantomData,
            markerer: PhantomData,
            markerof: PhantomData,
        }
    }
}

impl<H, T, EF, ER, F, OT> ResponseHandle<EF, ER> for MapResponseHandle<H, T, EF, ER, F, OT>
where
    H: ResponseHandle<EF, ER, Ret = T>,
    EF: Error + Into<DriverError>,
    ER: Error + Into<DriverError>,
    F: FnOnce(T) -> OT,
{
    type Ret = OT;

    fn wait(self) -> Result<OT, ResponseError<Self, OT, EF, ER>> {
        // can't use map on the result due to ownership
        match self.handle.wait() {
            Ok(t) => Ok((self.f)(t)),
            Err(e) => Err(e.map_handle(|h| MapResponseHandle::new(h, self.f))),
        }
    }
}

pub(in super::super) trait ResponseHandleMap<T, EF, ER>:
    ResponseHandle<EF, ER, Ret = T>
where
    EF: Error + Into<DriverError>,
    ER: Error + Into<DriverError>,
{
    fn map<F, OT>(self, f: F) -> MapResponseHandle<Self, T, EF, ER, F, OT>
    where
        Self: Sized,
        F: FnOnce(T) -> OT,
    {
        MapResponseHandle::new(self, f)
    }
}

impl<H, T, EF, ER> ResponseHandleMap<T, EF, ER> for H
where
    H: ResponseHandle<EF, ER, Ret = T>,
    EF: Error + Into<DriverError>,
    ER: Error + Into<DriverError>,
{
}
