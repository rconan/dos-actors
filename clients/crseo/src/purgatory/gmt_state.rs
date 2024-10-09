use std::vec::IntoIter;

#[cfg(feature = "apache-arrow")]
type MaybeData = Result<Vec<Vec<f64>>, crate::clients::arrow_client::ArrowError>;

#[allow(dead_code)]
#[derive(Debug, Default)]
pub struct GmtState {
    m1_rbm: Option<IntoIter<Vec<f64>>>,
    m2_rbm: Option<IntoIter<Vec<f64>>>,
    m1_mode: Option<IntoIter<Vec<f64>>>,
}
#[cfg(feature = "apache-arrow")]
impl From<crate::clients::arrow_client::Arrow> for GmtState {
    fn from(mut logs: crate::clients::arrow_client::Arrow) -> Self {
        use super::arrow_client::Get;
        let m1_rbm: MaybeData = logs.get("OSSM1Lcl");
        let m2_rbm: MaybeData = logs.get("MCM2Lcl6D");
        let m1_mode: MaybeData = logs.get("M1modes");
        Self {
            m1_rbm: m1_rbm.map(|x| x.into_iter()).ok(),
            m2_rbm: m2_rbm.map(|x| x.into_iter()).ok(),
            m1_mode: m1_mode.map(|x| x.into_iter()).ok(),
        }
    }
}
#[cfg(feature = "apache-arrow")]
impl From<(crate::clients::arrow_client::Arrow, usize, Option<usize>)> for GmtState {
    fn from(
        (mut logs, skip, take): (crate::clients::arrow_client::Arrow, usize, Option<usize>),
    ) -> Self {
        use super::arrow_client::Get;
        let m1_rbm: MaybeData = logs.get_skip_take("OSSM1Lcl", skip, take);
        let m2_rbm: MaybeData = logs.get_skip_take("MCM2Lcl6D", skip, take);
        let m1_mode: MaybeData = logs.get_skip_take("M1modes", skip, take);
        Self {
            m1_rbm: m1_rbm.map(|x| x.into_iter()).ok(),
            m2_rbm: m2_rbm.map(|x| x.into_iter()).ok(),
            m1_mode: m1_mode.map(|x| x.into_iter()).ok(),
        }
    }
}

impl crate::Update for GmtState {}
#[cfg(feature = "fem")]
impl crate::io::Write<fem::fem_io::OSSM1Lcl> for GmtState {
    fn write(&mut self) -> Option<std::sync::Arc<crate::io::Data<fem::fem_io::OSSM1Lcl>>> {
        self.m1_rbm
            .as_mut()
            .and_then(|x| x.next())
            .map(|x| std::sync::Arc::new(crate::io::Data::new(x)))
    }
}
#[cfg(feature = "fem")]
impl crate::io::Write<fem::fem_io::MCM2Lcl6D> for GmtState {
    fn write(&mut self) -> Option<std::sync::Arc<crate::io::Data<fem::fem_io::MCM2Lcl6D>>> {
        self.m2_rbm
            .as_mut()
            .and_then(|x| x.next())
            .map(|x| std::sync::Arc::new(crate::io::Data::new(x)))
    }
}
#[cfg(feature = "ceo")]
impl crate::io::Write<crate::clients::ceo::M1rbm> for GmtState {
    fn write(&mut self) -> Option<std::sync::Arc<crate::io::Data<crate::clients::ceo::M1rbm>>> {
        self.m1_rbm
            .as_mut()
            .and_then(|x| x.next())
            .map(|x| std::sync::Arc::new(crate::io::Data::new(x)))
    }
}
#[cfg(feature = "ceo")]
impl crate::io::Write<crate::clients::ceo::M2rbm> for GmtState {
    fn write(&mut self) -> Option<std::sync::Arc<crate::io::Data<crate::clients::ceo::M2rbm>>> {
        self.m2_rbm
            .as_mut()
            .and_then(|x| x.next())
            .map(|x| std::sync::Arc::new(crate::io::Data::new(x)))
    }
}
#[cfg(feature = "ceo")]
impl crate::io::Write<crate::clients::ceo::M1modes> for GmtState {
    fn write(&mut self) -> Option<std::sync::Arc<crate::io::Data<crate::clients::ceo::M1modes>>> {
        self.m1_mode
            .as_mut()
            .and_then(|x| x.next())
            .map(|x| std::sync::Arc::new(crate::io::Data::new(x)))
    }
}
