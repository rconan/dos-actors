#[macro_export]
macro_rules! count {
    () => (0usize);
    ( $x:tt $($xs:tt)* ) => (1usize + count!($($xs)*));
}

/**
Macro to build model

## Example
```ignore
model!(actor1, actor2, ...)
```
*/
#[macro_export]
macro_rules! model {
    ($($x:expr_2021),*) => ($crate::model::Model::new((vec![$(Box::new($x)),*])));
}

/* #[macro_export]
macro_rules! impl_update {
    ($module:ident) => {
        impl<'a> Update for $module::Controller<'a> {
            fn update(&mut self) {
                log::debug!("update");
                self.next();
            }
        }
    };
}
#[macro_export]
macro_rules! impl_read {
    ($module:ident, ($var:ident, $val:ident)) => {
        impl<'a> Read<$var> for $module::Controller<'a> {
            fn read(&mut self, data: Arc<Data<$var>>) {
                let $module::U::$var(val) = &mut self.$val;
                assert_eq!(
                    data.len(),
                    val.len(),
                    "data size ({}) do not match $ident size ({})",
                    data.len(),
                    val.len()
                );
                unsafe { ptr::copy_nonoverlapping((**data).as_ptr(), val.as_mut_ptr(), val.len()) }
            }
        }
    };
    ($data:ty, $module:ident, ($var:ident, $val:ident)) => {
        impl<'a> Read<$data> for $module::Controller<'a> {
            fn read(&mut self, data: Arc<Data<$data>>) {
                if let $module::U::$var(val) = &mut self.$val {
                    assert_eq!(
            data.len(),
            val.len(),
            "data size ({}) do not match $ident size ({})",
            data.len(),
            val.len()
                    );
                    unsafe { ptr::copy_nonoverlapping((**data).as_ptr(), val.as_mut_ptr(), val.len()) }
        }
            }
        }
    };
    ($module:ident, ($data:ty, $var:ident, $val:ident)) => {
        impl<'a> Read<$data> for $module::Controller<'a> {
            fn read(&mut self, data: Arc<Data<$data>>) {
                if let $module::U::$var(val) = &mut self.$val {
                    assert_eq!(
            data.len(),
            val.len(),
            "data size ({}) do not match $ident size ({})",
            data.len(),
            val.len()
                    );
                    unsafe { ptr::copy_nonoverlapping((**data).as_ptr(), val.as_mut_ptr(), val.len()) }
        }
            }
        }
    };
    ($module:ident, ($data:ty, $var:ident, $val:ident), $(($datao:ty, $varo:ident, $valo:ident)),+) => {
        impl<'a> Read<$data> for $module::Controller<'a> {
            fn read(&mut self, data: Arc<Data<$data>>) {
                if let $module::U::$var(val) = &mut self.$val {
                    assert_eq!(
                        data.len(),
                        val.len(),
                        "data size ({}) do not match $ident size ({})",
                        data.len(),
                        val.len()
                    );
                    unsafe { ptr::copy_nonoverlapping((**data).as_ptr(), val.as_mut_ptr(), val.len()) }
                }
            }
        }
    $(
        impl<'a> Read<$datao> for $module::Controller<'a> {
            fn read(&mut self, data: Arc<Data<$datao>>) {
                if let $module::U::$varo(val) = &mut self.$valo {
                    assert_eq!(
                        data.len(),
                        val.len(),
                        "data size ({}) do not match $ident size ({})",
                        data.len(),
                        val.len()
                    );
                    unsafe { ptr::copy_nonoverlapping((**data).as_ptr(), val.as_mut_ptr(), val.len()) }
                }
            }
        }
    )+
    };
    ($module:ident, ($var:ident, $val:ident), $(($varo:ident, $valo:ident)),+) => {
        impl<'a> Read<$var> for $module::Controller<'a> {
            fn read(&mut self, data: Arc<Data<$var>>) {
                if let $module::U::$var(val) = &mut self.$val {
                    assert_eq!(
                        data.len(),
                        val.len(),
                        "data size ({}) do not match $ident size ({})",
                        data.len(),
                        val.len()
                    );
                    unsafe {
                        ptr::copy_nonoverlapping((**data).as_ptr(), val.as_mut_ptr(), val.len())
                    }
                }
            }
        }
    $(
        impl<'a> Read<$varo> for $module::Controller<'a> {
            fn read(&mut self, data: Arc<Data<$varo>>) {
                if let $module::U::$varo(val) = &mut self.$valo {
                    assert_eq!(
                        data.len(),
                        val.len(),
                        "data size ({}) do not match $ident size ({})",
                        data.len(),
                        val.len()
                    );
                    unsafe {
                        ptr::copy_nonoverlapping((**data).as_ptr(), val.as_mut_ptr(), val.len())
                    }
                }
            }
        }
    )+
    };
}
#[macro_export]
macro_rules! impl_write {
    ($module:ident, ($var:ident, $val:ident)) => {
        impl<'a> Write<$var> for $module::Controller<'a> {
            fn write(&mut self) -> Option<Arc<Data<$var>>> {
                let $module::Y::$var(val) = &mut self.$val;
                let mut data = vec![0f64; val.len()];
                unsafe { ptr::copy_nonoverlapping(val.as_ptr(), data.as_mut_ptr(), data.len()) }
                Some(Arc::new(Data::new(data)))
            }
        }
    };
    ($data:ty, $module:ident, ($var:ident, $val:ident)) => {
        impl<'a> Write<$data> for $module::Controller<'a> {
            fn write(&mut self) -> Option<Arc<Data<$data>>> {
                if let $module::Y::$var(val) = &mut self.$val {
                let mut data = vec![0f64; val.len()];
                unsafe { ptr::copy_nonoverlapping(val.as_ptr(), data.as_mut_ptr(), data.len()) }
                Some(Arc::new(Data::new(data)))} else {None}
            }
        }
    };
    ($module:ident, ($data:ty, $var:ident, $val:ident)) => {
        impl<'a> Write<$data> for $module::Controller<'a> {
            fn write(&mut self) -> Option<Arc<Data<$data>>> {
                let $module::Y::$var(val) = &mut self.$val;
                let mut data = vec![0f64; val.len()];
                unsafe { ptr::copy_nonoverlapping(val.as_ptr(), data.as_mut_ptr(), data.len()) }
                Some(Arc::new(Data::new(data)))
            }
        }
    };
    ($module:ident, ($data:ty, $var:ident, $val:ident), $(($datao:ty, $varo:ident, $valo:ident)),+) => {
        impl<'a> Write<$data> for $module::Controller<'a> {
            fn write(&mut self) -> Option<Arc<Data<$data>>> {
                if let $module::Y::$var(val) = &mut self.$val {
                let mut data = vec![0f64; val.len()];
                unsafe { ptr::copy_nonoverlapping(val.as_ptr(), data.as_mut_ptr(), data.len()) }
                Some(Arc::new(Data::new(data)))} else {None}
            }
        }
    $(
        impl<'a> Write<$datao> for $module::Controller<'a> {
            fn write(&mut self) -> Option<Arc<Data<$datao>>> {
                if let $module::Y::$varo(val) = &mut self.$valo {
                let mut data = vec![0f64; val.len()];
                unsafe { ptr::copy_nonoverlapping(val.as_ptr(), data.as_mut_ptr(), data.len()) }
                Some(Arc::new(Data::new(data)))} else {None}
            }
        }
    )+
    };
    ($module:ident, ($var:ident, $val:ident), $(($varo:ident, $valo:ident)),+) => {
        impl<'a> Write<$var> for $module::Controller<'a> {
            fn write(&mut self) -> Option<Arc<Data<$var>>> {
                if let $module::Y::$var(val) = &mut self.$val {
                let mut data = vec![0f64; val.len()];
                unsafe { ptr::copy_nonoverlapping(val.as_ptr(), data.as_mut_ptr(), data.len()) }
                Some(Arc::new(Data::new(data)))} else {None}
            }
        }
    $(
        impl<'a> Write<$varo> for $module::Controller<'a> {
            fn write(&mut self) -> Option<Arc<Data<$varo>>> {
                if let $module::Y::$varo(val) = &mut self.$valo {
                let mut data = vec![0f64; val.len()];
                unsafe { ptr::copy_nonoverlapping(val.as_ptr(), data.as_mut_ptr(), data.len()) }
                Some(Arc::new(Data::new(data)))} else {None}
            }
        }
    )+
    };
}
 */
