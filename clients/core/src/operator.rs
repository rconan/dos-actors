use std::{
    fmt::Debug,
    marker::PhantomData,
    ops::{Add, Sub},
    sync::Arc,
};

use interface::{
    Data, Left, Read, Right, UniqueIdentifier, Update, Write,
    optics::state::{MirrorState, OpticalState},
};

#[derive(Default, Debug, Clone)]
pub enum OperatorKind {
    #[default]
    Add,
    Sub,
}
impl<S: AsRef<str>> From<S> for OperatorKind {
    fn from(value: S) -> Self {
        match value.as_ref() {
            "+" => Self::Add,
            "-" => Self::Sub,
            _ => unimplemented!(r#"operators are either "+" or "-""#),
        }
    }
}

#[derive(Debug, Clone)]
pub enum Plus {}
#[derive(Debug, Clone)]
pub enum Minus {}

pub trait PlusOrMinus {}
impl PlusOrMinus for Plus {}
impl PlusOrMinus for Minus {}

#[derive(Debug, Clone)]
pub struct Operator<T, K = Plus>
where
    K: PlusOrMinus,
{
    left: Arc<T>,
    right: Arc<T>,
    output: Arc<T>,
    kind: PhantomData<K>,
}
impl<T: Default, K: PlusOrMinus> Default for Operator<T, K> {
    fn default() -> Self {
        Self {
            left: Default::default(),
            right: Default::default(),
            output: Default::default(),
            kind: PhantomData,
        }
    }
}

impl<T> Operator<T>
where
    T: Default,
{
    pub fn new() -> Self {
        Default::default()
    }
}
impl<T> Operator<T, Plus>
where
    T: Default,
{
    pub fn plus() -> Self {
        Default::default()
    }
}
impl<T> Operator<T, Minus>
where
    T: Default,
{
    pub fn minus() -> Self {
        Default::default()
    }
}

impl<T> Update for Operator<Vec<T>, Plus>
where
    T: Copy + Add<Output = T> + Send + Sync + Debug + Default,
{
    fn update(&mut self) {
        match (self.left.is_empty(), self.right.is_empty()) {
            (true, true) => {
                self.output = Default::default();
                return;
            }
            (true, false) => {
                self.left = Arc::new(vec![T::default(); self.right.len()]);
            }
            (false, true) => {
                self.right = Arc::new(vec![T::default(); self.left.len()]);
            }
            (false, false) => (),
        };
        let mut buffer = self.left.as_slice().to_vec();
        if self.left.len() < self.right.len() {
            buffer.extend(vec![T::default(); self.right.len() - self.left.len()]);
        }
        assert_eq!(
            buffer.len(),
            self.right.len(),
            "cannot add or substract vectors of different sizes"
        );
        self.output = Arc::new(
            buffer
                .iter()
                .zip(&*self.right)
                .map(|(left, right)| *left + *right)
                .collect::<Vec<T>>(),
        );
    }
}
impl<T> Update for Operator<Vec<T>, Minus>
where
    T: Copy + Sub<Output = T> + Send + Sync + Debug + Default,
{
    fn update(&mut self) {
        match (self.left.is_empty(), self.right.is_empty()) {
            (true, true) => {
                self.output = Default::default();
                return;
            }
            (true, false) => {
                self.left = Arc::new(vec![T::default(); self.right.len()]);
            }
            (false, true) => {
                self.right = Arc::new(vec![T::default(); self.left.len()]);
            }
            (false, false) => (),
        };
        let mut buffer = self.left.as_slice().to_vec();
        if self.left.len() < self.right.len() {
            buffer.extend(vec![T::default(); self.right.len() - self.left.len()]);
        }
        assert_eq!(
            buffer.len(),
            self.right.len(),
            "cannot add or substract vectors of different sizes"
        );
        self.output = Arc::new(
            buffer
                .iter()
                .zip(&*self.right)
                .map(|(left, right)| *left - *right)
                .collect::<Vec<T>>(),
        );
    }
}

impl Update for Operator<MirrorState, Plus> {
    fn update(&mut self) {
        self.output = Arc::new(&*self.left + &*self.right)
    }
}
impl Update for Operator<OpticalState, Plus> {
    fn update(&mut self) {
        self.output = Arc::new(&*self.left + &*self.right)
    }
}
impl Update for Operator<MirrorState, Minus> {
    fn update(&mut self) {
        self.output = Arc::new(&*self.left - &*self.right)
    }
}

impl<T, U> Read<Left<U>> for Operator<T>
where
    U: UniqueIdentifier<DataType = T>,
    Operator<T>: Update,
{
    fn read(&mut self, data: Data<Left<U>>) {
        self.left = data.as_arc()
    }
}

impl<T, U> Read<Right<U>> for Operator<T>
where
    U: UniqueIdentifier<DataType = T>,
    Operator<T>: Update,
{
    fn read(&mut self, data: Data<Right<U>>) {
        self.right = data.as_arc()
    }
}

impl<T, U> Write<U> for Operator<T>
where
    U: UniqueIdentifier<DataType = T>,
    Operator<T>: Update,
{
    fn write(&mut self) -> Option<Data<U>> {
        Some(self.output.clone().into())
    }
}

// // Read left or right any data which UID implements OperatorLeftRight trait
// impl<T, U> Read<U> for Operator<T>
// where
//     T: Copy + Add<Output = T> + Sub<Output = T> + Send + Sync + Debug + Default,
//     U: UniqueIdentifier<DataType = T> + OperatorLeftRight,
//     Operator<T>: Update,
// {
//     fn read(&mut self, data: Data<U>) {
//         if <U as OperatorLeftRight>::LEFT {
//             self.left = data.as_arc()
//         } else {
//             self.right = data.as_arc()
//         }
//     }
// }
