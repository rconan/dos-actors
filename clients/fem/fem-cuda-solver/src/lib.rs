#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(improper_ctypes)]

use std::ptr;

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

impl Default for state_space {
    fn default() -> Self {
        Self {
            n_mode: 0i32,
            n_input: 0i32,
            n_output: 0i32,
            d_i2m: ptr::null_mut(),
            d_m2o: ptr::null_mut(),
            d_u: ptr::null_mut(),
            d_v: ptr::null_mut(),
            d_x0: ptr::null_mut(),
            d_y: ptr::null_mut(),
            handle: ptr::null_mut(),
            stream: ptr::null_mut(),
            d_mss: ptr::null_mut(),
            d_dcg: ptr::null_mut(),
        }
    }
}

unsafe impl Send for mode_state_space {}
unsafe impl Sync for mode_state_space {}
unsafe impl Send for state_space {}
unsafe impl Sync for state_space {}
