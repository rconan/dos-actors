# `gmt_dos-clients_domeseeing`

[![Crates.io](https://img.shields.io/crates/v/gmt_dos-clients_domeseeing.svg)](https://crates.io/crates/gmt_dos-clients_domeseeing)
[![Documentation](https://docs.rs/gmt_dos-clients_domeseeing/badge.svg)](https://docs.rs/gmt_dos-clients_domeseeing/)

A client for importing dome seeing wavefront error maps.

Building the crate depending on the feature:
 - default(=bincode): `cargo b`
 - npyz             : `cargo b --no-default-features --features npyz`
 - object_store     : `cargo b --no-default-features --features object_store`
