# Scope for UID IO in gmt_dos-clients_io

## Usage

Any `IO` type defined in `gmt_dos-clients_io` can be scoped with 
```shell
SIGNAL=<IO> cargo r -r 
```

Units given by the type defined in the module `unitd` of `gmt_dos-actors-clients_interface` are applied to the signal with
```shell
SIGNAL=<IO> U=<units> cargo r -r 
```

**Note**: if the signal is broadcasted from a remote server, the IP of the server need to be assigned to the environment variable `SCOPE_SERVER_IP`

## Example

Building a scope for `SegmentPiston` in nanometers" 
```shell
SIGNAL=SegmentPiston U=NM cargo r -r
```
