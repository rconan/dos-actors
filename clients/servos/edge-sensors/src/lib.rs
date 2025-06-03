/*!
# EDGE SENSORS API

## M2 VOICE COILS TO RBMS

[VoiceCoilToRbm] transforms voice coils displacements to rigid body motions.

The matrix transform is derive from 2 gain matrices of the FEM:
 * the gain from voice coils forces to rigid body motions $K_1$:
```shell
cargo run -r -p gmt_dos-clients_fem --bin static_gain --features="serde clap" -- \
    -i MC_M2_S1_VC_delta_F -i MC_M2_S2_VC_delta_F -i MC_M2_S3_VC_delta_F \
    -i MC_M2_S4_VC_delta_F -i MC_M2_S5_VC_delta_F -i MC_M2_S6_VC_delta_F \
    -i MC_M2_S7_VC_delta_F \
    -o MC_M2_lcl_6D \
    -f vcf_2_rbm.pkl
```
 * the gain from voice coils forces to voice coils displacements $K_2$:
```shell
cargo run -r -p gmt_dos-clients_fem --bin static_gain --features="serde clap" -- \
    -i MC_M2_S1_VC_delta_F -i MC_M2_S2_VC_delta_F -i MC_M2_S3_VC_delta_F \
    -i MC_M2_S4_VC_delta_F -i MC_M2_S5_VC_delta_F -i MC_M2_S6_VC_delta_F \
    -i MC_M2_S7_VC_delta_F \
    -o MC_M2_S1_VC_delta_D -o MC_M2_S2_VC_delta_D -o MC_M2_S3_VC_delta_D \
    -o MC_M2_S4_VC_delta_D -o MC_M2_S5_VC_delta_D -o MC_M2_S6_VC_delta_D \
    -o MC_M2_S7_VC_delta_D \
    -f vcf.pkl
```

The matrix transform $T$ is the solution of $$TK_2 = K_1$$
```python
import numpy as np
from scipy.io import savemat

data = np.load("vcf_2_rbm.pkl",allow_pickle=True)
k1 = np.asarray(data[0],order="F").reshape(data[2],data[1]).T
data = np.load("vcf.pkl",allow_pickle=True)
k2 = np.asarray(data[0],order="F").reshape(data[2],data[1]).T
m2_vc_r = {f"m2_s{i+1}_vc_r": \
    np.linalg.lstsq(k2[i*675:(i+1)*675,i*675:(i+1)*675].T,k1[i*6:(i+1)*6,i*675:(i+1)*675].T,\
    rcond=None)[0].T for i in range(7)}
savemat("m2_vc_r.mat",m2_vc_r)
```

## M2 RBMS TO FACESHEET

[RbmToShell] transforms rigid body motions to shell displacements.

The matrix transform is derive from 2 gain matrices of the FEM:
 * the gain from positioner forces to shell axis displacements $K_1$:
```shell
cargo run -r -p gmt_dos-clients_fem --bin static_gain --features="serde clap" -- \
    -i MC_M2_SmHex_F \
    -o M2_segment_1_axial_d -o M2_segment_2_axial_d -o M2_segment_3_axial_d \
    -o M2_segment_4_axial_d -o M2_segment_5_axial_d -o M2_segment_6_axial_d \
    -o M2_segment_7_axial_d \
    -f rbm_2_facesheet.pkl
```
 * the gain from positioner forces to the rigid body motions of the reference body $K_2$:
```shell
cargo run -r -p gmt_dos-clients_fem --bin static_gain --features="serde clap" -- \
    -i MC_M2_SmHex_F -o MC_M2_RB_6D -f hex_2_rbm.pkl
```

The matrix transform $T$ is given by $$T = K_2 K_1^{-1}$$
```python
import numpy as np
from scipy.io import savemat

data = np.load("rbm_2_facesheet.pkl",allow_pickle=True)
k1 = np.asarray(data[0],order="F").reshape(data[2],data[1]).T
data = np.load("hex_2_rbm.pkl",allow_pickle=True)
k2 = np.asarray(data[0],order="F").reshape(data[2],data[1]).T
k1p = k1[:,::2] - k1[:,1::2]
k2p = k2[:,::2] - k2[:,1::2]
rbm_2_faceheet = {f"m2_s{i+1}_rbm_2_shell": \
    k1p[i*675:(i+1)*675,i*6:(i+1)*6] @ np.linalg.inv(k2p[i*6:(i+1)*6,i*6:(i+1)*6])\
     for i in range(7)}
savemat("rbm_2_faceheet.mat",rbm_2_faceheet)
```

## M2 POSITIONER DISPLACEMENT TO M2 RBM

```shell
cargo run -r -p gmt_dos-clients_fem --bin static_gain --features="serde clap" -- \
    -i MC_M2_SmHex_F \
    -o MC_M2_SmHex_D \
    -f hex_f2d.pkl
cargo run -r -p gmt_dos-clients_fem --bin static_gain --features="serde clap" -- \
    -i MC_M2_SmHex_F \
    -o MC_M2_RB_6D \
    -f hex_f2r.pkl
```
```python
import numpy as np
from scipy.io import savemat

data = np.load("hex_f2d.pkl",allow_pickle=True)
k1 = np.asarray(data[0],order="F").reshape(data[2],data[1]).T
data = np.load("hex_f2r.pkl",allow_pickle=True)
k2 = np.asarray(data[0],order="F").reshape(data[2],data[1]).T
k1p = k1[::2,::2] - k1[1::2,1::2]
k2p = k2[:,::2] - k2[:,1::2]
d2r = k2p @ np.linalg.inv(k1p)
savemat("m2_hex_d2r.mat",{"d2r":d2r})
```

## ASMS OFF-LOADING

The ASMS off-loading algorithm depends on 2 transformations:
 1. from rigid body motions to edge sensors where $T$ is the solution of $TK_2 = K_1$, with:
    1. $K_2$: from positioner forces to edge sensors displacements
    2. $K_1$: from positioner forces to reference bodies RBMS
 2. from the rigid body motions of M2S7 to edge sensors given by the solution to $$TK_1^7 = K_1^7$$

 ```shell
cargo run -r -p gmt_dos-clients_fem --features serde,clap --bin static_gain -- \
    -i MC_M2_SmHex_F -o M2_edge_sensors -f asms_off-loading_k2.pkl
cargo run -r -p gmt_dos-clients_fem --features serde,clap --bin static_gain -- \
    -i MC_M2_SmHex_F -o MC_M2_RB_6D -f asms_off-loading_k1.pkl
```

```python
import numpy as np
from scipy.io import savemat

data = np.load("asms_off-loading_k1.pkl",allow_pickle=True)
k1 = np.asarray(data[0],order="F").reshape(data[2],data[1]).T
k1p = k1[:,::2] - k1[:,1::2]
data = np.load("asms_off-loading_k2.pkl",allow_pickle=True)
k2 = np.asarray(data[0],order="F").reshape(data[2],data[1]).T
k2p = k2[:,::2] - k2[:,1::2]
m2_r_es = np.linalg.lstsq(k2p[:,:36].T,k1p[:36,:36].T,rcond=None)[0].T

savemat("m2_r_es.mat",{"m2_r_es":m2_r_es})

m2_r7_es = np.linalg.lstsq(k1p[-6:,-6:].T,k2p[:,-6:].T,rcond=None)[0].T
savemat("m2_r7_es.mat",{"m2_r7_es":m2_r7_es})
```

## M1 EDGE SENSORS TO M1 RBM

 ```shell
cargo run -r -p gmt_dos-clients_fem --features serde,clap --bin static_gain -- \
    -i OSS_Harpoint_delta_F -o OSS_M1_lcl -f m1_r_es_k1.pkl
cargo run -r -p gmt_dos-clients_fem --features serde,clap --bin static_gain -- \
    -i OSS_Harpoint_delta_F -o OSS_M1_edge_sensors -f m1_r_es_k2.pkl
```
```python
import numpy as np
from scipy.io import loadmat, savemat

data = np.load("m1_r_es_k1.pkl",allow_pickle=True)
k1 = np.asarray(data[0],order="F").reshape(data[2],data[1]).T
data = np.load("m1_r_es_k2.pkl",allow_pickle=True)
k2 = np.asarray(data[0],order="F").reshape(data[2],data[1]).T
data = loadmat("M1_edge_sensor_conversion.mat")
A1 = data['A1']
k2p = A1@k2
m1_r_es = np.linalg.lstsq(k2p[:,:36].T,k1[:36,:36].T,rcond=None)[0].T

savemat("m12_r_es.mat",{"m1_r_es":m1_r_es})
```

*/

pub mod asms_offload;
pub mod edge_sensors_feed_forward;
mod hex_to_rbm;
pub mod m1_edgesensors_to_rbm;
mod m2_edgesensors_to_rbm;
mod rbm_to_shell;
mod scopes;
mod transform;
mod voice_coil_to_rbm;

pub const N_ACTUATOR: usize = 675;
pub const N_SCOPE: usize = 8;

pub use asms_offload::AsmsToHexOffload;
pub use edge_sensors_feed_forward::EdgeSensorsFeedForward;
use gmt_dos_clients_fem::{DiscreteModalSolver, solvers::ExponentialMatrix};
use gmt_dos_clients_io::gmt_m1::M1EdgeSensors;
pub use hex_to_rbm::HexToRbm;
pub use m1_edgesensors_to_rbm::M1EdgeSensorsToRbm;
pub use m2_edgesensors_to_rbm::M2EdgeSensorsToRbm;
pub use rbm_to_shell::RbmToShell;
pub use scopes::segment_piston;
pub use transform::{IO, Transform};
pub use voice_coil_to_rbm::VoiceCoilToRbm;

pub enum M1EdgeSensorsAsRbms {}
impl ::interface::UniqueIdentifier for M1EdgeSensorsAsRbms {
    const PORT: u16 = <M1EdgeSensors as ::interface::UniqueIdentifier>::PORT;
    type DataType = <M1EdgeSensors as ::interface::UniqueIdentifier>::DataType;
}
impl ::interface::Read<M1EdgeSensorsAsRbms> for RbmToShell {
    #[inline]
    fn read(&mut self, data: ::interface::Data<M1EdgeSensorsAsRbms>) {
        <Self as ::interface::Read<M1EdgeSensors>>::read(self, data.transmute());
    }
}
impl ::interface::Write<M1EdgeSensorsAsRbms> for DiscreteModalSolver<ExponentialMatrix> {
    #[inline]
    fn write(&mut self) -> Option<::interface::Data<M1EdgeSensorsAsRbms>> {
        <Self as ::interface::Write<M1EdgeSensors>>::write(self).map(|data| data.transmute())
    }
}
