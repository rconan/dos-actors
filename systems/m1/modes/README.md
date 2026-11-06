# M1 bending modes computation

The bending modes are derived such that they are in the null space of either
M1 rigid body motions, both M1 rigid body motions and hardpoints motion,
both M1 rigid body motions and the piston, tip and tilt of the surface
(`--null-space=rbm-ptt`) or both M1 rigid body motions and the tip and tilt of
the surface (`--null-space=rbm-tt`).

Without rigid body motions, focus comes with a piston of the surface: `rbm-ptt`
removes focus from the modes, `rbm-tt` keeps it.

## Usage

```shell
cargo r -r -- --help
``` 
