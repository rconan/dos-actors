use gmt_dos_actors::{model::Unknown, prelude::Model, Actor};
use gmt_dos_clients::interface::{Update, Write};
use gmt_dos_clients_fem::{DiscreteModalSolver, ExponentialMatrix};
use gmt_dos_clients_io::gmt_m1::segment::{ActuatorCommandForces, RBM};

use crate::{segment_builder::SegmentBuilder, Calibration, Mirror, Segment};

pub struct Builder<
    'a,
    const ACTUATOR_RATE: usize,
    CH1,
    CA1,
    const NH1: usize,
    const NA1: usize,
    CH2,
    CA2,
    const NH2: usize,
    const NA2: usize,
    CH3,
    CA3,
    const NH3: usize,
    const NA3: usize,
    CH4,
    CA4,
    const NH4: usize,
    const NA4: usize,
    CH5,
    CA5,
    const NH5: usize,
    const NA5: usize,
    CH6,
    CA6,
    const NH6: usize,
    const NA6: usize,
    CH7,
    CA7,
    const NH7: usize,
    const NA7: usize,
> where
    CH1: Update + Write<RBM<1>> + Send + 'static,
    CA1: Update + Write<ActuatorCommandForces<1>> + Send + 'static,
    CH2: Update + Write<RBM<2>> + Send + 'static,
    CA2: Update + Write<ActuatorCommandForces<2>> + Send + 'static,
    CH3: Update + Write<RBM<3>> + Send + 'static,
    CA3: Update + Write<ActuatorCommandForces<3>> + Send + 'static,
    CH4: Update + Write<RBM<4>> + Send + 'static,
    CA4: Update + Write<ActuatorCommandForces<4>> + Send + 'static,
    CH5: Update + Write<RBM<5>> + Send + 'static,
    CA5: Update + Write<ActuatorCommandForces<5>> + Send + 'static,
    CH6: Update + Write<RBM<6>> + Send + 'static,
    CA6: Update + Write<ActuatorCommandForces<6>> + Send + 'static,
    CH7: Update + Write<RBM<7>> + Send + 'static,
    CA7: Update + Write<ActuatorCommandForces<7>> + Send + 'static,
{
    calibration: Calibration,
    s1: Option<SegmentBuilder<'a, 1, ACTUATOR_RATE, CH1, CA1, NH1, NA1>>,
    s2: Option<SegmentBuilder<'a, 2, ACTUATOR_RATE, CH2, CA2, NH2, NA2>>,
    s3: Option<SegmentBuilder<'a, 3, ACTUATOR_RATE, CH3, CA3, NH3, NA3>>,
    s4: Option<SegmentBuilder<'a, 4, ACTUATOR_RATE, CH4, CA4, NH4, NA4>>,
    s5: Option<SegmentBuilder<'a, 5, ACTUATOR_RATE, CH5, CA5, NH5, NA5>>,
    s6: Option<SegmentBuilder<'a, 6, ACTUATOR_RATE, CH6, CA6, NH6, NA6>>,
    s7: Option<SegmentBuilder<'a, 7, ACTUATOR_RATE, CH7, CA7, NH7, NA7>>,
    valid_sid: Vec<u8>,
}

impl<'a, const ACTUATOR_RATE: usize> Mirror<ACTUATOR_RATE> {
    pub fn builder<
        CH1,
        CA1,
        const NH1: usize,
        const NA1: usize,
        CH2,
        CA2,
        const NH2: usize,
        const NA2: usize,
        CH3,
        CA3,
        const NH3: usize,
        const NA3: usize,
        CH4,
        CA4,
        const NH4: usize,
        const NA4: usize,
        CH5,
        CA5,
        const NH5: usize,
        const NA5: usize,
        CH6,
        CA6,
        const NH6: usize,
        const NA6: usize,
        CH7,
        CA7,
        const NH7: usize,
        const NA7: usize,
    >(
        calibration: Calibration,
    ) -> Builder<
        'a,
        ACTUATOR_RATE,
        CH1,
        CA1,
        NH1,
        NA1,
        CH2,
        CA2,
        NH2,
        NA2,
        CH3,
        CA3,
        NH3,
        NA3,
        CH4,
        CA4,
        NH4,
        NA4,
        CH5,
        CA5,
        NH5,
        NA5,
        CH6,
        CA6,
        NH6,
        NA6,
        CH7,
        CA7,
        NH7,
        NA7,
    >
    where
        CH1: Update + Write<RBM<1>> + Send + 'static,
        CA1: Update + Write<ActuatorCommandForces<1>> + Send + 'static,
        CH2: Update + Write<RBM<2>> + Send + 'static,
        CA2: Update + Write<ActuatorCommandForces<2>> + Send + 'static,
        CH3: Update + Write<RBM<3>> + Send + 'static,
        CA3: Update + Write<ActuatorCommandForces<3>> + Send + 'static,
        CH4: Update + Write<RBM<4>> + Send + 'static,
        CA4: Update + Write<ActuatorCommandForces<4>> + Send + 'static,
        CH5: Update + Write<RBM<5>> + Send + 'static,
        CA5: Update + Write<ActuatorCommandForces<5>> + Send + 'static,
        CH6: Update + Write<RBM<6>> + Send + 'static,
        CA6: Update + Write<ActuatorCommandForces<6>> + Send + 'static,
        CH7: Update + Write<RBM<7>> + Send + 'static,
        CA7: Update + Write<ActuatorCommandForces<7>> + Send + 'static,
    {
        Builder {
            calibration,
            s1: None,
            s2: None,
            s3: None,
            s4: None,
            s5: None,
            s6: None,
            s7: None,
            valid_sid: vec![],
        }
    }
}

impl<
        'a,
        const ACTUATOR_RATE: usize,
        CH1,
        CA1,
        const NH1: usize,
        const NA1: usize,
        CH2,
        CA2,
        const NH2: usize,
        const NA2: usize,
        CH3,
        CA3,
        const NH3: usize,
        const NA3: usize,
        CH4,
        CA4,
        const NH4: usize,
        const NA4: usize,
        CH5,
        CA5,
        const NH5: usize,
        const NA5: usize,
        CH6,
        CA6,
        const NH6: usize,
        const NA6: usize,
        CH7,
        CA7,
        const NH7: usize,
        const NA7: usize,
    >
    Builder<
        'a,
        ACTUATOR_RATE,
        CH1,
        CA1,
        NH1,
        NA1,
        CH2,
        CA2,
        NH2,
        NA2,
        CH3,
        CA3,
        NH3,
        NA3,
        CH4,
        CA4,
        NH4,
        NA4,
        CH5,
        CA5,
        NH5,
        NA5,
        CH6,
        CA6,
        NH6,
        NA6,
        CH7,
        CA7,
        NH7,
        NA7,
    >
where
    CH1: Update + Write<RBM<1>> + Send + 'static,
    CA1: Update + Write<ActuatorCommandForces<1>> + Send + 'static,
    CH2: Update + Write<RBM<2>> + Send + 'static,
    CA2: Update + Write<ActuatorCommandForces<2>> + Send + 'static,
    CH3: Update + Write<RBM<3>> + Send + 'static,
    CA3: Update + Write<ActuatorCommandForces<3>> + Send + 'static,
    CH4: Update + Write<RBM<4>> + Send + 'static,
    CA4: Update + Write<ActuatorCommandForces<4>> + Send + 'static,
    CH5: Update + Write<RBM<5>> + Send + 'static,
    CA5: Update + Write<ActuatorCommandForces<5>> + Send + 'static,
    CH6: Update + Write<RBM<6>> + Send + 'static,
    CA6: Update + Write<ActuatorCommandForces<6>> + Send + 'static,
    CH7: Update + Write<RBM<7>> + Send + 'static,
    CA7: Update + Write<ActuatorCommandForces<7>> + Send + 'static,
{
    /* pub fn segment<
        CH: Update + Send + 'static,
        CA: Update + Send + 'static,
        const NH: usize,
        const NA: usize,
    >(
        mut self,
        calibration: Calibration,
        sid: u8,
        rbm_setpoint_actor: &'a mut Actor<CH, NH, 1>,
        actuators_setpoint_actor: &'a mut Actor<CA, NA, ACTUATOR_RATE>,
    ) {
        match sid {
            i if i == 1 => {
                self.s1 = Some(Segment::<1, ACTUATOR_RATE>::builder(
                    calibration.clone(),
                    rbm_setpoint_actor,
                    actuators_setpoint_actor,
                ))
            }
            _ => unimplemented!(),
        }
    } */
    pub fn build(
        self,
        plant: &mut Actor<DiscreteModalSolver<ExponentialMatrix>>,
    ) -> anyhow::Result<Model<Unknown>> {
        todo!()
    }
}
