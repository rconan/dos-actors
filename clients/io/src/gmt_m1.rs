//! GMT primary mirror

use interface::UID;

// M1 Rigid Body Motions
// #[derive(UID)]
// #[uid(port = 56_001)]
// pub enum M1RigidBodyMotions {}
pub type M1RigidBodyMotions = assembly::M1RigidBodyMotions;
// M1 Mode Shapes
// #[derive(UID)]
// #[uid(port = 56_002)]
// pub enum M1ModeShapes {}
pub type M1ModeShapes = assembly::M1ModeShapes;
/// M1 edge sensors
#[derive(UID, Debug)]
#[uid(port = 56_003)]
pub enum M1EdgeSensors {}

/// Mirror IO
pub mod assembly {
    use interface::UniqueIdentifier;
    use std::sync::Arc;

    use crate::Assembly;

    /// Rigid body motion (Tx,Ty,Tz,Rx,Ry,Rz)
    pub enum M1RigidBodyMotions {}
    impl Assembly for M1RigidBodyMotions {}
    impl UniqueIdentifier for M1RigidBodyMotions {
        type DataType = Vec<f64>;
        const PORT: u16 = 50_006;
    }

    /// Hardpoints displacements `[cell,mirror]`
    pub enum M1HardpointsMotion {}
    impl Assembly for M1HardpointsMotion {}
    impl UniqueIdentifier for M1HardpointsMotion {
        type DataType = Vec<Arc<Vec<f64>>>;
        const PORT: u16 = 50_007;
    }

    /// Hardpoints forces
    pub enum M1HardpointsForces {}
    impl Assembly for M1HardpointsForces {}
    impl UniqueIdentifier for M1HardpointsForces {
        type DataType = Vec<Arc<Vec<f64>>>;
        const PORT: u16 = 50_007;
    }

    /// Actuators command forces
    pub enum M1ActuatorCommandForces {}
    impl Assembly for M1ActuatorCommandForces {}
    impl UniqueIdentifier for M1ActuatorCommandForces {
        type DataType = Vec<f64>;
        const PORT: u16 = 50_008;
    }

    /// Actuators applied forces
    pub enum M1ActuatorAppliedForces {}
    impl Assembly for M1ActuatorAppliedForces {}
    impl UniqueIdentifier for M1ActuatorAppliedForces {
        type DataType = Vec<Arc<Vec<f64>>>;
        const PORT: u16 = 50_008;
    }
    /// M1 Mode Shapes
    pub enum M1ModeShapes {}
    impl Assembly for M1ModeShapes {}
    impl UniqueIdentifier for M1ModeShapes {
        type DataType = Vec<f64>;
        const PORT: u16 = 50_008;
    }
    /// M1 Mode Coefficients
    pub enum M1ModeCoefficients {}
    impl Assembly for M1ModeCoefficients {}
    impl UniqueIdentifier for M1ModeCoefficients {
        type DataType = Vec<f64>;
        const PORT: u16 = 50_009;
    }
}

/// Segment IO
pub mod segment {
    use interface::UniqueIdentifier;
    /// Force and moment at center of gravity
    pub enum BarycentricForce<const ID: u8> {}
    impl<const ID: u8> UniqueIdentifier for BarycentricForce<ID> {
        const PORT: u16 = 56_001 + 100 * ID as u16;
        type DataType = Vec<f64>;
    }
    /// Rigid body motion (Tx,Ty,Tz,Rx,Ry,Rz)
    pub enum RBM<const ID: u8> {}
    impl<const ID: u8> UniqueIdentifier for RBM<ID> {
        const PORT: u16 = 56_002 + 100 * ID as u16;
        type DataType = Vec<f64>;
    }
    /// Hardpoints displacements `[cell,mirror]`
    pub enum HardpointsMotion<const ID: u8> {}
    impl<const ID: u8> UniqueIdentifier for HardpointsMotion<ID> {
        const PORT: u16 = 56_003 + 100 * ID as u16;
        type DataType = Vec<f64>;
    }
    /// Hardpoints forces
    pub enum HardpointsForces<const ID: u8> {}
    impl<const ID: u8> UniqueIdentifier for HardpointsForces<ID> {
        const PORT: u16 = 56_004 + 100 * ID as u16;
        type DataType = Vec<f64>;
    }
    /// Actuators applied forces
    pub enum ActuatorAppliedForces<const ID: u8> {}
    impl<const ID: u8> UniqueIdentifier for ActuatorAppliedForces<ID> {
        const PORT: u16 = 56_005 + 100 * ID as u16;
        type DataType = Vec<f64>;
    }
    /// Actuators command forces
    pub enum ActuatorCommandForces<const ID: u8> {}
    impl<const ID: u8> UniqueIdentifier for ActuatorCommandForces<ID> {
        const PORT: u16 = 56_006 + 100 * ID as u16;
        type DataType = Vec<f64>;
    }
    /// SEGMENT RBM DOF selector (`[0,...,6]->[Tx,Ty,Tz,Rx,Ry,Rz]`)
    pub enum M1S<const ID: u8, const DOF: u8> {}
    impl<const ID: u8, const DOF: u8> UniqueIdentifier for M1S<ID, DOF> {
        const PORT: u16 = 56_001 + 10 * (1 + DOF) as u16 + 100 * ID as u16;
        type DataType = Vec<f64>;
    }
    #[deprecated = r#"Deprecated UID in favor of "ModesShape""#]
    /// BendingModes
    pub enum BendingModes<const ID: u8> {}
    #[allow(deprecated)]
    impl<const ID: u8> UniqueIdentifier for BendingModes<ID> {
        const PORT: u16 = 56_007 + 100 * ID as u16;
        type DataType = Vec<f64>;
    }
    /// Mode shapes
    pub enum ModeShapes<const ID: u8> {}
    impl<const ID: u8> UniqueIdentifier for ModeShapes<ID> {
        const PORT: u16 = 56_007 + 100 * ID as u16;
        type DataType = Vec<f64>;
    }
}
