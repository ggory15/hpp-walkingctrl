#pragma once

namespace hpp{
    namespace walkingctrl {
        enum InterfaceIntParam {
            InterfaceIntParam_MAX_TEST_DURATION,
            InterfaceIntParam_maxIter,
            InterfaceIntParam_verb,
        };

        enum InterfaceBoolParam{
            InterfaceBoolParam_ENABLE_CAPTURE_POINT_LIMITS,
            InterfaceBoolParam_ENABLE_TORQUE_LIMITS,
            InterfaceBoolParam_ENABLE_FORCE_LIMITS,
            InterfaceBoolParam_ENABLE_JOINT_LIMITS,
            InterfaceBoolParam_IMPOSE_POSITION_BOUNDS,
            InterfaceBoolParam_IMPOSE_VELOCITY_BOUNDS,
            InterfaceBoolParam_IMPOSE_VIABILITY_BOUNDS,
            InterfaceBoolParam_IMPOSE_ACCELERATION_BOUNDS,
            InterfaceBoolParam_USE_JOINT_VELOCITY_ESTIMATOR,
            InterfaceBoolParam_ACCOUNT_FOR_ROTOR_INERTIAS,
            InterfaceBoolParam_USE_LCP_SOLVER,

            InterfaceBoolParam_SAVE_RESULT,
        };

        enum InterfaceDoubleParam{
            InterfaceDoubleParam_dt,
            InterfaceDoubleParam_JOINT_POS_PREVIEW,
            InterfaceDoubleParam_JOINT_VEL_PREVIEW,
            InterfaceDoubleParam_MAX_JOINT_ACC,
            InterfaceDoubleParam_MAX_MIN_JOINT_ACC,

            InterfaceDoubleParam_kp_posture,
            InterfaceDoubleParam_kd_posture,
            InterfaceDoubleParam_kp_constr,
            InterfaceDoubleParam_kd_constr,
            InterfaceDoubleParam_kp_com,
            InterfaceDoubleParam_kd_com,

            InterfaceDoubleParam_w_com,
            InterfaceDoubleParam_w_posture,
            InterfaceDoubleParam_maxTime,
            InterfaceDoubleParam_fmin,

            InterfaceDoubleParam_MAX_COM_VELOCITY,
            InterfaceDoubleParam_MAX_CONSTRAINT_ERROR,
        };

        enum InterfaceStringParam{
            InterfaceStringParam_ConfigFile,
            InterfaceStringParam_SaveDynamicsFile,
        };

        enum InterfaceIntVectorParam {
            InterfaceIntVectorParam_constraint_mask,
        };

        enum InterfaceVectorParam{
            InterfaceVectorParam_q0,
            InterfaceVectorParam_DEFAULT_CONTACT_POINTS,
            InterfaceVectorParam_DEFAULT_CONTACT_NORMALS,
            InterfaceVectorParam_mu,
        };

    }
}
