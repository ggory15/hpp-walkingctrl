//
// Copyright (c) 2017 CNRS
//
// This file is part of tsid
// tsid is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// tsid is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// tsid If not, see
// <http://www.gnu.org/licenses/>.
//

#ifndef __HPP_WALKINGCTRL_INVDYN_UTIL_
#define __HPP_WALKINGCTRL_INVDYN_UTIL_

#include <hpp/walkingctrl/fwd.hh>
#include <hpp/walkingctrl/interface/Setting.hpp>

#include <hpp/walkingctrl/math/constraint-bound.hpp>
#include <hpp/walkingctrl/math/constraint-equality.hpp>
#include <hpp/walkingctrl/math/constraint-inequality.hpp>

#include <hpp/walkingctrl/invDynForm/contacts/contact-6d.hpp>

#include <hpp/walkingctrl/task/task.hh>
#include <hpp/walkingctrl/trajectory/trajectory.hh>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include "hpp/pinocchio/urdf/util.hh"
#include "hpp/pinocchio/humanoid-robot.hh"
#include "hpp/pinocchio/center-of-mass-computation.hh"
#include "hpp/pinocchio/bias.hh"
#include "hpp/pinocchio/joint.hh"
#include "hpp/pinocchio/configuration.hh"
#include <hpp/pinocchio/device.hh>

#include <vector>

namespace hpp{
 namespace walkingctrl{
    class InvDynForm{
      public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        InvDynForm(const DevicePtr_t & model);
        ~InvDynForm(){};
        void createInvDynUtil(InterfaceSetting& setting);
        void addTask(JointPostureTask& task, const double& gain);
        vector3_t getCOM(){return com_;};

      private:
        inline InterfaceSetting& getSetting() {return *interface_setting_;}
        inline const InterfaceSetting& getSetting() const { return *interface_setting_; }
        void createInvDynFormulation(const vector_t q, const vector_t v);
        void updateInequalityData(bool updateConstrainedDynamics=true, bool doNotUpdateForceLimits =false);
        void updateSupportPolygon();
        void computeConstraintTask(const double& t, matrix_t& A, vector_t& a);
        void updateConstrainedDynamics(); 
        void setNewSensorData(const double& t, const vector_t& q, const vector_t& v);
        void setPositions(const vector_t& q, bool updateConstraintReference = true);
        inline void setVelocities(const vector_t& v){ v_ = v;};
        void getDynamics();
        
        
       

      private:
        InterfaceSetting* interface_setting_;

        int nq_, nv_, na_, k_, m_in_; 
        double dt_, t_;
        bool use_com_gen_, enableCapturePoint_, enableTorqueLimit_, enableForceLimit_, enableJointLimit_, use_pos_preview_, use_vel_preview_, use_max_joint_acc_, use_vel_estimator_, use_rotor_initia_,
             support_polygon_computed_;
        vector_t q_, v_, qMin_, qMax_, tauMax_, dqMax_, ddqMax_, ddqStop_, contact_points_, contact_normal_, b_sp_, lb_, ub_, b_;
        matrix_t Md_, S_T_, Nc_T_, B_sp_, B_;

        vector_t dJc_v_, dx_c_, ddx_c_des_, c_;
        matrix_t Jc_, Jc_Minv_, Lambda_c_, Jc_T_pinv_, C_;

        vector2_t cp_;
        vector3_t com_;
        matrix_t J_com_, M_, h_, Minv_;
        vector_t dx_com_;
        
        Eigen::VectorXi index_acc_in_, index_cp_in_, index_task_constr;
        std::vector<contacts::Contact6d> rigidContactConstraints_;

        hpp::pinocchio::DevicePtr_t r_; 
        hpp::pinocchio::CenterOfMassComputationPtr_t com_pinocchio_;
        hpp::pinocchio::BiasComputationPtr_t bias_pinocchio_;    
        std::vector<JointPostureTask> taskstate_; 
        std::vector<double> w_gain_;   
    };       
  }
}

#endif // ifndef __invdyn_solvers_hqp_output_hpp__
