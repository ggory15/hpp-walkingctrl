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
#include "hpp/pinocchio/urdf/util.hh"
#include "hpp/pinocchio/humanoid-robot.hh"
#include "hpp/pinocchio/joint.hh"
#include "hpp/pinocchio/configuration.hh"
#include <hpp/pinocchio/device.hh>
#include <hpp/walkingctrl/interface/robot.hpp>

#include <vector>

namespace hpp{
 namespace walkingctrl{
    class InvDynForm : public RobotKinDyn {
      public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        InvDynForm(DevicePtr_t & model) : RobotKinDyn (model) {}
        ~InvDynForm(){}

        void createInvDynUtil(InterfaceSetting& setting);
        void addTask(JointPostureTask& task, const double& gain);
        inline InterfaceSetting& getSetting() {return *interface_setting_;}
        inline const InterfaceSetting& getSetting() const { return *interface_setting_; }

        std::vector<std::string> getContactSet() {return contact_names_;}
        bool existUnilateralContactConstraint(const std::string name);
        void addUnilateralContactConstraint(const SE3Task& constr, const vector3_t& contact_point, const vector3_t& contact_normal);

        void addContact(const std::string & name){contact_names_.push_back(name);}
      
      private:

        void createInvDynFormulation(const vector_t q, const vector_t v);

        void updateInequalityData(bool updateConstrainedDynamics=true, bool doNotUpdateForceLimits =false);
        void updateSupportPolygon();
        void computeConstraintTask(const double& t, matrix_t& A, vector_t& a);
        void updateConstrainedDynamics(); 
        void setNewSensorData(const double& t, const vector_t& q, const vector_t& v);

        void setPositions(const vector_t& q, bool updateConstraintReference = true);
        inline void setVelocities(const vector_t& v){ v_res_ = v;};

       

      private:
        InterfaceSetting* interface_setting_;

        int k_, m_in_; 
        double dt_, t_, fmin_;
        bool use_com_gen_, enableCapturePoint_, enableTorqueLimit_, enableForceLimit_, enableJointLimit_, use_pos_preview_, use_vel_preview_, use_max_joint_acc_, use_vel_estimator_, use_rotor_initia_,
             support_polygon_computed_;
        vector_t q0_, v0_, q_res_, v_res_, qMin_, qMax_, tauMax_, dqMax_, ddqMax_, ddqStop_,  b_sp_, lb_, ub_, b_;
        matrix_t Md_, S_T_, Nc_T_, B_sp_, B_;

        vector_t dJc_v_, dx_c_, ddx_c_des_, c_;
        matrix_t Jc_, Jc_Minv_, Lambda_c_, Jc_T_pinv_, C_;

        vector2_t cp_, mu_;
        vector3_t com_;
        matrix_t J_com_, M_, h_, Minv_;
        vector_t dx_com_;

        matrix_t contact_points_, contact_normals_;
        
        Eigen::VectorXi index_acc_in_, index_cp_in_, index_task_constr;
        std::vector<SE3Task> rigidContactConstraints_;
        std::vector<vector3_t> rigidContactConstraints_p_, rigidContactConstraints_N_;
        std::vector<double> rigidContactConstraints_fMin_;
        std::vector<vector2_t> rigidContactConstraints_mu_;

        std::vector<JointPostureTask> taskstate_; 
        std::vector<double> w_gain_;   
        std::vector<std::string> contact_names_;
    };       
  }
}

#endif // ifndef __invdyn_solvers_hqp_output_hpp__
