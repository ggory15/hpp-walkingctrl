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

#ifndef __HPP_WALKINGCTRL_SIMULATOR__
#define __HPP_WALKINGCTRL_SIMULATOR__

#include <hpp/walkingctrl/fwd.hh>
#include <hpp/walkingctrl/interface/Setting.hpp>
#include <hpp/walkingctrl/invDynForm/contacts/contact-6d.hpp>

#include <hpp/walkingctrl/solver/staggered_projections.hpp>

#include <hpp/walkingctrl/math/constraint-bound.hpp>
#include <hpp/walkingctrl/math/constraint-equality.hpp>
#include <hpp/walkingctrl/math/constraint-inequality.hpp>

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
    class Simulator{
      public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Simulator(const DevicePtr_t & model, bool detectContactPoint = true);
        ~Simulator(){};
        void createSimulator(InterfaceSetting& setting);
        void reset(const double & time, const vector_t& q, const vector_t & v, const double & dt);
        vector_t getjointAngle() {return q_;}
        vector_t getjointVelocity() {return v_;}

      private:         
        void getDynamics();
        void updateInequalityData();

      private:
        int nq_, nv_, na_, verb_, k_;
        double time_step, fmin_, dt_;
        bool DETECT_CONTACT_POINTS, INITIALIZE_TORQUE_FILTER;
        vector2_t mu_;
        vector3_t com_;
        vector_t q_, v_, vold_, dv_, qMin_, qMax_, dqMax_, tauMax_, c_;
        matrix_t S_T_, M_, J_com_, Md_, C_;

        hpp::pinocchio::DevicePtr_t r_;     

      private:
        InterfaceSetting* interface_setting_;
        solvers::StaggeredProjections* LCP_;
        hpp::pinocchio::CenterOfMassComputationPtr_t com_pinocchio_;
        hpp::pinocchio::BiasComputationPtr_t bias_pinocchio_;    
        std::vector<contacts::ContactInformation> rigidContactConstraints_;
    };       
  }
}

#endif // ifndef __invdyn_solvers_hqp_output_hpp__
