//
// Copyright (c) 2015 CNRS
// Authors: Florent Lamiraux
//
// This file is part of hpp-walkgen
// hpp-walkgen is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-walkgen is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-walkgen  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_WALKINGCTRL_SOLVER_FWD_HH
#define HPP_WALKINGCTRL_SOLVER_FWD_HH

# include <hpp/walkingctrl/config.hh>
# include <hpp/walkingctrl/fwd.hh>
# include <hpp/walkingctrl/math/fwd.hpp>
# include <Eigen/Core>
#include <pinocchio/container/aligned-vector.hpp>

namespace hpp {
  namespace walkingctrl {
    namespace solvers{
        enum HPP_WALKINGCTRL_DLLAPI SolverHQP{
                SOLVER_HQP_QPOSASE = 0
        };
        enum HPP_WALKINGCTRL_DLLAPI HQPStatus        {
          HQP_STATUS_UNKNOWN=-1,
          HQP_STATUS_OPTIMAL=0,
          HQP_STATUS_INFEASIBLE=1,
          HQP_STATUS_UNBOUNDED=2,
          HQP_STATUS_MAX_ITER_REACHED=3,
          HQP_STATUS_ERROR=4
        };

        class HQPOutput;

        class HPP_WALKINGCTRL_DLLAPI SolverHQPBase;

        template<typename T1, typename T2>
        class aligned_pair
        {
        public:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW

          aligned_pair(const T1 & t1, const T2 & t2) : first(t1), second(t2) {}

          T1 first;
          T2 second;

        };

        template<typename T1, typename T2>
        inline aligned_pair<T1,T2> make_pair(const T1 & t1, const T2 & t2)
        { return aligned_pair<T1,T2>(t1,t2); }


        typedef se3::container::aligned_vector< aligned_pair<double, math::ConstraintBase*> > ConstraintLevel;
        typedef se3::container::aligned_vector< aligned_pair<double, const math::ConstraintBase*> > ConstConstraintLevel;
        typedef se3::container::aligned_vector<ConstraintLevel> HQPData;
        typedef se3::container::aligned_vector<ConstConstraintLevel> ConstHQPData;
    } // namespace solvers
  } // namespace walingctrl
} // namespace hpp

#endif // HPP_WALKINGCTRL_FWD_HH
