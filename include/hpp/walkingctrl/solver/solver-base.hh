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

#ifndef HPP_WALKINGCTRL_SOLVER_BASE_HH
#define HPP_WALKINGCTRL_SOLVER_BASE_HH

#include "hpp/walkingctrl/solver/fwd.hh"
#include "hpp/walkingctrl/solver/solver-HQP-output.hpp"

namespace hpp {
  namespace walkingctrl {
    namespace solvers{
        class HPP_WALKINGCTRL_DLLAPI SolverHQPBase
        {
        public:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW

          static std::string const HQP_status_string [5];
          SolverHQPBase(const std::string & name);
          virtual const std::string & name() { return m_name; }
          virtual void resize(unsigned int n, unsigned int neq, unsigned int nin) = 0;
          virtual const HQPOutput & solve(const HQPData & problemData) = 0;
          virtual double getObjectiveValue() = 0;
          virtual bool getUseWarmStart(){ return m_useWarmStart; }
          virtual void setUseWarmStart(bool useWarmStart){ m_useWarmStart = useWarmStart; }
          virtual unsigned int getMaximumIterations(){ return m_maxIter; }
          virtual bool setMaximumIterations(unsigned int maxIter);
          virtual double getMaximumTime(){ return m_maxTime; }
          virtual bool setMaximumTime(double seconds);

         protected:

           std::string           m_name;
           bool                  m_useWarmStart;   // true if the solver is allowed to warm start
           int                   m_maxIter;        // max number of iterations
           double                m_maxTime;        // max time to solve the HQP [s]
           HQPOutput             m_output;

        };
  } // namespace solver
 } // walkingctrl
} // namespace hpp

#endif // HPP_WALKINGCTRL_FWD_HH
