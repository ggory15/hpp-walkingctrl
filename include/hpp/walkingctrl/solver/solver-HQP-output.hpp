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

#ifndef __HPP_WALKINGCTRL_SOLVER_OUTPUT__
#define __HPP_WALKINGCTRL_SOLVER_OUTPUT__

#include <hpp/walkingctrl/solver/fwd.hh>
#include <vector>


namespace hpp{
 namespace walkingctrl{
      namespace solvers
      {
        class HQPOutput
        {
        public:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW

          HQPStatus      status;    /// solver status
          vector_t x;            /// solution
          vector_t lambda;       /// Lagrange multipliers
          vector_t activeSet;  /// indexes of active inequalities
          int iterations;      /// number of iterations performed by the solver

          HQPOutput(){}

          HQPOutput(int nVars, int nEqCon, int nInCon)
          {
            resize(nVars, nEqCon, nInCon);
          }

          void resize(int nVars, int nEqCon, int nInCon)
          {
            x.resize(nVars);
            lambda.resize(nEqCon+nInCon);
            activeSet.resize(nInCon);
          }
        };
      }
  }
}

#endif // ifndef __invdyn_solvers_hqp_output_hpp__
