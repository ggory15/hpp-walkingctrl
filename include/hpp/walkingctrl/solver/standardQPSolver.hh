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

#ifndef HPP_WALKINGCTRL_STANDARD_QP_HH
#define HPP_WALKINGCTRL_STANDARD_QP_HH

#include <hpp/walkingctrl/solver/abstractsolver.hh>

namespace hpp {
  namespace walkingctrl {
    namespace solver{
      class StandardQpSolver : public AbstractSolver {
        public: 
          ~StandardQpSolver(){};

          StandardQpSolver(const int & n, const int & m_in, const std::string & solver_type = "qpoase", const double & accuracy = 1e-6, const int & maxIter = 100, const int & verb =0)
          : AbstractSolver(n, m_in, solver_type, accuracy, maxIter, verb){}
        private:

      };
    }
  } // namespace walingctrl
} // namespace hpp

#endif // HPP_WALKINGCTRL_FWD_HH
