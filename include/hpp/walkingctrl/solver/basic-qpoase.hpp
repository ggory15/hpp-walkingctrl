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

#ifndef QPOSASE_HPP
#define QPOASE_HPP

#include <Eigen/Dense>
#include <hpp/walkingctrl/math/fwd.hpp>
#include <qpOASES.hpp>

#define DEBUG_STREAM(msg)
#define DEFAULT_MAX_ITER 1000

using namespace qpOASES;

namespace hpp{
    namespace walkingctrl{
        namespace solvers{

        enum QPOSASE_status
        {
          QPOSASE_OPTIMAL=0,
          QPOSASE_INFEASIBLE=1,
          QPOSASE_UNBOUNDED=2,
          QPOSASE_MAX_ITER_REACHED=3,
          QPOSASE_REDUNDANT_EQUALITIES=4
        };

        class Qposes_basic{
            typedef Eigen::MatrixXd MatrixXd;
            typedef Eigen::VectorXd VectorXd;
            typedef Eigen::VectorXi VectorXi;

        public:

          EIGEN_MAKE_ALIGNED_OPERATOR_NEW
          Qposes_basic();
          virtual ~Qposes_basic() {};

          void setSoftInequalityIndexes (math::Index index) { softInequalityIndexes = index; };
          void changeInequalityNumber(int m_in, int n);
          void setProblemData(Eigen::MatrixXd D, Eigen::VectorXd d, Eigen::MatrixXd A, Eigen::VectorXd lbA, Eigen::VectorXd ubA, Eigen::VectorXd lb, Eigen::VectorXd ub, Eigen::VectorXd x0);
          void setWarmStart(bool a) {warmstart = a;};
          void setMaxTime(double time) {maxTime = time;};
          void solve(Eigen::MatrixXd D, Eigen::VectorXd d, Eigen::MatrixXd A, Eigen::VectorXd lbA, Eigen::VectorXd ubA, Eigen::VectorXd lb, Eigen::VectorXd ub, Eigen::VectorXd x0);

          double f_cost(Eigen::VectorXd x);
          Eigen::MatrixXd f_cost_hess(Eigen::VectorXd x);
          Eigen::VectorXd f_cost_grad(Eigen::VectorXd x);
          Eigen::VectorXd f_inequalities(Eigen::VectorXd x);

        private:
          math::Index m_nVars;
          math::Index m_nEqCon;
          math::Index m_nIneqCon;

          Eigen::MatrixXd D_; //quadratic cost matrix
          Eigen::VectorXd d_; //quadratic cost vector;
          Eigen::MatrixXd G_; // inequality matrix
          Eigen::VectorXd g_; // inequality vector

          Eigen::MatrixXd A_;
          Eigen::VectorXd lbA_, ubA_, lb_, ub_;
          Eigen::VectorXd bounds;


          Eigen::MatrixXd Hess_, H_; // Hessian
          Eigen::VectorXd grad_;
          double fx_;

          Eigen::MatrixXd dD_;



          Eigen::VectorXd x0_, x_; // initial geuss
          math::Index softInequalityIndexes;

          double accuracy;
          int m_maxIter, verb;  /// max number of active-set iterations

          int iter;
          int imode;
          double computationTime, qpTime, maxTime, maxComputationTime;
          int iterationNumber, maxActiveSetIter;
          int approxProb;

          bool initialized, warmstart, removeSoftInequalities;
          int nActiveInequalities, nViolatedInequalities;
          int outerIter;

          QProblem qpOasesSolver;
          Options qpOption;

        }; //class
        } //sovler
    }//walking ctlr
}//hpp



#endif
