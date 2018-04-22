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

#include <hpp/walkingctrl/solver/basic-qpoase.hpp>
#include <hpp/walkingctrl/utils/stop-watch.hpp>
using namespace qpOASES;

namespace hpp{
namespace walkingctrl{
namespace solvers{

    Qposes_basic::Qposes_basic(){
        m_nVars = 0; // n in python
        m_nEqCon = 0;
        m_nIneqCon = -1; // m_in number of inequlities
        accuracy = 1e-6;

        m_maxIter = DEFAULT_MAX_ITER;
        iter = 0;
        verb = 0;
        computationTime = 0.0;
        qpTime = 0.0;
        approxProb = 0;

        nActiveInequalities = 0;
        nViolatedInequalities = 0;
        outerIter = 0;
        maxTime = 100.0;
    }
    void Qposes_basic::changeInequalityNumber(int m_in, int n){
        if (m_nVars != n || m_nIneqCon != m_in){
            m_nVars = n;
            m_nIneqCon = m_in;
            iter =0;

            if (verb <= 0)
                qpOption.printLevel=PL_NONE;
            else if (verb == 1)
                qpOption.printLevel=PL_LOW;
            else if (verb == 2)
                qpOption.printLevel=PL_MEDIUM;
            else if (verb > 2){
                qpOption.printLevel=PL_DEBUG_ITER;
                std::cout << "set high print level" << std::endl;
            }
            qpOption.enableRegularisation = BT_TRUE;
            qpOasesSolver = QProblem(m_nVars, m_nIneqCon);
            qpOasesSolver.setOptions(qpOption);
            initialized = false;
         }
        else {
            std::cout << "Not Changed Inequality Number" << std::endl;
        }
    }
    void Qposes_basic::setProblemData(Eigen::MatrixXd D, Eigen::VectorXd d, Eigen::MatrixXd A, Eigen::VectorXd lbA, Eigen::VectorXd ubA, Eigen::VectorXd lb, Eigen::VectorXd ub, Eigen::VectorXd x0){
        D_ = D;
        d_ = d;
        if (A.rows() == m_nVars && A.cols() == m_nIneqCon){
            A_ = A;
            lbA_ = lbA;
            ubA_ = ubA;
        }
        else
            std::cout << "ERROR: Wrong size of the constraint matrix" << std::endl;
        if (lb.size() == m_nVars && ub.size() == m_nVars){
            lb_ = lb;
            ub_ = ub;
        }
        else
            std::cout << "ERROR: Wrong size of the bound vector" << std::endl;

        H_ = D_.transpose() * D_;
        dD_ = D_.transpose() * d_;
    }
    void Qposes_basic::solve(Eigen::MatrixXd D, Eigen::VectorXd d, Eigen::MatrixXd A, Eigen::VectorXd lbA, Eigen::VectorXd ubA, Eigen::VectorXd lb, Eigen::VectorXd ub, Eigen::VectorXd x0){
        if (!warmstart){
            qpOasesSolver = QProblem(m_nVars, m_nIneqCon);
            qpOasesSolver.setOptions(qpOption);
            initialized = false;
        }

        iter = 0;
        qpTime = 0.0;
        removeSoftInequalities = false;
        setProblemData(D, d, A, lbA, ubA, lb, ub, x0);

        x_.resize(x0.size());
        x_.setZero();

        Hess_ = f_cost_hess(x_);
        grad_ = f_cost_grad(x_);
        fx_ = f_cost(x_);
        maxActiveSetIter = m_maxIter - iter;
        maxComputationTime = maxTime;
        if (!initialized){
            imode = qpOasesSolver.init(Hess_.data(), grad_.data(), A_.data(), lb_.data(), ub_.data(), lbA_.data(), ubA_.data(), maxActiveSetIter, &maxComputationTime);
            if (imode == 0)
                initialized = true;
        }
        else{
            imode = qpOasesSolver.hotstart(grad_.data(), lb_.data(), ub_.data(), lbA_.data(), ubA_.data(), maxActiveSetIter, &maxComputationTime);
            if (imode !=0){
                maxActiveSetIter = m_maxIter;
                maxComputationTime = maxTime;
                imode = qpOasesSolver.init(Hess_.data(), grad_.data(), A_.data(), lb_.data(), ub_.data(), lbA_.data(), ubA_.data(), maxActiveSetIter, &maxComputationTime);
                if (imode != 0)
                    initialized = false;

                qpTime +=maxComputationTime;
                iter = 1+ maxActiveSetIter;
                iterationNumber = iter;

                qpOasesSolver.getPrimalSolution(x_.data());
                Eigen::VectorXd ineq_marg = f_inequalities(x_);
               // bool qpUnfeasible = False;
             //   if ((ineq_marg < -1e-4).any())
            }
        }
    }
    Eigen::MatrixXd Qposes_basic::f_cost_hess(Eigen::VectorXd x){
        Eigen::VectorXd f0 = f_cost_grad(x);
        Eigen::MatrixXd H(x.size(), f0.size());
        Eigen::VectorXd dx(x.size());
        dx.setZero();

        for (int i=0; i<x.size(); i++){
            dx(i) = 1e-8;
            H.row(i) = (f_cost_grad(x+dx) - f0) / dx(i);
            dx(i) = 0.0;
        }

        return H.transpose();
    }
    Eigen::VectorXd Qposes_basic::f_cost_grad(Eigen::VectorXd x){
        double f0 = f_cost(x);
        Eigen::VectorXd grad (x.size());
        Eigen::VectorXd ei (x.size());
        Eigen::VectorXd d (x.size());
        ei.setZero();

        for (int k=0; k<x.size(); k++){
            ei(k) = 1.0;
            d = 1e-8 * ei;
            grad(k) = (f_cost(x+d)-f0) / d(k);
            ei(k) = 0.0;
        }

        return grad;
    }
    double Qposes_basic::f_cost(Eigen::VectorXd x){
        return (D_.transpose()*x - d_).norm();
    }
    Eigen::VectorXd Qposes_basic::f_inequalities(Eigen::VectorXd x){
        VectorXd ineq_marg(2*m_nIneqCon);
        ineq_marg.head(m_nIneqCon) = A_ * x - lbA_;
        ineq_marg.tail(m_nIneqCon) = ubA_ - A_*x;

        return ineq_marg;
    }








}  //solver
} //walkningctrl
} // hpp

