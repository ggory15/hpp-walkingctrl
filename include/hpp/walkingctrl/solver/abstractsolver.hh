#ifndef HPP_WALKINGCTRL_ABSTRACT_SOVLER_HH
#define HPP_WALKINGCTRL_ABSTRACT_SOVLER_HH

#include <hpp/walkingctrl/fwd.hh>
#include <qpOASES.hpp>

namespace hpp {
  namespace walkingctrl {
    namespace solvers{
      class AbstractSolver {
        public:
            AbstractSolver(const int & n, const int & m_in, const std::string & solver_type, const double & accuracy, const int & maxIter, const int & verb);
            ~AbstractSolver() {};
            vector_t get_dvec() {return d_;}
            matrix_t get_Dmat() {return D_;}
            matrix_t get_Hmat() {return H_;}
            vector_t get_dDvec() {return dD_;}
            matrix_t get_ineqMat() {return A_;}
            vector_t get_upperbound() {return ubA_;}
            vector_t get_lowerbound() {return lbA_;}
            
        private:
          void changeInequalityNumber(const int & m_in, int n = -1);

        private:
            int n_, m_in_, maxIter_, verb_, iter_, iterationNumber_, approxProb_, nActiveInequalities_, nViolatedInequalities_, outerIter_;
            double accuracy_, computationTime_, qpTime_, epsilon_, INEQ_VIOLATION_THR;
            std::string solver_;
            matrix_t D_, G_, H_, dD_, A_;
            vector_t d_, g_, x0_, lbA_, ubA_;
            bool initialized_;

            qpOASES::QProblem qpOasesSolver_;
            qpOASES::Options qpOption_;
            qpOASES::SolutionAnalysis qpOasesAnalyser_;
      };
    }
  } // namespace walingctrl
} // namespace hpp

#endif // HPP_WALKINGCTRL_FWD_HH