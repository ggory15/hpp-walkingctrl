
#include <hpp/walkingctrl/solver/abstractsolver.hh>

using namespace qpOASES;

namespace hpp{
    namespace walkingctrl{
        namespace solvers{
            AbstractSolver::AbstractSolver(const int & n, const int & m_in, const std::string & solver_type, const double & accuracy, const int & maxIter, const int & verb){
                n_ = n;
                iter_ = 0;
                solver_ = solver_type;
                accuracy_ = accuracy;
                maxIter_ = maxIter;
                verb_ = verb;
                
                m_in_ = -1;
                computationTime_= 0.0;
                qpTime_ = 0.0;
                iterationNumber_ = 0.0;
                approxProb_ = 0;

                initialized_ = false;
                nActiveInequalities_ = 0;
                nViolatedInequalities_ = 0;
                outerIter_ = 0;
                epsilon_ = 1.49e-08;
                INEQ_VIOLATION_THR = 1e-4;
                this->changeInequalityNumber(m_in, n);

            }

            void AbstractSolver::changeInequalityNumber(const int & m_in, int n){
                if (n ==-1)
                    n = n_;
                if (m_in == m_in_ && n == n_)
                    return;
                
                m_in_ = m_in;
                n_ = n;
                iter_ = 0;
                qpOasesSolver_ = SQProblem(n_, m_in);
                if (verb_ <= 0)
                    qpOption_.printLevel=PL_NONE;
                else if (verb_ == 1)
                    qpOption_.printLevel=PL_LOW;
                else if (verb_ == 2)
                    qpOption_.printLevel=PL_MEDIUM;
                else if (verb_ > 2){
                    qpOption_.printLevel=PL_DEBUG_ITER;
                    std::cout << "set high print level" << std::endl;
                }

                qpOption_.enableRegularisation = BT_TRUE;
                qpOasesSolver_.setOptions(qpOption_);
                initialized_ = false;              
            }
        }
    }
}