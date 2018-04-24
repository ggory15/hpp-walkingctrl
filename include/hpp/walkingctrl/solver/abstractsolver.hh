#ifndef HPP_WALKINGCTRL_ABSTRACT_SOVLER_HH
#define HPP_WALKINGCTRL_ABSTRACT_SOVLER_HH

#include <hpp/walkingctrl/fwd.hh>
#include <qpOASES.hpp>

namespace hpp {
  namespace walkingctrl {
    namespace solver{
      class AbstractSolver {
        public:
            AbstractSolver(const int & n, const int & m_in, const std::string & solver_type, const double & accuracy, const int & maxIter, const int & verb) {};
            ~AbstractSolver() {};
        private:

      };
    }
  } // namespace walingctrl
} // namespace hpp

#endif // HPP_WALKINGCTRL_FWD_HH