#ifndef __HPP_WALKINGCTRL_CONTACT__
#define __HPP_WALKINGCTRL_CONTACT__

#include "hpp/walkingctrl/math/constraint-inequality.hpp"
#include "hpp/walkingctrl/math/constraint-equality.hpp"
namespace hpp{
    namespace walkingctrl{
        namespace contacts{
            class Contact6d{
                public:
                 EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                 typedef math::ConstRefMatrix ConstRefMatrix;
                 typedef math::ConstRefVector ConstRefVector;
                 typedef math::Vector2 Vector2;
                 typedef math::ConstraintInequality ConstraintInequality;
                 typedef math::ConstraintEquality ConstraintEquality;

                Contact6d(const std::string & name, ConstRefMatrix contactPoints, ConstRefVector contactNormal, Vector2 mu){};
                ~Contact6d(){}; 
            };
        }
    }
}

#endif
