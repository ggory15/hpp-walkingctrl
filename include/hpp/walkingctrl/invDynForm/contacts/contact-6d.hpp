#ifndef __HPP_WALKINGCTRL_CONTACT__
#define __HPP_WALKINGCTRL_CONTACT__

#include <hpp/walkingctrl/fwd.hh>

namespace hpp{
    namespace walkingctrl{
        namespace contacts{
            class ContactInformation{
                public:
                 EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                    ContactInformation(const std::string & name="default", const vector3_t & contactPoint = vector3_t::Zero(), const vector3_t & contactNormal = vector3_t::Zero(), const vector2_t& mu = vector2_t::Zero()) {
                        name_ = name; 
                        contactPoint_ = contactPoint;
                        contactNormal_ = contactNormal;
                        mu_ = mu_;
                    };
                    ~ContactInformation(){}; 

                    std::string getName() {return name_;}
                    vector3_t getContactPoint() {return contactPoint_;}
                    vector3_t getContactNormal() {return contactNormal_;}
                    vector2_t getFriectionCoeff() {return mu_;}

                private:
                    std::string name_;
                    vector3_t contactPoint_, contactNormal_;
                    vector2_t mu_;


            };
        }
    }
}

#endif
