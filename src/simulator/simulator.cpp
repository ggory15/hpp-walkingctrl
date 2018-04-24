# include <hpp/walkingctrl/simulator/simulator.hpp>

using namespace hpp::walkingctrl::math;
using namespace std;
using namespace se3;

using hpp::pinocchio::HumanoidRobot;
using hpp::pinocchio::HumanoidRobotPtr_t;
using namespace hpp::pinocchio::urdf;
using hpp::pinocchio::Configuration_t;
using hpp::pinocchio::CenterOfMassComputation;
using hpp::pinocchio::BiasComputation;
using hpp::pinocchio::CenterOfMassComputationPtr_t;
using hpp::pinocchio::Device;
using hpp::pinocchio::DevicePtr_t;

namespace hpp{
    namespace walkingctrl{
        Simulator::Simulator(const DevicePtr_t & model, bool detectContactPoint) {
            r_ = model;
            nq_ = r_->configSize();
            nv_ = nq_ - 1;
            na_ = nv_ - 6;    
            v_.resize(nv_);
            v_.setZero();
            
            com_pinocchio_ = CenterOfMassComputation::create (r_);
            com_pinocchio_->add (r_->rootJoint ());
            bias_pinocchio_ = BiasComputation::create (r_);

            DETECT_CONTACT_POINTS = detectContactPoint;
        }
        void Simulator::createSimulator(InterfaceSetting& setting){
            interface_setting_ = &setting;
            mu_ = interface_setting_->get(InterfaceVectorParam_mu);
            verb_ = interface_setting_->get(InterfaceIntParam_verb);
            fmin_ = interface_setting_->get(InterfaceDoubleParam_fmin);
            dt_ = interface_setting_->get(InterfaceDoubleParam_dt);
            q_ = interface_setting_->get(InterfaceVectorParam_q0);            
            
            LCP_ = new solver::StaggeredProjections(nv_, mu_(0));  
            this->reset(0, q_, v_, dt_);
        }
        void Simulator::reset(const double & time, const vector_t& q, const vector_t & v, const double & dt){
            Md_.resize(nv_, nv_);
            Md_.setZero();
            q_ =  q;
            vold_ = v;
            dv_.resize(nv_);
            dv_.setZero();
            S_T_.resize(nv_+6, nv_);
            S_T_.topRightCorner(nv_, nv_).setIdentity();
            this->getDynamics();

            qMax_ = r_->model().upperPositionLimit;
            qMin_ = r_->model().lowerPositionLimit;
            dqMax_ = r_->model().velocityLimit;
            tauMax_ = r_->model().effortLimit.tail(na_);
            
            for (int i=0; i<7; i++){
                qMax_(i) = 1e100;
                qMin_(i)  = -1e100;
            }

            rigidContactConstraints_.clear();
            this->updateInequalityData();
        }
        void Simulator::getDynamics(){
            com_pinocchio_ ->compute(hpp::pinocchio::Device::COM);
            com_ = com_pinocchio_->com();
            com_pinocchio_ ->compute(hpp::pinocchio::Device::JACOBIAN);                   
            J_com_ = com_pinocchio_->jacobian();
            bias_pinocchio_ ->compute(q_, v_);

            M_ = bias_pinocchio_->M();
            for (int i=1; i<nv_; i++)
                M_.row(i).head(i) = M_.col(i).head(i);
        }
        void Simulator::updateInequalityData(){
            int c = rigidContactConstraints_.size();
            if (DETECT_CONTACT_POINTS)
                k_ = 3*c;
            else
                k_ = 6*c;    
            
            if (k_ != 0){

            }

            C_.resize(nv_+k_+na_, na_);
            C_.setZero();
            C_.topRightCorner(na_,na_).setIdentity();
            c_.resize(nv_+k_+na_);
            c_.setZero();
        }


    }
};