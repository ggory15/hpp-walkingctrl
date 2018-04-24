# include <hpp/walkingctrl/invDynForm/invDynForm_util.hpp>

#include "pinocchio/algorithm/kinematics.hpp"

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
        InvDynForm::InvDynForm(const DevicePtr_t & model) {
           r_ = model;
           nq_ = r_->configSize();
           nv_ = nq_ - 1;
           na_ = nv_ - 6;    
           q_.resize(nq_);
           v_.resize(nv_); 
           
           com_pinocchio_ = CenterOfMassComputation::create (r_);
           com_pinocchio_->add (r_->rootJoint ());
           bias_pinocchio_ = BiasComputation::create (r_);
        }
        
        void InvDynForm::createInvDynUtil(InterfaceSetting& setting){
           interface_setting_ = &setting;

        // set Init-posture
           q_ = interface_setting_->get(InterfaceVectorParam_q0);
           v_.setZero();
                 
        // set bool
           use_com_gen_ = false;
           enableCapturePoint_ = interface_setting_->get(InterfaceBoolParam_ENABLE_CAPTURE_POINT_LIMITS);
           enableTorqueLimit_ = interface_setting_->get(InterfaceBoolParam_ENABLE_TORQUE_LIMITS);   
           enableForceLimit_ = interface_setting_->get(InterfaceBoolParam_ENABLE_FORCE_LIMITS);   
           enableJointLimit_ = interface_setting_->get(InterfaceBoolParam_ENABLE_JOINT_LIMITS);   
           use_pos_preview_ = interface_setting_->get(InterfaceBoolParam_IMPOSE_POSITION_BOUNDS);
           use_vel_preview_ = interface_setting_->get(InterfaceBoolParam_IMPOSE_VELOCITY_BOUNDS);
           use_max_joint_acc_ = interface_setting_->get(InterfaceBoolParam_IMPOSE_ACCELERATION_BOUNDS);
           use_vel_estimator_ = interface_setting_->get(InterfaceBoolParam_USE_JOINT_VELOCITY_ESTIMATOR);
           use_rotor_initia_ = interface_setting_->get(InterfaceBoolParam_ACCOUNT_FOR_ROTOR_INERTIAS);

           support_polygon_computed_ = false;

           this->createInvDynFormulation(q_, v_);
        }

        void InvDynForm::createInvDynFormulation(const vector_t q, const vector_t v){
            k_ = 0; // number of constraints
            dt_ = this->getSetting().get(InterfaceDoubleParam_dt);
            t_ = 0.0;
            Md_.resize(nq_-6, nq_-6);
            Md_.setZero();
            //baseVelocityFilter -> not use.

            S_T_.resize(nv_, na_);
            S_T_.setZero();
            S_T_.bottomLeftCorner(na_,na_).setIdentity();
            Nc_T_.resize(nv_,nv_);
            Nc_T_.setIdentity();

            ddqMax_.resize(nq_);
            ddqMax_.setZero();
            ddqStop_.resize(nq_);
            ddqStop_.setZero();

            qMax_ = r_->model().upperPositionLimit;
            qMin_ = r_->model().lowerPositionLimit;
            dqMax_ = r_->model().velocityLimit;
            tauMax_ = r_->model().effortLimit.tail(na_);
            
            for (int i=0; i<7; i++){
                qMax_(i) = 1e100;
                qMin_(i)  = -1e100;
            }

            contact_points_ = vector3_t::Zero();
            this->updateInequalityData(false);
            this->setNewSensorData(0, q, v);
        }
        void InvDynForm::updateInequalityData(bool updateConstrainedDynamics, bool doNotUpdateForceLimits){
            this->updateSupportPolygon();

            m_in_ = 0;
            int c = rigidContactConstraints_.size(); // number of unilateral contacts;

            if (!doNotUpdateForceLimits){
                if (enableForceLimit_){

                }
            } 

            if (enableJointLimit_){
                index_acc_in_.resize(na_*2);
                for (int i=0; i<na_*2; i++)
                    index_acc_in_(i) = i;
                m_in_ += 2*na_;    
            }
            if (enableTorqueLimit_){
                lb_ = -1.*tauMax_;
                ub_ = tauMax_;
            }
            //if (enableCapturePoint_){
            //    index_cp_in_ = 
            //}
            matrix_t A; vector_t a;
            this->computeConstraintTask(0.0, A, a);

            if(doNotUpdateForceLimits){
                //TODO
            }
            else{                
                B_.resize(m_in_, nv_+k_+na_);
                B_.setZero();
                b_.resize(m_in_);
                b_.setZero();
                C_.resize(nv_+k_+na_, na_);
                c_.resize(nv_+k_+na_);
                C_.setZero();
                c_.setZero();
                if (k_ != 0){
                    Jc_.resize(k_, nv_);
                    dJc_v_.resize(k_);
                    dx_c_.resize(k_);
                    ddx_c_des_.resize(k_);
                    Jc_Minv_.resize(k_, nv_);
                    Lambda_c_.resize(k_, k_);
                    Jc_T_pinv_.resize(k_, nv_);


                    Jc_.setZero();
                    dJc_v_.setZero();
                    dx_c_.setZero();
                    ddx_c_des_.setZero();
                    Jc_Minv_.setZero();
                    Lambda_c_.setZero();
                    Jc_T_pinv_.setZero();
                }

                if (enableForceLimit_ && k_ >0){

                }
            }
            if (updateConstrainedDynamics){
                this->updateConstrainedDynamics();
            }



        }
        void InvDynForm::updateSupportPolygon(){
            //TODO-ggory inv_dyan_formulation_util line 330~344
            int ncp = 0;
            contact_points_ = vector3_t::Zero();
            contact_normal_ = vector3_t::Zero();
            int i = 0;

            if (ncp == 0){
               // cout << "Init Polygon" << endl;
            }
            else{
               //TODO
            }

        }
        void InvDynForm::computeConstraintTask(const double& t, matrix_t& A, vector_t& a){
            //TODO
            int n_task = 0;
            if (n_task == 0){

            }
        }
        void InvDynForm::updateConstrainedDynamics(){
            int i=0;

            Minv_ = M_.inverse();

            if (k_ > 0){

            }
            else
                Nc_T_.setIdentity();

            matrix_t eye_12(12,12);
            eye_12.setIdentity();

            C_.topLeftCorner(nv_, na_) = Minv_ * Nc_T_*S_T_; // (18*18) * (18 x 18) * (18x12)
            C_.bottomRightCorner(na_, na_) = eye_12;
            c_.head(nv_) = -Minv_ * Nc_T_ * h_;
        }
        void InvDynForm::setNewSensorData(const double& t, const vector_t& q, const vector_t& v){
            t_ = t;
            this->setPositions(q, false);
            this->setVelocities(v);
            this->getDynamics();

            if (use_rotor_initia_){
                M_.bottomRightCorner(na_, na_) += Md_;
            }
            dx_com_ = J_com_ * v; 
            if (com_(2) > 0.0)
                cp_ = com_.head(2) + dx_com_.head(2)/pow(9.81/com_(2), 0.5); 
            else
                cp_.setZero();
            this->updateConstrainedDynamics();
        }
        void InvDynForm::setPositions(const vector_t& q, bool updateConstraintReference){
            q_ = q;
            r_->currentConfiguration(q_);
            r_->computeForwardKinematics();

            if (updateConstraintReference){
                
                this->updateSupportPolygon();
            }
        }
        void InvDynForm::getDynamics(){
            com_pinocchio_ ->compute(hpp::pinocchio::Device::COM);
            com_ = com_pinocchio_->com();
            com_pinocchio_ ->compute(hpp::pinocchio::Device::JACOBIAN);
                   
            J_com_ = com_pinocchio_->jacobian();
            bias_pinocchio_ ->compute(q_, v_);

            M_ = bias_pinocchio_->M();
            for (int i=1; i<nv_; i++)
                M_.row(i).head(i) = M_.col(i).head(i);

            h_ = bias_pinocchio_->nle();
        }


        void InvDynForm::addTask(JointPostureTask& task, const double& gain){
            taskstate_.push_back(task);
            w_gain_.push_back(gain);

            if (gain < 0.0)
                cout << "WARN::Wrong w_gain" << endl;
        }


    }//walkingctrl
}//hpp`