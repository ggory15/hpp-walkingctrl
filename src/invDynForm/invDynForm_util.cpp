# include <hpp/walkingctrl/invDynForm/invDynForm_util.hpp>

using namespace std;


namespace hpp{
    namespace walkingctrl{       
        void InvDynForm::createInvDynUtil(InterfaceSetting& setting){
           interface_setting_ = &setting;

           q0_.resize(getQsize());
           v0_.resize(getVsize());
           v0_.setZero();

           q0_ = interface_setting_->get(InterfaceVectorParam_q0);
                 
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

           this->createInvDynFormulation(q0_, v0_);
        }
        void InvDynForm::createInvDynFormulation(const vector_t q, const vector_t v){
            k_ = 0; // number of constraints
            dt_ = this->getSetting().get(InterfaceDoubleParam_dt);
            t_ = 0.0;
            Md_.resize(getQsize()-6, getQsize()-6);
            Md_.setZero();
            //baseVelocityFilter -> not use.

            S_T_.resize(getVsize(), getAsize());
            S_T_.setZero();
            S_T_.bottomLeftCorner(getAsize(),getAsize()).setIdentity();
            Nc_T_.resize(getVsize(),getVsize());
            Nc_T_.setIdentity();

            ddqMax_.resize(getQsize());
            ddqMax_.setZero();
            ddqStop_.resize(getQsize());
            ddqStop_.setZero();

            qMax_ = getRobot()->model().upperPositionLimit;
            qMin_ = getRobot()->model().lowerPositionLimit;
            dqMax_ = getRobot()->model().velocityLimit;
            tauMax_ = getRobot()->model().effortLimit.tail(getAsize());
            
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
                index_acc_in_.resize(getAsize()*2);
                for (int i=0; i<getAsize()*2; i++)
                    index_acc_in_(i) = i;
                m_in_ += 2*getAsize();    
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
                B_.resize(m_in_, getVsize()+k_+getAsize());
                B_.setZero();
                b_.resize(m_in_);
                b_.setZero();
                C_.resize(getVsize()+k_+getAsize(), getAsize());
                c_.resize(getVsize()+k_+getAsize());
                C_.setZero();
                c_.setZero();
                if (k_ != 0){
                    Jc_.resize(k_, getVsize());
                    dJc_v_.resize(k_);
                    dx_c_.resize(k_);
                    ddx_c_des_.resize(k_);
                    Jc_Minv_.resize(k_, getVsize());
                    Lambda_c_.resize(k_, k_);
                    Jc_T_pinv_.resize(k_, getVsize());

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
            //TODO-ggory igetVsize()dyan_formulation_util line 330~344
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

            C_.topLeftCorner(getVsize(), getAsize()) = Minv_ * Nc_T_*S_T_; // (18*18) * (18 x 18) * (18x12)
            C_.bottomRightCorner(getAsize(), getAsize()) = eye_12;
            c_.head(getVsize()) = -Minv_ * Nc_T_ * h_;
        }
        void InvDynForm::setNewSensorData(const double& t, const vector_t& q, const vector_t& v){
            t_ = t;
            this->setPositions(q, false);
            this->setVelocities(v);

            computeAllTerms(q_res_, v_res_);
            updateFrameKinematics();

            com_ = getCOM();
            M_ = getMassMatrix();
            h_ = getBiasTerm();
            J_com_ = getJCOM();

            for (int i=1; i<getVsize(); i++)
               M_.row(i).head(i) = M_.col(i).head(i);

            if (use_rotor_initia_){
                M_.bottomRightCorner(getAsize(), getAsize()) += Md_;
            }

            dx_com_ = J_com_ * v; 

            if (com_(2) > 0.0)
                cp_ = com_.head(2) + dx_com_.head(2)/pow(9.81/com_(2), 0.5); 
            else
                cp_.setZero();
            this->updateConstrainedDynamics();
        }
        void InvDynForm::setPositions(const vector_t& q, bool updateConstraintReference){
            q_res_ = q;
            updateforwardKinematics(q);
           
            if (updateConstraintReference){                
                this->updateSupportPolygon();
            }
        }
        void InvDynForm::addTask(JointPostureTask& task, const double& gain){
            taskstate_.push_back(task);
            w_gain_.push_back(gain);
           
            if (gain < 0.0)
                cout << "WARN::Wrong w_gain" << endl;
        }
        bool InvDynForm::existUnilateralContactConstraint(const std::string name){
             for (vector<contacts::ContactInformation>::iterator iter = rigidContactConstraints_.begin(); iter != rigidContactConstraints_.end(); ++iter){
                if (name.compare(iter->getName()))
                    return true;
            }

            return false;
        }
    }//walkingctrl
}//hpp`