#include <iostream>
#include <hpp/walkingctrl/interface/Setting.hpp>

namespace hpp{
    namespace walkingctrl{
        void InterfaceSetting::initialize(const std::string& cfg_file, const std::string& planner_vars_yaml)
        {
            cfg_file_ = cfg_file;
            
            savefile_ = cfg_file.substr(0, cfg_file.size()-5) + "_results.yaml";

            try {
                YAML::Node interface_cfg = YAML::LoadFile(cfg_file.c_str());
                YAML::Node interface_vars = interface_cfg[planner_vars_yaml.c_str()];
                
                // read int  test_duration_, maxIter_, verv_
                readParameter(interface_vars, "MAX_TEST_DURATION", test_duration_);
                readParameter(interface_vars, "maxIter", maxIter_);
                readParameter(interface_vars, "verb", verb_);

                // read bool
                readParameter(interface_vars, "ENABLE_CAPTURE_POINT_LIMITS", capture_point_limit_);
                readParameter(interface_vars, "ENABLE_TORQUE_LIMITS", torque_limit_);
                readParameter(interface_vars, "ENABLE_FORCE_LIMITS", force_limit_);
                readParameter(interface_vars, "ENABLE_JOINT_LIMITS", joint_limit_);
                readParameter(interface_vars, "IMPOSE_POSITION_BOUNDS", position_bounds_);
                readParameter(interface_vars, "IMPOSE_VELOCITY_BOUNDS", velocity_bounds_);
                readParameter(interface_vars, "IMPOSE_VIABILITY_BOUNDS", viability_bounds_);
                readParameter(interface_vars, "IMPOSE_ACCELERATION_BOUNDS", acc_bounds_);
                readParameter(interface_vars, "USE_JOINT_VELOCITY_ESTIMATOR", use_joint_vel_estimator_);
                readParameter(interface_vars, "ACCOUNT_FOR_ROTOR_INERTIAS", use_roter_inertia_);
//                readParameter(interface_vars, "ENABLE_CAPTURE_POINT_LIMITS", use_lcp_solver_);
                readParameter(interface_vars, "SAVE", save_result_);

                // read double
                readParameter(interface_vars, "dt", dt_);
                readParameter(interface_vars, "JOINT_POS_PREVIEW", joint_pos_preview_);
                readParameter(interface_vars, "JOINT_VEL_PREVIEW", joint_vel_preview_);
                readParameter(interface_vars, "MAX_JOINT_ACC", max_joint_acc_);
                readParameter(interface_vars, "kp_posture", kp_posture_);
                readParameter(interface_vars, "kp_constr", kp_constr_);
                readParameter(interface_vars, "kp_com", kp_com_);
                readParameter(interface_vars, "w_com", w_com_);
                readParameter(interface_vars, "w_posture", w_posture_);
                readParameter(interface_vars, "maxTime", maxTime_);
                readParameter(interface_vars, "fMin", fmin_);
                readParameter(interface_vars, "MAX_COM_VELOCITY", max_com_vel_);
                readParameter(interface_vars, "MAX_CONSTRAINT_ERROR", max_constraint_err_);

                kd_posture_ = 2*sqrt(kp_posture_);
                kd_constr_ = 2*sqrt(kp_constr_);
                kd_com_ = 2*sqrt(kp_com_);           
                
                //read vector 
                readParameter(interface_vars, "mu", mu_);
                readParameter(interface_vars, "DEFAULT_CONTACT_POINTS", de_contact_point_);
                readParameter(interface_vars, "DEFAULT_CONTACT_NORMALS", de_contact_normal_);
                readParameter(interface_vars, "constraint_mask", constraint_mask_);
                readParameter(interface_vars, "q0", q0_);
            }
            catch (std::runtime_error& e)
            {
                std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
            }
        }
        const int& InterfaceSetting::get(InterfaceIntParam param) const{
            switch (param){
                case InterfaceIntParam_MAX_TEST_DURATION : {return test_duration_;}
                case InterfaceIntParam_maxIter : {return maxIter_;}
                case InterfaceIntParam_verb : {return verb_;}
                
                default: { throw std::runtime_error("InterfaceSetting::get InterfaceIntPARAM invalid"); break; }
            }
        }
        const bool& InterfaceSetting::get(InterfaceBoolParam param) const{
            switch (param){
                case InterfaceBoolParam_ENABLE_CAPTURE_POINT_LIMITS : {return capture_point_limit_;}
                case InterfaceBoolParam_ENABLE_TORQUE_LIMITS : {return torque_limit_;}
                case InterfaceBoolParam_ENABLE_FORCE_LIMITS : {return force_limit_;}
                case InterfaceBoolParam_ENABLE_JOINT_LIMITS : {return joint_limit_;}
                case InterfaceBoolParam_IMPOSE_POSITION_BOUNDS : {return position_bounds_;}
                case InterfaceBoolParam_IMPOSE_VELOCITY_BOUNDS : {return velocity_bounds_;}
                case InterfaceBoolParam_IMPOSE_ACCELERATION_BOUNDS : {return acc_bounds_;}
                case InterfaceBoolParam_IMPOSE_VIABILITY_BOUNDS : {return viability_bounds_;}
                case InterfaceBoolParam_USE_JOINT_VELOCITY_ESTIMATOR : {return use_joint_vel_estimator_;}
                case InterfaceBoolParam_ACCOUNT_FOR_ROTOR_INERTIAS : {return use_roter_inertia_;}
                case InterfaceBoolParam_SAVE_RESULT : {return save_result_;}
                               
                default: { throw std::runtime_error("InterfaceSetting::get InterfaceBoolParam invalid"); break; }
            }
        }
        const double& InterfaceSetting::get(InterfaceDoubleParam param) const{
            switch (param){
                case InterfaceDoubleParam_dt : {return dt_;}
                case InterfaceDoubleParam_JOINT_POS_PREVIEW : {return joint_pos_preview_;}
                case InterfaceDoubleParam_JOINT_VEL_PREVIEW : {return joint_vel_preview_;}
                case InterfaceDoubleParam_MAX_JOINT_ACC : {return max_joint_acc_;}
                case InterfaceDoubleParam_MAX_MIN_JOINT_ACC : {return max_min_joint_acc_;}
                case InterfaceDoubleParam_kp_posture : {return kp_posture_;}
                case InterfaceDoubleParam_kd_posture : {return kd_posture_;}
                case InterfaceDoubleParam_kp_constr : {return kp_constr_;}
                case InterfaceDoubleParam_kd_constr : {return kd_constr_;}
                case InterfaceDoubleParam_kp_com : {return kp_com_;}
                case InterfaceDoubleParam_kd_com : {return kd_com_;}
                case InterfaceDoubleParam_w_com : {return w_com_;}
                case InterfaceDoubleParam_w_posture : {return w_posture_;}
                case InterfaceDoubleParam_maxTime : {return maxTime_;}
                case InterfaceDoubleParam_fmin : {return fmin_;}
                case InterfaceDoubleParam_MAX_COM_VELOCITY : {return max_com_vel_;}
                case InterfaceDoubleParam_MAX_CONSTRAINT_ERROR : {return max_constraint_err_;}
                               
                default: { throw std::runtime_error("InterfaceSetting::get InterfaceDoubleParam invalid"); break; }
            }       
        }
        const Eigen::Ref<const Eigen::VectorXd> InterfaceSetting::get(InterfaceVectorParam param) const{
            switch (param)            {
                case InterfaceVectorParam_DEFAULT_CONTACT_NORMALS : {return de_contact_normal_;}
                case InterfaceVectorParam_DEFAULT_CONTACT_POINTS : {return de_contact_point_;}
                case InterfaceVectorParam_mu : {return mu_;}
                case InterfaceVectorParam_q0 : {return q0_;}

                default: { throw std::runtime_error("InterfaceSettingtting::get InterfaceVectorParam invalid"); break; }
            }

        }
        const Eigen::Ref<const Eigen::VectorXi> InterfaceSetting::get(InterfaceIntVectorParam param) const{
            switch (param)            {
                case InterfaceIntVectorParam_constraint_mask : {return constraint_mask_;}
                default: { throw std::runtime_error("InterfaceSetting::get InterfaceIntVectorParam invalid"); break; }
            }
        }
        const std::string& InterfaceSetting::get(InterfaceStringParam param) const
        {
            switch (param)            {
                // Storage information
                case InterfaceStringParam_ConfigFile : { return cfg_file_; }
                case InterfaceStringParam_SaveDynamicsFile : { return savefile_; }

                // Not handled parameters
                default: { throw std::runtime_error("InterfaceSetting::get InterfaceStringParam invalid"); break; }
            }
        }
   } // walkingctrl
} // hpp
