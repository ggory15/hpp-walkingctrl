#pragma once

#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <hpp/walkingctrl/yaml_eigen.h>
#include <hpp/walkingctrl/interface/Definitions.hpp>
#include <hpp/walkingctrl/interface/InterfaceParam.hpp>

namespace hpp{
    namespace walkingctrl{

    class InterfaceSetting{
        public:
            InterfaceSetting(){}
            ~InterfaceSetting(){}

            void initialize(const std::string& cfg_file,  const std::string& planner_vars_yaml = "interface_variables");

            const int& get(InterfaceIntParam param) const;
            const bool& get(InterfaceBoolParam param) const;
            const double& get(InterfaceDoubleParam param) const;
            const std::string& get(InterfaceStringParam param) const;
            const Eigen::Ref<const Eigen::VectorXd> get(InterfaceVectorParam param) const;
            const Eigen::Ref<const Eigen::VectorXi> get(InterfaceIntVectorParam param) const;
        private:
             std::string cfg_file_, savefile_;

             int test_duration_, maxIter_, verb_;
             bool capture_point_limit_, torque_limit_, force_limit_, joint_limit_, position_bounds_, velocity_bounds_,
                  viability_bounds_, acc_bounds_, use_joint_vel_estimator_, use_roter_inertia_, use_lcp_solver_, save_result_;
             
             double dt_, joint_pos_preview_, joint_vel_preview_, max_joint_acc_, max_min_joint_acc_, kp_posture_, kd_posture_, kp_constr_, kd_constr_, kp_com_, kd_com_,
                    w_com_, w_posture_, maxTime_, fmin_, max_com_vel_, max_constraint_err_;

              Eigen::Vector2d mu_;
              Eigen::Vector3d de_contact_point_, de_contact_normal_;
              Eigen::VectorXd q0_;
              Eigen::VectorXi constraint_mask_;
    };

  }
}
