#pragma once
#include <hpp/walkingctrl/fwd.hh>
#include <hpp/walkingctrl/trajectory/trajectory.hh>

namespace hpp{
    namespace walkingctrl{
        class JointPostureTask : public ConstantNdTrajectory {
            public:
                //JointPostureTask(){};
                ~JointPostureTask(){};

                JointPostureTask(const std::string& name, const vector_t& q): ConstantNdTrajectory(name, q){
                    mask_.resize(getDoF());
                    jacobian_.resize(getDoF(), getDoF()+6);
                    jacobian_.bottomRightCorner(getDoF(), getDoF()).setIdentity();
                }

                inline void setgain(const double& kp, const double& kv) {kp_ = kp; kv_ = kv;};

            private:
            // ConstandNdTrajectory trajectory_;
                Eigen::VectorXi mask_;
                matrix_t jacobian_;
                double kp_, kv_;
        };
        class SE3Task : public ConstantSE3Trajectory {
            public: 
                SE3Task(const DevicePtr_t & robot, const int & id, Transform3f& oMi, const std::string & name) : ConstantSE3Trajectory(name, oMi){
                    r_ = robot;

                    for (int i=0; i<3; i++)
                        masks_.push_back(true);
                    for (int i=0;i<3;i++)
                        masks_.push_back(false);
                }
                ~SE3Task(){}; 
                inline void setgain(const double& kp, const double& kv) {kp_ = kp; kv_ = kv;};

            private:
                DevicePtr_t r_;
                double kp_;
                double kv_;
                std::vector<bool> masks_;
        };
    }
}
