#pragma once
#include <hpp/walkingctrl/fwd.hh>
#include <hpp/walkingctrl/trajectory/trajectory.hh>

namespace hpp{
    namespace walkingctrl{
    class JointPostureTask : public ConstandNdTrajectory {
        public:
            //JointPostureTask(){};
            ~JointPostureTask(){};

            JointPostureTask(const std::string& name, const vector_t& q): ConstandNdTrajectory(name, q){
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
  }
}