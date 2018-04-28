#pragma once
#include <hpp/walkingctrl/fwd.hh>

namespace hpp{
    namespace walkingctrl{
    class ConstantNdTrajectory{
        public:
            ConstantNdTrajectory(const std::string& name, const vector_t& q){
                name_ = name;
                q_ref_ = q;
                dim_ = q.size();
            };
            ~ConstantNdTrajectory(){};
            inline int getDoF() {return dim_;};
            inline vector_t getRef() {return q_ref_;};

        private:
            std::string name_;
            vector_t q_ref_;
            int dim_;
    };
    class ConstantSE3Trajectory{
        public:
            ConstantSE3Trajectory(const std::string& name, Transform3f& T){
                name_ = name;
                T_ref_ = &T;
                dim_ = 6;
            };
            ~ConstantSE3Trajectory(){};

            inline int getDoF() {return dim_;};
            inline Transform3f& getRef() {return *T_ref_;};

        private:
            std::string name_;
            Transform3f* T_ref_;
            int dim_;
    };

  }
}
