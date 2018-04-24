#pragma once
#include <hpp/walkingctrl/fwd.hh>

namespace hpp{
    namespace walkingctrl{
    class ConstandNdTrajectory{
        public:
            ConstandNdTrajectory(const std::string& name, const vector_t& q){
                name_ = name;
                q_ref_ = q;
                dim_ = q.size();
            };
            ~ConstandNdTrajectory(){};
            inline int getDoF() {return dim_;};
            inline vector_t getRef() {return q_ref_;};

        private:
            std::string name_;
            vector_t q_ref_;
            int dim_;
    };
  }
}
