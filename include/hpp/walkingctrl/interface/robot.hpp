#pragma once
#include <hpp/walkingctrl/fwd.hh>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>

namespace hpp{
    namespace walkingctrl{
        class RobotKinDyn{
            public:
                RobotKinDyn(DevicePtr_t robot){
                    r_ = robot;
                    nq_ = r_->configSize();
                    nv_ = nq_ - 1;
                    na_ = nv_ - 6;    
                    q_.resize(nq_);
                    v_.resize(nv_); 
                    a_.resize(na_);
                    a_.setZero();
                };
                ~RobotKinDyn(){};
                
                void updateforwardKinematics(vector_t q, vector_t v){
                     q_ = q;
                     v_ = v;                 
                     
                     se3::forwardKinematics(r_->model(), r_->data(), q_, v_, a_);
                }
                                
                void updateforwardKinematics(vector_t q){
                     q_ = q;                     
                     se3::forwardKinematics(r_->model(), r_->data(), q_);
                }


                void updateFrameKinematics(){
                    se3::framesForwardKinematics(r_->model(), r_->data());
                }

                int getFrameId(const std::string name){
                    return r_->model().getFrameId(name);
                }

                Transform3f getFrameTransformation(const int & id){ 
                    se3::Frame f = r_->model().frames[id];
                    if (f.type == se3::JOINT)
                    return r_->data().oMi[f.parent];
                    else  
                    return r_->data().oMi[f.parent] * f.placement;            
                }

                vector6_t getFrameVeolocity(const int& id){
                    se3::Frame f = r_->model().frames[id];
                    return f.placement.actInv(r_->data().v[f.parent]);
                }

                void computeAllTerms(vector_t q, vector_t v){
                    se3::computeAllTerms(r_->model(), r_->data(), q, v);
                }

                DevicePtr_t getRobot() {
                    return r_;
                }

                vector3_t getCOM(){
                    return r_->data().com[0];
                }

                matrix_t getJCOM(){
                    return r_->data().Jcom;
                }

                matrix_t getMassMatrix(){
                    return r_->data().M;
                }

                matrix_t getBiasTerm(){
                    return r_->data().nle;
                }

                int getQsize(){
                    return nq_;
                }
                int getVsize(){
                    return nv_;
                }
                int getAsize(){
                    return na_;
                }               
            
            private:
                DevicePtr_t r_;
                int na_, nq_, nv_;
                vector_t q_, v_, a_;
        };
    }
}
