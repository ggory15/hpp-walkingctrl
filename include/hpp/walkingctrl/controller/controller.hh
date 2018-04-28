#ifndef HPP_WALKINGCTRL_CONTROLLER_HH
#define HPP_WALKINGCTRL_CONTROLLER_HH

#include <hpp/walkingctrl/fwd.hh>
#include <hpp/walkingctrl/invDynForm/invDynForm_util.hpp>
#include <hpp/walkingctrl/simulator/simulator.hpp>
#include <hpp/walkingctrl/solver/standardQPSolver.hh>

namespace hpp {
  namespace walkingctrl {

    struct ControlVariable{
        public: 
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            ControlVariable(int size);
            ~ControlVariable(){}

            vector_t& getJointAngle() {return q_;}
            vector_t& getJointVelocity() {return v_;}
            double& time() {return t_;}
            const vector_t& getJointAngle() const {return q_;}
            const vector_t& getJointVelocity() const {return v_;}
            const double& time() const {return t_;}

       private:
            vector_t q_, v_;
            double t_;
    };
    class ControlResult{
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW   
            ControlResult(){}
            ~ControlResult(){}

            void resize(int duration, int size);
            void clean() {return control_result_.clear();}
            int size() {return control_result_.size();}

            ControlVariable& controlVariable (int time_id) {return control_result_[time_id];}  
            const ControlVariable& controlVariable (int time_id) const {return control_result_[time_id];}  
        private:
           std::vector<ControlVariable> control_result_;

    };

    class Controller {
        public:
            Controller(InvDynForm* invdyn, Simulator* simul, solvers::StandardQpSolver* solver);
            ~Controller() {};
            void startSimulation();

        private:
            bool updateConstraints(const int& t, const int& i, const vector_t q, const vector_t v);    

        private:
            InvDynForm* invdyn_;
            Simulator* simulator_;
            solvers::StandardQpSolver* QPsolver_;

            ControlResult& controlResult() { return control_result_; }
            const ControlResult&controlResult() const { return control_result_; }

        private:
            double t_, dt_;
            int contact_switch_index_;
            vector_t q0_, v0_, torque_;
            vector3_t Pi_, Ni_;
            ControlResult control_result_;
    };

   
    

  } // namespace walingctrl
} // namespace hpp

#endif // HPP_WALKINGCTRL_FWD_HH