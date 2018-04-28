#include <hpp/walkingctrl/controller/controller.hh>

using namespace std;
using namespace hpp::walkingctrl::solvers;

namespace hpp{
    namespace walkingctrl{
        Controller::Controller(InvDynForm* invdyn, Simulator* simul, StandardQpSolver* solver) {
            invdyn_ = invdyn; 
            simulator_ = simul;
            QPsolver_ = solver;

            dt_ = invdyn_->getSetting().get(InterfaceDoubleParam_dt);

            controlResult().resize(invdyn_->getSetting().get(InterfaceIntParam_MAX_TEST_DURATION), invdyn_->getAsize());
            for (int i=0; i< invdyn_->getSetting().get(InterfaceIntParam_MAX_TEST_DURATION); i++)
                controlResult().controlVariable(i).time() = dt_ * i;

            torque_.resize(invdyn_->getAsize());
            torque_.setZero();

            Pi_ = invdyn_->getSetting().get(InterfaceVectorParam_DEFAULT_CONTACT_POINTS);  
        }    
        void Controller::startSimulation(){
            t_ = 0.0;
            controlResult().controlVariable(0).getJointAngle() = invdyn_->getSetting().get(InterfaceVectorParam_q0);
            controlResult().controlVariable(0).getJointVelocity().setZero();

            contact_switch_index_ = 0;
            simulator_->reset(t_, controlResult().controlVariable(0).getJointAngle(), controlResult().controlVariable(0).getJointVelocity(), dt_);

            bool updateCon = this->updateConstraints(t_, 0, simulator_->getjointAngle(), simulator_->getjointVelocity());

        }
        bool Controller::updateConstraints(const int& t, const int& i, const vector_t q, const vector_t v){
            bool contact_changed = false;
            vector<string> contactset = invdyn_->getContactSet();

            for (vector<string>::iterator iter=contactset.begin() ; iter != contactset.end() ; ++iter){
                if (invdyn_->existUnilateralContactConstraint(*iter))
                    continue;

                contact_changed = true;
                invdyn_->updateforwardKinematics(q, v);
                invdyn_ ->updateFrameKinematics();

                int fid = invdyn_->getFrameId(*iter);
                Transform3f oMi = invdyn_->getFrameTransformation(fid);
                SE3Task constr(invdyn_->getRobot(), fid, oMi, *iter);
                constr.setgain(invdyn_->getSetting().get(InterfaceDoubleParam_kp_constr), invdyn_->getSetting().get(InterfaceDoubleParam_kd_constr));

                cout << "Adding Contact:" << " " <<  *iter  << ",  " <<  "contact vel:" <<"   " << invdyn_->getFrameVeolocity(fid).head(3).transpose() << endl;

                Ni_ = oMi.rotation().transpose() * invdyn_->getSetting().get(InterfaceVectorParam_DEFAULT_CONTACT_NORMALS);   
                cout << "Contact Point in World Frame" << " " << oMi.act(Pi_).transpose() << endl;

                invdyn_->addUnilateralContactConstraint(constr, Pi_, Ni_);
            }
            return contact_changed;
        }

/////////////////////////////////////////////////////////////////////////////////////
        ControlVariable::ControlVariable(int size){
            q_.resize(size+7); v_.resize(size); 
            q_.setZero(); v_.setZero();
        }
        void ControlResult::resize(int duration, int size){
            control_result_.clear();
            for (int i=0; i<duration; i++)
                control_result_.push_back(ControlVariable(size));
       }
      
    }
}