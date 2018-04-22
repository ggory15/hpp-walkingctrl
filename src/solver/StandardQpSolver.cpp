
#include <hpp/walkingctrl/solver/StandardQpSolver.hh>

namespace hpp{
    namespace timeopt {

      // ContactPlan
      void ContactPlanFromFile::optimize(const DynamicsState& ini_state)
      {
        // this function fills in the contact sequence by reading it from a file
        try
        {
          YAML::Node planner_cfg = YAML::LoadFile(this->getSetting().get(PlannerStringParam_ConfigFile));
          YAML::Node contact_vars = planner_cfg["contact_plan"];

          // Contact parameters
          readParameter(contact_vars, "num_contacts", this->endeffectorContacts());
          for (int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++) {
            this->contactSequence().endeffectorContacts(eff_id).clear();
            if (this->endeffectorContacts(eff_id)>0) {
              YAML::Node eff_params = contact_vars[("effcnt_" + Problem::idToEndeffectorString(eff_id)).c_str()];
              for (int cnt_id=0; cnt_id<this->endeffectorContacts(eff_id); cnt_id++) {
                Eigen::VectorXd v(10);
                readParameter(eff_params, "cnt"+std::to_string(cnt_id), v);
                this->contactSequence().endeffectorContacts(eff_id).push_back(ContactState());
                this->contactSequence().endeffectorContacts(eff_id)[cnt_id].contactActivationTime() = v[0];
                this->contactSequence().endeffectorContacts(eff_id)[cnt_id].contactDeactivationTime() = v[1];
                this->contactSequence().endeffectorContacts(eff_id)[cnt_id].contactPosition() = Eigen::Vector3d(v[2], v[3], v[4]);
                this->contactSequence().endeffectorContacts(eff_id)[cnt_id].contactType() = idToContactType(v(9));
                this->contactSequence().endeffectorContacts(eff_id)[cnt_id].contactOrientation() = Eigen::Quaternion<double>(v[5],v[6],v[7],v[8]);
              }
            }
          }
        }
        catch (std::runtime_error& e)
        {
          std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
        }
      }
    }
}
