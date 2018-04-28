#include <iostream>

#include "hpp/pinocchio/urdf/util.hh"
#include "hpp/pinocchio/humanoid-robot.hh"
#include "hpp/pinocchio/center-of-mass-computation.hh"
#include "hpp/pinocchio/joint.hh"
#include "hpp/pinocchio/configuration.hh"
#include <hpp/pinocchio/device.hh>

using hpp::pinocchio::HumanoidRobot;
using hpp::pinocchio::HumanoidRobotPtr_t;
using namespace hpp::pinocchio::urdf;
using hpp::pinocchio::Configuration_t;
using hpp::pinocchio::CenterOfMassComputation;
using hpp::pinocchio::CenterOfMassComputationPtr_t;
using hpp::pinocchio::Device;
using hpp::pinocchio::DevicePtr_t;

#include "hpp/walkingctrl/interface/robot.hpp"

#include <boost/test/unit_test.hpp>

using namespace std;
using namespace hpp::walkingctrl;

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )
void test(){
  DevicePtr_t hyq = Device::create ("hyq");
  loadRobotModel(hyq, 0, "", "freeflyer", "hyq_description", "hyq", "", "");
 
  RobotKinDyn robotKinDyn_(hyq);
  vector_t q(17), v(16);
  q(2) = 1.0;
  
  robotKinDyn_.updateforwardKinematics(q, v);
  robotKinDyn_.updateFrameKinematics();
  int i = robotKinDyn_.getFrameId("lf_foot");
  std::cout << robotKinDyn_.getFrameTransformation(i) << std::endl;

  robotKinDyn_.computeAllTerms(q, v);
  std::cout << "com" << robotKinDyn_.getCOM() << std::endl;
  std::cout << "Jcom" << robotKinDyn_.getJCOM() << std::endl;
  std::cout << "mass" << robotKinDyn_.getMassMatrix() << std::endl;
  std::cout << "bias" << robotKinDyn_.getBiasTerm() << std::endl;
}

BOOST_AUTO_TEST_CASE ( build_model )
{
  test();
} 

BOOST_AUTO_TEST_SUITE_END()
