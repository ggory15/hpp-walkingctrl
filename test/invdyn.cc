//
// Copyright (c) 2015 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

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

#include "hpp/walkingctrl/invDynForm/invDynForm_util.hpp"
#include "hpp/walkingctrl/simulator/simulator.hpp"


#include <boost/test/unit_test.hpp>

using namespace hpp::walkingctrl;

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )
void test(){
  DevicePtr_t hyq = Device::create ("hyq");
  loadRobotModel(hyq, 0, "", "freeflyer", "hyq_description", "hyq", "", "");
  //setupHumanoidRobot (robot, "");

  std::string cfg_file = TEST_PATH + std::string("example_setting.yaml");

  InterfaceSetting interface_setting;
  interface_setting.initialize(cfg_file);

  InvDynForm invDynForm_(hyq);
  invDynForm_.createInvDynUtil(interface_setting);

  JointPostureTask posture_task("posture_traj", interface_setting.get(InterfaceVectorParam_q0).tail(12));
  posture_task.setgain(interface_setting.get(InterfaceDoubleParam_kp_posture), interface_setting.get(InterfaceDoubleParam_kd_posture));
  invDynForm_.addTask(posture_task, interface_setting.get(InterfaceDoubleParam_w_posture));

  vector3_t com_ref;
  com_ref = invDynForm_.getCOM();
  com_ref(0) -= 0.3;
  JointPostureTask com_task("com_traj", com_ref);
  com_task.setgain(interface_setting.get(InterfaceDoubleParam_kp_com), interface_setting.get(InterfaceDoubleParam_kd_com));
  invDynForm_.addTask(com_task, interface_setting.get(InterfaceDoubleParam_w_com));

  Simulator simulator_(hyq);
  simulator_.createSimulator(interface_setting);
}

BOOST_AUTO_TEST_CASE ( build_model )
{
  test();
} 

BOOST_AUTO_TEST_SUITE_END()
