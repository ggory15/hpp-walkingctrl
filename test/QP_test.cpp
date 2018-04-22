
#include <iostream>


#include <hpp/walkingctrl/math/utils.hpp>
#include <hpp/walkingctrl/solver/basic-qpoase.hpp>

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN

#define BOOST_TEST_MODULE qp_test

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

using namespace hpp::walkingctrl;
using namespace hpp::walkingctrl::math;
using namespace hpp::walkingctrl::solvers;
using namespace std;
using namespace Eigen;


BOOST_AUTO_TEST_SUITE (test_hpp_walkingctrl)

BOOST_AUTO_TEST_CASE (qp_test)
{
    const unsigned int n = 2;
    const unsigned int m = 1;
    const unsigned int neq = 0;
    const unsigned int nin = 0;
    const double damping = 1e-4;


    // QP Test //
    // Step 0. Qposes_basic
    Qposes_basic * QPsolver;

    // Step 0. Set Basic Value;
    Eigen::VectorXd q0(19); //7+12
    q0 << 0., 0., 0.63, 0., -0., 0., 1., -0.51, 0.74, -0.93, -0.18, -0.38, 1.35,-0.36, 0.89, -1.19, -0.3 , 0.19, 0.79;
    Eigen::VectorXd v0(19);
    v0.setZero();
    Eigen::VectorXd torque(12);
    torque.setZero();

    // Step 1. set SoftInequlaityIndexex





    //QPsolver->solve();
   // solver->solve();
}
BOOST_AUTO_TEST_SUITE_END()
