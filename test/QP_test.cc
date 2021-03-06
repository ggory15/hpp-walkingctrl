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

#include "hpp/walkingctrl/solver/standardQPSolver.hh"
#include <boost/test/unit_test.hpp>

using namespace hpp::walkingctrl;

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )
void test(){
  solvers::StandardQpSolver(12, 0);
}

BOOST_AUTO_TEST_CASE ( build_model )
{
  test();
} 

BOOST_AUTO_TEST_SUITE_END()
