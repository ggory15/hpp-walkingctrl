#
# Copyright (c) 2015 CNRS
# Authors: Florent Lamiraux
#
#
# This file is part of hpp-walkgen
# hpp-walkgen is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-walkgen is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-walkgen  If not, see
# <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CXX_DISABLE_WERROR TRUE)
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/test.cmake)

ADD_REQUIRED_DEPENDENCY("eigen3 >= 3.0")
ADD_REQUIRED_DEPENDENCY("yaml-cpp >= 0.5")
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${YAML-CPP_INCLUDE_DIRS})

set(BUILD_DOCUMENTATION OFF)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -g -Wall -Wwrite-strings -pedantic -O3 -funroll-loops -fPIC")


SET(PROJECT_NAME hpp-walkingctrl)
SET(PROJECT_DESCRIPTION
  "Wholebody motion generation"
  )
SET(PROJECT_URL "")

SETUP_PROJECT()
SEARCH_FOR_BOOST()

# Activate hpp-util logging if requested
SET (HPP_DEBUG FALSE CACHE BOOL "trigger hpp-util debug output")
IF (HPP_DEBUG)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_DEBUG")
ENDIF()

SEARCH_FOR_BOOST ()

set(INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")

ADD_REQUIRED_DEPENDENCY("yaml-cpp >= 0.5")
ADD_REQUIRED_DEPENDENCY("hpp-core >= 4")
ADD_REQUIRED_DEPENDENCY("hpp-constraints >= 4")
ADD_REQUIRED_DEPENDENCY("hpp-statistics >= 1.1")
ADD_REQUIRED_DEPENDENCY("qpOASES")


SET(${PROJECT_NAME}HEADERS
  include/hpp/walkingctrl/fwd.hh

  include/hpp/walkingctrl/solver/StandardQpSolver.hh
  include/hpp/walkingctrl/solver/solver-base.hh
  include/hpp/walkingctrl/solver/solver-HQP-output.hpp
  include/hpp/walkingctrl/solver/basic-qpoase.hpp

  include/hpp/walkingctrl/math/constraint-base.hpp
  include/hpp/walkingctrl/math/constraint-bound.hpp
  include/hpp/walkingctrl/math/constraint-equality.hpp
  include/hpp/walkingctrl/math/constraint-inequality.hpp
  include/hpp/walkingctrl/math/fwd.hpp
  include/hpp/walkingctrl/math/utils.hpp

  include/hpp/walkingctrl/utils/stop-watch.hpp
  include/hpp/walkingctrl/utils/stastics.hpp
  include/hpp/walkingctrl/utils/Stdafx.hh

  include/hpp/walkingctrl/yaml_eigen.h
  include/hpp/walkingctrl/interface/Setting.hpp
  include/hpp/walkingctrl/interface/Definitions.hpp
  include/hpp/walkingctrl/interface/InterfaceParam.hpp

 # include/hpp/walkingctrl/invDynForm/fwd.hh
  include/hpp/walkingctrl/invDynForm/invDynForm_util.hh  

  include/hpp/walkingctrl/trajectory/trajectory.hh
  include/hpp/walkingcrl/task/task.hh

)



ADD_SUBDIRECTORY(src)



get_filename_component(TEST_PATH config ABSOLUTE)
add_subdirectory(test)


PKG_CONFIG_APPEND_LIBS("hpp-walkingctrl")

SETUP_PROJECT_FINALIZE()
