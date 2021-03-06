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

SET(LIBRARY_NAME ${PROJECT_NAME})

SET(${LIBRARY_NAME}_SOURCES
   #solver/StandardQpSolver.cpp
   #solver/solver-base.cpp
   #solver/basic-qpoase.cpp
   solver/abstractsolver.cpp

   math/constraint-base.cpp
   math/constraint-bound.cpp
   math/constraint-equality.cpp
   math/constraint-inequality.cpp
   math/utils.cpp

   utils/stop-watch.cpp
   utils/statistics.cpp
   utils/cdd_for_hpp.cpp

   interface/Setting.cpp

   invDynForm/invDynForm_util.cpp
   simulator/simulator.cpp

   controller/controller.cpp
  )

ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  ${${LIBRARY_NAME}_SOURCES}
  )


PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-core)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} qpOASES)

TARGET_LINK_LIBRARIES (${LIBRARY_NAME} ${Boost_LIBRARIES})

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)

