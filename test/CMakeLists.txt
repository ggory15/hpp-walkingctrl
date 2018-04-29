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

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
INCLUDE_DIRECTORIES("${CDD_INCLUDE_DIR}")
#INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)

ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

# ADD_TESTCASE(NAME)
# ------------------------
#
# Define a test named `NAME'.
#
# This macro will create a binary from `NAME.cc', link it against
# Boost and add it to the test suite.
#

MACRO(ADD_TEST_CFLAGS target flag)
  SET_PROPERTY(TARGET ${target} APPEND_STRING PROPERTY COMPILE_FLAGS " ${flag}")
ENDMACRO(ADD_TEST_CFLAGS)

MACRO(ADD_TESTCASE NAME GENERATED)
  IF (${GENERATED} STREQUAL TRUE)
    ADD_EXECUTABLE(${NAME} ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.cc)
  ELSE()
    ADD_EXECUTABLE(${NAME} ${NAME}.cc)
  ENDIF()
  ADD_TEST(${NAME} ${RUNTIME_OUTPUT_DIRECTORY}/${NAME})
  
  # Link against Boost and project library.
  TARGET_LINK_LIBRARIES(${NAME}
    ${Boost_LIBRARIES} 
    ${PROJECT_NAME}     
    yaml-cpp
    qpOASES
    ${CDD_LIBRARIES}
    )
    PKG_CONFIG_USE_DEPENDENCY(${NAME} qpOASES)
    PKG_CONFIG_USE_DEPENDENCY(${NAME} hpp-core)
ENDMACRO(ADD_TESTCASE)

ADD_TESTCASE(testcontroller FALSE)
set_target_properties(testcontroller PROPERTIES COMPILE_DEFINITIONS TEST_PATH="${TEST_PATH}/")
#ADD_TESTCASE(cddtest FALSE)
#ADD_TESTCASE(robot_test FALSE)
#set_target_properties(robot_test PROPERTIES COMPILE_DEFINITIONS TEST_PATH="${TEST_PATH}/")
#ADD_TESTCASE(invdyn FALSE)
#set_target_properties(invdyn PROPERTIES COMPILE_DEFINITIONS TEST_PATH="${TEST_PATH}/")


#ADD_TESTCASE(QP_test FALSE)
#ADD_TESTCASE(yaml_reader FALSE)
#ADD_TESTCASE(invdyn FALSE)

#set_target_properties(yaml_reader PROPERTIES COMPILE_DEFINITIONS TEST_PATH="${TEST_PATH}/")
#set_target_properties(invdyn PROPERTIES COMPILE_DEFINITIONS TEST_PATH="${TEST_PATH}/")

