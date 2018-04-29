//
// Copyright (c) 2015 CNRS
// Authors: Florent Lamiraux
//
// This file is part of hpp-walkgen
// hpp-walkgen is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-walkgen is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-walkgen  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_WALKINGCTRL_FWD_HH
#define HPP_WALKINGCTRL_FWD_HH

# include <vector>
# include <Eigen/Core>
# include <hpp/core/fwd.hh>
# include <hpp/walkingctrl/config.hh>

namespace hpp {
  namespace walkingctrl {

    typedef double value_type;
    typedef Eigen::Matrix <value_type, Eigen::Dynamic, 1> vector_t;
    typedef Eigen::Matrix<value_type, Eigen::Dynamic, Eigen::Dynamic> matrix_t;
    typedef matrix_t::Index size_type;
    typedef std::vector <value_type> Times_t;
    typedef Eigen::Matrix <value_type, 2, 1> vector2_t;
    typedef Eigen::Matrix <value_type, 1, 1> vector1_t;
    typedef Eigen::Matrix <value_type, 3, 1> vector3_t;
    typedef Eigen::Matrix <value_type, 6, 1> vector6_t;
    typedef Eigen::Matrix <value_type, 3, 3> matrix3_t;

    typedef hpp::core::Transform3f Transform3f;
    typedef hpp::core::Path Path;
    typedef hpp::core::PathPtr_t PathPtr_t;
    typedef hpp::core::PathVectorPtr_t PathVectorPtr_t;
    typedef hpp::core::PathVector PathVector;
    typedef hpp::core::ConfigurationOut_t ConfigurationOut_t;
    typedef hpp::core::Configuration_t Configuration_t;
    typedef hpp::core::ConstraintSetPtr_t ConstraintSetPtr_t;
    typedef hpp::core::DevicePtr_t DevicePtr_t;
    typedef Eigen::Quaternion<double> quaternion_t;

    typedef Eigen::Ref<matrix_t>    Ref_matrix_t;
    typedef const Eigen::Ref<const matrix_t>    & CRef_matrix_t;

  } // namespace walingctrl
} // namespace hpp

#endif // HPP_WALKINGCTRL_FWD_HH
