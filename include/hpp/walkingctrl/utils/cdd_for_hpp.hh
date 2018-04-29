#ifndef HPP_WALKINGCTRL_CDD_HH
#define HPP_WALKINGCTRL_CDD_HH

#include <hpp/walkingctrl/fwd.hh>
#include <iostream>
#include <fstream>

#include <Eigen/Dense>
#include <Eigen/src/Core/util/Macros.h>

#include "cdd/cddmp.h"
#include "cdd/setoper.h"
#include "cdd/cddtypes.h"
#include "cdd/cdd.h"

namespace hpp{
    namespace walkingctrl{
      dd_MatrixPtr cone_span_eigen_to_cdd(CRef_matrix_t input, const bool canonicalize=false);
      //Rotation crossMatrix(Cref_vector3 x);
      
      void init_cdd_library();

      void release_cdd_library();

      void uniform(CRef_matrix_t lower_bounds, CRef_matrix_t upper_bounds, Ref_matrix_t out);

      void compute_convex_hull(CRef_matrix_t input, matrix_t& B_sp, vector_t& b_sp);
  }
}




#endif
