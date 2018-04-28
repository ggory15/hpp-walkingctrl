#pragma once
#include <hpp/walkingctrl/fwd.hh>

namespace hpp{
    namespace walkingctrl{
        namespace solvers{
        class StaggeredProjections{
            public:
                StaggeredProjections(const int & nv, const double & mu, const double& eps = 0.001, const int& verb = 0){};
                ~StaggeredProjections(){};

            private:

        };
    }
  }
}
