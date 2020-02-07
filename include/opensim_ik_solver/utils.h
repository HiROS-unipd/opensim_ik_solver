#ifndef hiros_opensim_ik_solver_ultis_h
#define hiros_opensim_ik_solver_ultis_h

// OpenSim dependencies
#include "SimTKcommon/basics.h"

namespace hiros {
  namespace opensim_ik {
    namespace utils {

      template <class T>
      std::vector<T> toStdVector(SimTK::Vector_<T> v)
      {
        std::vector<T> r;
        r.reserve(v.size());
        for (auto& e : v) {
          r.push_back(e);
        }
        return r;
      }

    } // namespace utils
  } // namespace opensim_ik
} // namespace hiros

#endif
