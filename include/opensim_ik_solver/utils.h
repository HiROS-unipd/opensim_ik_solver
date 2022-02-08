#ifndef hiros_opensim_ik_solver_ultis_h
#define hiros_opensim_ik_solver_ultis_h

// OpenSim dependencies
#include "OpenSim/Common/TimeSeriesTable.h"
#include "OpenSim/Simulation/OrientationsReference.h"

// Custom external dependencies
#include "hiros_skeleton_msgs/SkeletonGroup.h"

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

      OpenSim::TimeSeriesTable_<SimTK::Quaternion> toQuaternionsTable(const hiros_skeleton_msgs::SkeletonGroup& t_msg);
      OpenSim::TimeSeriesTable_<SimTK::Rotation> toRotationsTable(const hiros_skeleton_msgs::SkeletonGroup& t_msg);

      OpenSim::Set<OpenSim::OrientationWeight> toOrientationWeightSet(const hiros_skeleton_msgs::SkeletonGroup& t_msg);
      OpenSim::OrientationsReference toOrientationsReference(const hiros_skeleton_msgs::SkeletonGroup& t_msg);

    } // namespace utils
  } // namespace opensim_ik
} // namespace hiros

#endif
