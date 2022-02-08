#ifndef hiros_opensim_ik_solver_ultis_h
#define hiros_opensim_ik_solver_ultis_h

// OpenSim dependencies
#include "OpenSim/Common/TimeSeriesTable.h"
#include "OpenSim/Simulation/MarkersReference.h"
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

      template <class T>
      std::vector<T> toStdVector(OpenSim::Array<T> a)
      {
        std::vector<T> r;
        r.reserve(a.size());
        for (int i = 0; i < a.size(); ++i) {
          r.push_back(a[i]);
        }
        return r;
      }

      OpenSim::TimeSeriesTable_<SimTK::Vec3>
      toVec3Table(const hiros_skeleton_msgs::SkeletonGroup& t_msg,
                  const std::vector<std::string> t_marker_names,
                  const SimTK::Rotation& t_sensor_to_opensim = SimTK::Rotation());

      OpenSim::Set<OpenSim::MarkerWeight> toMarkerWeightSet(const hiros_skeleton_msgs::SkeletonGroup& t_msg,
                                                            const std::vector<std::string> t_marker_names);
      OpenSim::MarkersReference toMarkersReference(const hiros_skeleton_msgs::SkeletonGroup& t_msg,
                                                   const std::vector<std::string> t_marker_names,
                                                   const SimTK::Rotation& t_sensor_to_opensim = SimTK::Rotation());

      OpenSim::TimeSeriesTable_<SimTK::Quaternion>
      toQuaternionsTable(const hiros_skeleton_msgs::SkeletonGroup& t_msg,
                         const SimTK::Rotation& t_sensor_to_opensim = SimTK::Rotation());
      OpenSim::TimeSeriesTable_<SimTK::Rotation>
      toRotationsTable(const hiros_skeleton_msgs::SkeletonGroup& t_msg,
                       const SimTK::Rotation& t_sensor_to_opensim = SimTK::Rotation());

      OpenSim::Set<OpenSim::OrientationWeight> toOrientationWeightSet(const hiros_skeleton_msgs::SkeletonGroup& t_msg);
      OpenSim::OrientationsReference
      toOrientationsReference(const hiros_skeleton_msgs::SkeletonGroup& t_msg,
                              const SimTK::Rotation& t_sensor_to_opensim = SimTK::Rotation());

    } // namespace utils
  } // namespace opensim_ik
} // namespace hiros

#endif
