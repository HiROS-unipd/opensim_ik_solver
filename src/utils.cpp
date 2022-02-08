// OpenSim dependencies
#include "OpenSim/Simulation/OpenSense/OpenSenseUtilities.h"

// Internal dependencies
#include "opensim_ik_solver/utils.h"

OpenSim::TimeSeriesTable_<SimTK::Vec3>
hiros::opensim_ik::utils::toVec3Table(const hiros_skeleton_msgs::SkeletonGroup& t_msg,
                                      const std::vector<std::string> t_marker_names,
                                      const SimTK::Rotation& t_sensor_to_opensim)
{
  // NB: currently support a single skeleton per skelton group. Only the first skeleton is processed.

  unsigned long n_cols = t_marker_names.size();

  std::vector<double> time{t_msg.skeletons.front().src_time.toSec()};

  SimTK::Matrix_<SimTK::Vec3> vec3_matrix(1, static_cast<int>(n_cols), SimTK::Vec3());
  std::vector<std::string> marker_labels;
  marker_labels.reserve(n_cols);

  int col = -1;
  for (const auto& name : t_marker_names) {
    auto mk_it = std::find_if(t_msg.skeletons.front().markers.begin(),
                              t_msg.skeletons.front().markers.end(),
                              [&](const auto& mk) { return mk.name == name; });

    auto mk_pos =
      (mk_it != t_msg.skeletons.front().markers.end())
        ? t_sensor_to_opensim
            * SimTK::Vec3(mk_it->center.pose.position.x, mk_it->center.pose.position.y, mk_it->center.pose.position.z)
        : SimTK::Vec3();

    vec3_matrix.updElt(0, ++col) = mk_pos;
    marker_labels.push_back(name);
  }

  return OpenSim::TimeSeriesTable_<SimTK::Vec3>(time, vec3_matrix, marker_labels);
}

OpenSim::Set<OpenSim::MarkerWeight>
hiros::opensim_ik::utils::toMarkerWeightSet(const hiros_skeleton_msgs::SkeletonGroup& t_msg,
                                            const std::vector<std::string> t_marker_names)
{
  // NB: currently support a single skeleton per skelton group. Only the first skeleton is processed.

  OpenSim::Set<OpenSim::MarkerWeight> weights;
  for (const auto& name : t_marker_names) {
    auto mk_it = std::find_if(t_msg.skeletons.front().markers.begin(),
                              t_msg.skeletons.front().markers.end(),
                              [&](const auto& mk) { return mk.name == name; });

    double confidence = (mk_it != t_msg.skeletons.front().markers.end()) ? mk_it->confidence : 0;
    weights.insert(weights.getSize(), OpenSim::MarkerWeight(name, confidence));
  }

  return weights;
}

OpenSim::MarkersReference hiros::opensim_ik::utils::toMarkersReference(const hiros_skeleton_msgs::SkeletonGroup& t_msg,
                                                                       const std::vector<std::string> t_marker_names,
                                                                       const SimTK::Rotation& t_sensor_to_opensim)
{
  // NB: currently support a single skeleton per skelton group. Only the first skeleton is processed.

  return OpenSim::MarkersReference(toVec3Table(t_msg, t_marker_names, t_sensor_to_opensim),
                                   toMarkerWeightSet(t_msg, t_marker_names));
}

OpenSim::TimeSeriesTable_<SimTK::Quaternion>
hiros::opensim_ik::utils::toQuaternionsTable(const hiros_skeleton_msgs::SkeletonGroup& t_msg,
                                             const SimTK::Rotation& t_sensor_to_opensim)
{
  // NB: currently support a single skeleton per skelton group. Only the first skeleton is processed.

  unsigned long n_cols = t_msg.skeletons.front().links.size();

  std::vector<double> time{t_msg.skeletons.front().src_time.toSec()};

  SimTK::Matrix_<SimTK::Quaternion> quaternion_matrix(1, static_cast<int>(n_cols), SimTK::Quaternion());
  std::vector<std::string> imu_labels;
  imu_labels.reserve(n_cols);

  int col = -1;
  for (const auto& link : t_msg.skeletons.front().links) {
    quaternion_matrix.updElt(0, ++col) = SimTK::Quaternion(link.center.pose.orientation.w,
                                                           link.center.pose.orientation.x,
                                                           link.center.pose.orientation.y,
                                                           link.center.pose.orientation.z);
    imu_labels.push_back(link.name);
  }

  auto quat_table = OpenSim::TimeSeriesTable_<SimTK::Quaternion>(time, quaternion_matrix, imu_labels);
  OpenSim::OpenSenseUtilities::rotateOrientationTable(quat_table, t_sensor_to_opensim);

  return quat_table;
}

OpenSim::TimeSeriesTable_<SimTK::Rotation>
hiros::opensim_ik::utils::toRotationsTable(const hiros_skeleton_msgs::SkeletonGroup& t_msg,
                                           const SimTK::Rotation& t_sensor_to_opensim)
{
  // NB: currently support a single skeleton per skelton group. Only the first skeleton is processed.

  unsigned long n_cols = t_msg.skeletons.front().links.size();

  std::vector<double> time{t_msg.skeletons.front().src_time.toSec()};

  SimTK::Matrix_<SimTK::Rotation> rotation_matrix(1, static_cast<int>(n_cols), SimTK::Rotation());
  std::vector<std::string> imu_labels;
  imu_labels.reserve(n_cols);

  int col = -1;
  for (const auto& link : t_msg.skeletons.front().links) {
    rotation_matrix.updElt(0, ++col) = t_sensor_to_opensim
                                       * SimTK::Rotation(SimTK::Quaternion(link.center.pose.orientation.w,
                                                                           link.center.pose.orientation.x,
                                                                           link.center.pose.orientation.y,
                                                                           link.center.pose.orientation.z));
    imu_labels.push_back(link.name);
  }

  return OpenSim::TimeSeriesTable_<SimTK::Rotation>(time, rotation_matrix, imu_labels);
}

OpenSim::Set<OpenSim::OrientationWeight>
hiros::opensim_ik::utils::toOrientationWeightSet(const hiros_skeleton_msgs::SkeletonGroup& t_msg)
{
  // NB: currently support a single skeleton per skelton group. Only the first skeleton is processed.

  OpenSim::Set<OpenSim::OrientationWeight> weights;
  for (const auto& link : t_msg.skeletons.front().links) {
    weights.insert(weights.getSize(), OpenSim::OrientationWeight(link.name, link.confidence));
  }

  return weights;
}

OpenSim::OrientationsReference
hiros::opensim_ik::utils::toOrientationsReference(const hiros_skeleton_msgs::SkeletonGroup& t_msg,
                                                  const SimTK::Rotation& t_sensor_to_opensim)
{
  // NB: currently support a single skeleton per skelton group. Only the first skeleton is processed.

  auto weights = toOrientationWeightSet(t_msg);

  return OpenSim::OrientationsReference(toRotationsTable(t_msg, t_sensor_to_opensim), &weights);
}
