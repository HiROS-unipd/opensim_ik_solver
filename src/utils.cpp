// OpenSim dependencies
#include "OpenSim/Simulation/OpenSense/OpenSenseUtilities.h"

// Internal dependencies
#include "opensim_ik_solver/utils.h"

OpenSim::TimeSeriesTable_<SimTK::Vec3>
hiros::opensim_ik::utils::toVec3Table(const hiros_skeleton_msgs::SkeletonGroup& t_msg,
                                      const SimTK::Rotation& t_sensor_to_opensim)
{
  // NB: currently support a single skeleton per skelton group. Only the first skeleton is processed.

  unsigned long n_cols = t_msg.skeletons.front().markers.size();

  std::vector<double> time{t_msg.skeletons.front().src_time.toSec()};

  SimTK::Matrix_<SimTK::Vec3> vec3_matrix(1, static_cast<int>(n_cols), SimTK::Vec3());
  std::vector<std::string> marker_labels;
  marker_labels.reserve(n_cols);

  int col = -1;
  for (const auto& marker : t_msg.skeletons.front().markers) {
    vec3_matrix.updElt(0, ++col) =
      t_sensor_to_opensim
      * SimTK::Vec3(marker.center.pose.position.x, marker.center.pose.position.y, marker.center.pose.position.z);

    marker_labels.push_back(marker.name);
  }

  return OpenSim::TimeSeriesTable_<SimTK::Vec3>(time, vec3_matrix, marker_labels);
}

OpenSim::Set<OpenSim::MarkerWeight>
hiros::opensim_ik::utils::toMarkerWeightSet(const hiros_skeleton_msgs::SkeletonGroup& t_msg)
{
  // NB: currently support a single skeleton per skelton group. Only the first skeleton is processed.

  OpenSim::Set<OpenSim::MarkerWeight> weights;
  for (const auto& marker : t_msg.skeletons.front().markers) {
    weights.insert(weights.getSize(), OpenSim::MarkerWeight(marker.name, marker.confidence));
  }

  return weights;
}

OpenSim::MarkersReference hiros::opensim_ik::utils::toMarkersReference(const hiros_skeleton_msgs::SkeletonGroup& t_msg,
                                                                       const SimTK::Rotation& t_sensor_to_opensim)
{
  // NB: currently support a single skeleton per skelton group. Only the first skeleton is processed.

  return OpenSim::MarkersReference(toVec3Table(t_msg, t_sensor_to_opensim), toMarkerWeightSet(t_msg));
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
