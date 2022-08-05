// OpenSim dependencies
#include "OpenSim/Simulation/OpenSense/OpenSenseUtilities.h"

// Internal dependencies
#include "opensim_ik_solver/utils.h"

std::vector<std::string> hiros::opensim_ik::utils::getMarkerNames(const OpenSim::Model& t_model)
{
  OpenSim::Array<std::string> marker_names;

  t_model.getMarkerSet().getNames(marker_names);

  return utils::toStdVector(marker_names);
}

std::vector<std::string> hiros::opensim_ik::utils::getOrientationNames(OpenSim::Model t_model)
{
  std::vector<std::string> orientation_names;

  t_model.finalizeFromProperties();

  auto components = t_model.getComponentList<OpenSim::PhysicalOffsetFrame>();
  for (const auto& component : components) {
    auto component_name = component.getName();
    if (component_name.find("_imu") != std::string::npos) {
      orientation_names.push_back(component_name);
    }
  }

  return orientation_names;
}

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
        : SimTK::Vec3(std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::quiet_NaN());

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

  auto vec3table = toVec3Table(t_msg, t_marker_names, t_sensor_to_opensim);
  auto marker_weight_set = toMarkerWeightSet(t_msg, t_marker_names);

  double weights_sum = 0.;
  for (int i = 0; i < marker_weight_set.getSize(); ++i) {
    weights_sum += marker_weight_set[i].getWeight();
  }

  return (weights_sum == 0.) ? OpenSim::MarkersReference() : OpenSim::MarkersReference(vec3table, marker_weight_set);
}

OpenSim::TimeSeriesTable_<SimTK::Quaternion>
hiros::opensim_ik::utils::toQuaternionsTable(const hiros_skeleton_msgs::SkeletonGroup& t_msg,
                                             const std::vector<std::string> t_orientation_names,
                                             const SimTK::Rotation& t_sensor_to_opensim)
{
  // NB: currently support a single skeleton per skelton group. Only the first skeleton is processed.

  unsigned long n_cols = t_orientation_names.size();

  std::vector<double> time{t_msg.skeletons.front().src_time.toSec()};

  SimTK::Matrix_<SimTK::Quaternion> quaternion_matrix(1, static_cast<int>(n_cols), SimTK::Quaternion());
  std::vector<std::string> imu_labels;
  imu_labels.reserve(n_cols);

  int col = -1;

  for (const auto& name : t_orientation_names) {
    auto lk_it = std::find_if(t_msg.skeletons.front().links.begin(),
                              t_msg.skeletons.front().links.end(),
                              [&](const auto& lk) { return lk.name == name; });

    auto lk_or = (lk_it != t_msg.skeletons.front().links.end())
                   ? SimTK::Quaternion(lk_it->center.pose.orientation.w,
                                       lk_it->center.pose.orientation.x,
                                       lk_it->center.pose.orientation.y,
                                       lk_it->center.pose.orientation.z)
                   : SimTK::Quaternion(std::numeric_limits<double>::quiet_NaN(),
                                       std::numeric_limits<double>::quiet_NaN(),
                                       std::numeric_limits<double>::quiet_NaN(),
                                       std::numeric_limits<double>::quiet_NaN());

    quaternion_matrix.updElt(0, ++col) = lk_or;
    imu_labels.push_back(name);
  }

  auto quat_table = OpenSim::TimeSeriesTable_<SimTK::Quaternion>(time, quaternion_matrix, imu_labels);
  OpenSim::OpenSenseUtilities::rotateOrientationTable(quat_table, t_sensor_to_opensim);

  return quat_table;
}

OpenSim::TimeSeriesTable_<SimTK::Rotation>
hiros::opensim_ik::utils::toRotationsTable(const hiros_skeleton_msgs::SkeletonGroup& t_msg,
                                           const std::vector<std::string> t_orientation_names,
                                           const SimTK::Rotation& t_sensor_to_opensim)
{
  // NB: currently support a single skeleton per skelton group. Only the first skeleton is processed.

  unsigned long n_cols = t_orientation_names.size();

  std::vector<double> time{t_msg.skeletons.front().src_time.toSec()};

  SimTK::Matrix_<SimTK::Rotation> rotation_matrix(1, static_cast<int>(n_cols), SimTK::Rotation());
  std::vector<std::string> imu_labels;
  imu_labels.reserve(n_cols);

  int col = -1;

  for (const auto& name : t_orientation_names) {
    auto lk_it = std::find_if(t_msg.skeletons.front().links.begin(),
                              t_msg.skeletons.front().links.end(),
                              [&](const auto& lk) { return lk.name == name; });

    auto lk_or = (lk_it != t_msg.skeletons.front().links.end())
                   ? SimTK::Rotation(SimTK::Quaternion(lk_it->center.pose.orientation.w,
                                                       lk_it->center.pose.orientation.x,
                                                       lk_it->center.pose.orientation.y,
                                                       lk_it->center.pose.orientation.z))
                   : SimTK::Rotation(SimTK::Quaternion(std::numeric_limits<double>::quiet_NaN(),
                                                       std::numeric_limits<double>::quiet_NaN(),
                                                       std::numeric_limits<double>::quiet_NaN(),
                                                       std::numeric_limits<double>::quiet_NaN()));

    rotation_matrix.updElt(0, ++col) = t_sensor_to_opensim * lk_or;
    imu_labels.push_back(name);
  }

  return OpenSim::TimeSeriesTable_<SimTK::Rotation>(time, rotation_matrix, imu_labels);
}

OpenSim::Set<OpenSim::OrientationWeight>
hiros::opensim_ik::utils::toOrientationWeightSet(const hiros_skeleton_msgs::SkeletonGroup& t_msg,
                                                 const std::vector<std::string> t_orientation_names)
{
  // NB: currently support a single skeleton per skelton group. Only the first skeleton is processed.

  OpenSim::Set<OpenSim::OrientationWeight> weights;
  for (const auto& name : t_orientation_names) {
    auto lk_it = std::find_if(t_msg.skeletons.front().links.begin(),
                              t_msg.skeletons.front().links.end(),
                              [&](const auto& lk) { return lk.name == name; });

    double confidence = (lk_it != t_msg.skeletons.front().links.end()) ? lk_it->confidence : 0;
    weights.insert(weights.getSize(), OpenSim::OrientationWeight(name, confidence));
  }

  return weights;
}

OpenSim::OrientationsReference
hiros::opensim_ik::utils::toOrientationsReference(const hiros_skeleton_msgs::SkeletonGroup& t_msg,
                                                  const std::vector<std::string> t_orientation_names,
                                                  const SimTK::Rotation& t_sensor_to_opensim)
{
  // NB: currently support a single skeleton per skelton group. Only the first skeleton is processed.

  auto rot_table = toRotationsTable(t_msg, t_orientation_names, t_sensor_to_opensim);
  auto orientation_weight_set = toOrientationWeightSet(t_msg, t_orientation_names);

  double weights_sum = 0.;
  for (int i = 0; i < orientation_weight_set.getSize(); ++i) {
    weights_sum += orientation_weight_set[i].getWeight();
  }

  return (weights_sum == 0.) ? OpenSim::OrientationsReference()
                             : OpenSim::OrientationsReference(rot_table, &orientation_weight_set);
}

bool hiros::opensim_ik::utils::isNaN(const OpenSim::TimeSeriesTable_<SimTK::Vec3>& t_vec3_table)
{
  for (unsigned int row = 0; row < t_vec3_table.getNumRows(); ++row) {
    auto vec3_row = t_vec3_table.getRowAtIndex(row);

    for (const auto& vec3 : vec3_row) {
      if (vec3.isNaN()) {
        return true;
      }
    }
  }

  return false;
}

bool hiros::opensim_ik::utils::isNaN(const OpenSim::TimeSeriesTable_<SimTK::Quaternion>& t_quat_table)
{
  for (unsigned int row = 0; row < t_quat_table.getNumRows(); ++row) {
    auto quat_row = t_quat_table.getRowAtIndex(row);

    for (const auto& quat : quat_row) {
      if (quat.isNaN()) {
        return true;
      }
    }
  }

  return false;
}

bool hiros::opensim_ik::utils::isNaN(const OpenSim::TimeSeriesTable_<SimTK::Rotation>& t_rot_table)
{
  for (unsigned int row = 0; row < t_rot_table.getNumRows(); ++row) {
    auto rot_row = t_rot_table.getRowAtIndex(row);

    for (const auto& rot : rot_row) {
      if (rot.isNaN()) {
        return true;
      }
    }
  }

  return false;
}
