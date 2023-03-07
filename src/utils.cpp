// ROS dependencies
#include <rclcpp/rclcpp.hpp>

// OpenSim dependencies
#include "OpenSim/Simulation/OpenSense/OpenSenseUtilities.h"

// Internal dependencies
#include "opensim_ik_solver/utils.h"

std::vector<std::string> hiros::opensim_ik::utils::getMarkerNames(
    const OpenSim::Model& model) {
  OpenSim::Array<std::string> marker_names;

  model.getMarkerSet().getNames(marker_names);

  return utils::toStdVector(marker_names);
}

std::vector<std::string> hiros::opensim_ik::utils::getOrientationNames(
    OpenSim::Model model) {
  std::vector<std::string> orientation_names;

  model.finalizeFromProperties();

  auto components = model.getComponentList<OpenSim::PhysicalOffsetFrame>();
  for (const auto& component : components) {
    auto component_name = component.getName();
    if (component_name.find("_imu") != std::string::npos) {
      orientation_names.push_back(component_name);
    }
  }

  return orientation_names;
}

OpenSim::TimeSeriesTable_<SimTK::Vec3> hiros::opensim_ik::utils::toVec3Table(
    const hiros_skeleton_msgs::msg::SkeletonGroup& msg,
    const std::vector<std::string> marker_names,
    const SimTK::Rotation& sensor_to_opensim) {
  // NB: currently support a single skeleton per skelton group. Only the first
  // skeleton is processed.

  unsigned long n_cols = marker_names.size();

  std::vector<double> time{
      rclcpp::Time{msg.skeletons.front().src_time}.seconds()};

  SimTK::Matrix_<SimTK::Vec3> vec3_matrix(1, static_cast<int>(n_cols),
                                          SimTK::Vec3());
  std::vector<std::string> marker_labels;
  marker_labels.reserve(n_cols);

  int col = -1;

  for (const auto& name : marker_names) {
    auto mk_it = std::find_if(msg.skeletons.front().markers.begin(),
                              msg.skeletons.front().markers.end(),
                              [&](const auto& mk) { return mk.name == name; });

    auto mk_pos =
        (mk_it != msg.skeletons.front().markers.end())
            ? sensor_to_opensim * SimTK::Vec3(mk_it->center.pose.position.x,
                                              mk_it->center.pose.position.y,
                                              mk_it->center.pose.position.z)
            : SimTK::Vec3(std::numeric_limits<double>::quiet_NaN(),
                          std::numeric_limits<double>::quiet_NaN(),
                          std::numeric_limits<double>::quiet_NaN());

    vec3_matrix.updElt(0, ++col) = mk_pos;
    marker_labels.push_back(name);
  }

  return OpenSim::TimeSeriesTable_<SimTK::Vec3>(time, vec3_matrix,
                                                marker_labels);
}

OpenSim::Set<OpenSim::MarkerWeight> hiros::opensim_ik::utils::toMarkerWeightSet(
    const hiros_skeleton_msgs::msg::SkeletonGroup& msg,
    const std::vector<std::string> marker_names) {
  // NB: currently support a single skeleton per skelton group. Only the first
  // skeleton is processed.

  OpenSim::Set<OpenSim::MarkerWeight> weights;
  for (const auto& name : marker_names) {
    auto mk_it = std::find_if(msg.skeletons.front().markers.begin(),
                              msg.skeletons.front().markers.end(),
                              [&](const auto& mk) { return mk.name == name; });

    double confidence =
        (mk_it != msg.skeletons.front().markers.end()) ? mk_it->confidence : 0;
    weights.insert(weights.getSize(), OpenSim::MarkerWeight(name, confidence));
  }

  return weights;
}

OpenSim::MarkersReference hiros::opensim_ik::utils::toMarkersReference(
    const hiros_skeleton_msgs::msg::SkeletonGroup& msg,
    const std::vector<std::string> marker_names,
    const SimTK::Rotation& sensor_to_opensim) {
  // NB: currently support a single skeleton per skelton group. Only the first
  // skeleton is processed.

  auto vec3table = toVec3Table(msg, marker_names, sensor_to_opensim);
  auto marker_weight_set = toMarkerWeightSet(msg, marker_names);

  double weights_sum = 0.;
  for (int i = 0; i < marker_weight_set.getSize(); ++i) {
    weights_sum += marker_weight_set[i].getWeight();
  }

  return (weights_sum == 0.)
             ? OpenSim::MarkersReference()
             : OpenSim::MarkersReference(vec3table, marker_weight_set);
}

OpenSim::TimeSeriesTable_<SimTK::Quaternion>
hiros::opensim_ik::utils::toQuaternionsTable(
    const hiros_skeleton_msgs::msg::SkeletonGroup& msg,
    const std::vector<std::string> orientation_names,
    const SimTK::Rotation& sensor_to_opensim,
    const SimTK::Vec3& heading_rot_vec) {
  // NB: currently support a single skeleton per skelton group. Only the first
  // skeleton is processed.

  unsigned long n_cols = orientation_names.size();

  std::vector<double> time{
      rclcpp::Time{msg.skeletons.front().src_time}.seconds()};

  SimTK::Matrix_<SimTK::Quaternion> quaternion_matrix(
      1, static_cast<int>(n_cols), SimTK::Quaternion());
  std::vector<std::string> imu_labels;
  imu_labels.reserve(n_cols);

  int col = -1;

  for (const auto& name : orientation_names) {
    auto lk_it = std::find_if(msg.skeletons.front().links.begin(),
                              msg.skeletons.front().links.end(),
                              [&](const auto& lk) { return lk.name == name; });

    auto lk_or =
        (lk_it != msg.skeletons.front().links.end())
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

  auto quat_table = OpenSim::TimeSeriesTable_<SimTK::Quaternion>(
      time, quaternion_matrix, imu_labels);
  OpenSim::OpenSenseUtilities::rotateOrientationTable(quat_table,
                                                      sensor_to_opensim);

  SimTK::Rotation heading_rotation(
      SimTK::BodyOrSpaceType::SpaceRotationSequence, heading_rot_vec[0],
      SimTK::XAxis, heading_rot_vec[1], SimTK::YAxis, heading_rot_vec[2],
      SimTK::ZAxis);
  OpenSim::OpenSenseUtilities::rotateOrientationTable(quat_table,
                                                      heading_rotation);

  return quat_table;
}

OpenSim::TimeSeriesTable_<SimTK::Rotation>
hiros::opensim_ik::utils::toRotationsTable(
    const hiros_skeleton_msgs::msg::SkeletonGroup& msg,
    const std::vector<std::string> orientation_names,
    const SimTK::Rotation& sensor_to_opensim,
    const SimTK::Vec3& heading_rot_vec) {
  // NB: currently support a single skeleton per skelton group. Only the first
  // skeleton is processed.

  unsigned long n_cols = orientation_names.size();

  std::vector<double> time{
      rclcpp::Time{msg.skeletons.front().src_time}.seconds()};

  SimTK::Rotation heading_rotation(
      SimTK::BodyOrSpaceType::SpaceRotationSequence, heading_rot_vec[0],
      SimTK::XAxis, heading_rot_vec[1], SimTK::YAxis, heading_rot_vec[2],
      SimTK::ZAxis);

  SimTK::Matrix_<SimTK::Rotation> rotation_matrix(1, static_cast<int>(n_cols),
                                                  SimTK::Rotation());
  std::vector<std::string> imu_labels;
  imu_labels.reserve(n_cols);

  int col = -1;

  for (const auto& name : orientation_names) {
    auto lk_it = std::find_if(msg.skeletons.front().links.begin(),
                              msg.skeletons.front().links.end(),
                              [&](const auto& lk) { return lk.name == name; });

    auto lk_or = (lk_it != msg.skeletons.front().links.end())
                     ? SimTK::Rotation(
                           SimTK::Quaternion(lk_it->center.pose.orientation.w,
                                             lk_it->center.pose.orientation.x,
                                             lk_it->center.pose.orientation.y,
                                             lk_it->center.pose.orientation.z))
                     : SimTK::Rotation(SimTK::Quaternion(
                           std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::quiet_NaN()));

    rotation_matrix.updElt(0, ++col) =
        heading_rotation * sensor_to_opensim * lk_or;
    imu_labels.push_back(name);
  }

  return OpenSim::TimeSeriesTable_<SimTK::Rotation>(time, rotation_matrix,
                                                    imu_labels);
}

OpenSim::Set<OpenSim::OrientationWeight>
hiros::opensim_ik::utils::toOrientationWeightSet(
    const hiros_skeleton_msgs::msg::SkeletonGroup& msg,
    const std::vector<std::string> orientation_names) {
  // NB: currently support a single skeleton per skelton group. Only the first
  // skeleton is processed.

  OpenSim::Set<OpenSim::OrientationWeight> weights;
  for (const auto& name : orientation_names) {
    auto lk_it = std::find_if(msg.skeletons.front().links.begin(),
                              msg.skeletons.front().links.end(),
                              [&](const auto& lk) { return lk.name == name; });

    double confidence =
        (lk_it != msg.skeletons.front().links.end()) ? lk_it->confidence : 0;
    weights.insert(weights.getSize(),
                   OpenSim::OrientationWeight(name, confidence));
  }

  return weights;
}

OpenSim::OrientationsReference
hiros::opensim_ik::utils::toOrientationsReference(
    const hiros_skeleton_msgs::msg::SkeletonGroup& msg,
    const std::vector<std::string> orientation_names,
    const SimTK::Rotation& sensor_to_opensim,
    const SimTK::Vec3& heading_rot_vec) {
  // NB: currently support a single skeleton per skelton group. Only the first
  // skeleton is processed.

  auto rot_table = toRotationsTable(msg, orientation_names, sensor_to_opensim,
                                    heading_rot_vec);
  auto orientation_weight_set = toOrientationWeightSet(msg, orientation_names);

  double weights_sum = 0.;
  for (int i = 0; i < orientation_weight_set.getSize(); ++i) {
    weights_sum += orientation_weight_set[i].getWeight();
  }

  return (weights_sum == 0.) ? OpenSim::OrientationsReference()
                             : OpenSim::OrientationsReference(
                                   rot_table, &orientation_weight_set);
}

bool hiros::opensim_ik::utils::isNaN(
    const OpenSim::TimeSeriesTable_<SimTK::Vec3>& vec3_table) {
  for (unsigned int row = 0; row < vec3_table.getNumRows(); ++row) {
    auto vec3_row = vec3_table.getRowAtIndex(row);

    for (const auto& vec3 : vec3_row) {
      if (!vec3.isNaN()) {
        return false;
      }
    }
  }

  return true;
}

bool hiros::opensim_ik::utils::isNaN(
    const OpenSim::TimeSeriesTable_<SimTK::Quaternion>& quat_table) {
  for (unsigned int row = 0; row < quat_table.getNumRows(); ++row) {
    auto quat_row = quat_table.getRowAtIndex(row);

    for (const auto& quat : quat_row) {
      if (!quat.isNaN()) {
        return false;
      }
    }
  }

  return true;
}

bool hiros::opensim_ik::utils::isNaN(
    const OpenSim::TimeSeriesTable_<SimTK::Rotation>& rot_table) {
  for (unsigned int row = 0; row < rot_table.getNumRows(); ++row) {
    auto rot_row = rot_table.getRowAtIndex(row);

    for (const auto& rot : rot_row) {
      if (!rot.isNaN()) {
        return false;
      }
    }
  }

  return true;
}
