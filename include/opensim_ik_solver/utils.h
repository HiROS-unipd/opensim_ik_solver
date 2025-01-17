#ifndef hiros_opensim_ik_solver_ultis_h
#define hiros_opensim_ik_solver_ultis_h

// OpenSim dependencies
#include "OpenSim/Common/TimeSeriesTable.h"
#include "OpenSim/Simulation/MarkersReference.h"
#include "OpenSim/Simulation/Model/Model.h"
#include "OpenSim/Simulation/OrientationsReference.h"

// Custom external dependencies
#include "hiros_skeleton_msgs/msg/skeleton_group.hpp"

namespace hiros {
namespace opensim_ik {

struct LinkInfo {
  int id{-1};
  std::string name{};
  int parent_joint_id{-1};
  int child_joint_id{-1};
};

struct GeneralParameters {
  int n_threads{1};

  bool use_marker_positions{false};
  bool use_link_orientations{false};

  std::string model_path{};
  bool model_calibration{false};

  std::string input_topic{};
  std::string out_joint_state_topic{};
  std::string out_skeleton_group_topic{};
};

struct IMUPlacerParameters {
  bool heading_correction{false};

  std::string base_imu_label{};
  std::string base_heading_axis{};
  bool use_marker_based_ik_as_initial_pose{false};

  bool save_calibrated_model{false};
  std::string calibrated_model_path{};

  bool use_visualizer{false};
};

struct IKToolParameters {
  double accuracy{1.e-4};

  bool use_marker_positions{false};
  bool use_link_orientations{false};

  double markers_weight{1.};
  double orientations_weight{1.};

  SimTK::Rotation sensor_to_opensim{};
  SimTK::Vec3 heading_rot_vec{0, 0, 0};

  bool use_visualizer{false};

  std::vector<LinkInfo> links_info{};
};

namespace utils {

template <class T>
std::vector<T> toStdVector(SimTK::Vector_<T> v) {
  std::vector<T> r;
  r.reserve(v.size());
  for (auto& e : v) {
    r.push_back(e);
  }
  return r;
}

template <class T>
std::vector<T> toStdVector(OpenSim::Array<T> a) {
  std::vector<T> r;
  r.reserve(a.size());
  for (int i = 0; i < a.size(); ++i) {
    r.push_back(a[i]);
  }
  return r;
}

std::vector<std::string> getMarkerNames(const OpenSim::Model& model);
std::vector<std::string> getOrientationNames(OpenSim::Model model);

OpenSim::TimeSeriesTable_<SimTK::Vec3> toVec3Table(
    const hiros_skeleton_msgs::msg::SkeletonGroup& msg,
    const std::vector<std::string> marker_names,
    const SimTK::Rotation& sensor_to_opensim = SimTK::Rotation());

OpenSim::Set<OpenSim::MarkerWeight> toMarkerWeightSet(
    const hiros_skeleton_msgs::msg::SkeletonGroup& msg,
    const std::vector<std::string> marker_names);
OpenSim::MarkersReference toMarkersReference(
    const hiros_skeleton_msgs::msg::SkeletonGroup& msg,
    const std::vector<std::string> marker_names,
    const SimTK::Rotation& sensor_to_opensim = SimTK::Rotation());

OpenSim::TimeSeriesTable_<SimTK::Quaternion> toQuaternionsTable(
    const hiros_skeleton_msgs::msg::SkeletonGroup& msg,
    const std::vector<std::string> orientation_names,
    const SimTK::Rotation& sensor_to_opensim = SimTK::Rotation(),
    const SimTK::Vec3& heading_rot_vec = SimTK::Vec3(0, 0, 0));
OpenSim::TimeSeriesTable_<SimTK::Rotation> toRotationsTable(
    const hiros_skeleton_msgs::msg::SkeletonGroup& msg,
    const std::vector<std::string> orientation_names,
    const SimTK::Rotation& sensor_to_opensim = SimTK::Rotation(),
    const SimTK::Vec3& heading_rot_vec = SimTK::Vec3(0, 0, 0));

OpenSim::Set<OpenSim::OrientationWeight> toOrientationWeightSet(
    const hiros_skeleton_msgs::msg::SkeletonGroup& msg,
    const std::vector<std::string> orientation_names);
OpenSim::OrientationsReference toOrientationsReference(
    const hiros_skeleton_msgs::msg::SkeletonGroup& msg,
    const std::vector<std::string> orientation_names,
    const SimTK::Rotation& sensor_to_opensim = SimTK::Rotation(),
    const SimTK::Vec3& heading_rot_vec = SimTK::Vec3(0, 0, 0));

bool isNaN(const OpenSim::TimeSeriesTable_<SimTK::Vec3>& vec3_table);
bool isNaN(const OpenSim::TimeSeriesTable_<SimTK::Quaternion>& quat_table);
bool isNaN(const OpenSim::TimeSeriesTable_<SimTK::Rotation>& rot_table);

}  // namespace utils
}  // namespace opensim_ik
}  // namespace hiros

#endif
