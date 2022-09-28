// Custom external dependencies
#include "skeletons/types.h"
#include "skeletons/utils.h"

// Internal dependencies
#include "opensim_ik_solver/Consumer.h"
#include "opensim_ik_solver/utils.h"

hiros::opensim_ik::Consumer::Consumer(SkelGroupToPubDataQueuePtr queue_ptr,
                                      const OpenSim::Model& model,
                                      const IKToolParameters& params)
    : params_(params), queue_ptr_(queue_ptr) {
  // Initialize marker and orientation names
  marker_names_ = utils::getMarkerNames(model);
  orientation_names_ = utils::getOrientationNames(model);

  // Initialize IK Tool
  rt_ik_tool_ = std::make_unique<hiros::opensim_ik::RTIKTool>(model, params_);
}

void hiros::opensim_ik::Consumer::runSingleFrameIK() {
  queue_ptr_->takeNextToConsume(skeleton_group_, pub_data_, processed_);
  runIK();
  queue_ptr_->notifyOutputReady(processed_);
}

void hiros::opensim_ik::Consumer::runIK() {
  OpenSim::MarkersReference marker_refs;
  OpenSim::OrientationsReference orientation_refs;

  // TODO: currently only run IK on the first skeleton

  if (params_.use_marker_positions &&
      !skeleton_group_->skeletons.front().markers.empty()) {
    marker_refs = utils::toMarkersReference(*skeleton_group_, marker_names_,
                                            params_.sensor_to_opensim);
  }

  if (params_.use_link_orientations &&
      !skeleton_group_->skeletons.front().links.empty()) {
    orientation_refs = utils::toOrientationsReference(
        *skeleton_group_, orientation_names_, params_.sensor_to_opensim,
        params_.heading_rot_vec);
  }

  if (rt_ik_tool_->runSingleFrameIK(marker_refs, orientation_refs)) {
    fillJointAngles();
    fillSkeletonGroup();
  }
}

void hiros::opensim_ik::Consumer::fillJointAngles() {
  pub_data_->joint_angles.header.stamp =
      rclcpp::Time(static_cast<long>(rt_ik_tool_->getState().getTime() * 1e9));
  pub_data_->joint_angles.name = rt_ik_tool_->getJointAngleNames();
  pub_data_->joint_angles.position = rt_ik_tool_->getJointAngleValues(true);
}

void hiros::opensim_ik::Consumer::fillSkeletonGroup() {
  auto names = rt_ik_tool_->getMarkerNames();
  auto positions = rt_ik_tool_->getMarkerPositions();
  auto velocities = rt_ik_tool_->getMarkerVelocities();
  auto accelerations = rt_ik_tool_->getMarkerAccelerations();

  pub_data_->skeleton_group.header.stamp =
      rclcpp::Time(std::chrono::system_clock::now()
                       .time_since_epoch()
                       .count());  // TODO: ROS2 check
  pub_data_->skeleton_group.header.frame_id = skeleton_group_->header.frame_id;

  hiros::skeletons::types::Skeleton s(
      skeleton_group_->skeletons.front().id +
          1000,  // TODO: just to have a different skeleton's color for
                 // debugging
      rclcpp::Time(skeleton_group_->skeletons.front().src_time).seconds(),
      skeleton_group_->skeletons.front().src_frame,
      static_cast<unsigned int>(names.size()));

  for (unsigned int mk_idx = 0; mk_idx < names.size(); ++mk_idx) {
    const auto& name = names.at(mk_idx);

    hiros::skeletons::types::Pose p;
    hiros::skeletons::types::Velocity v;
    hiros::skeletons::types::Acceleration a;

    if (!positions.empty()) {
      const auto& pos =
          params_.sensor_to_opensim.invert() * positions.at(mk_idx);
      p.position = {pos[0], pos[1], pos[2]};
    }

    if (!velocities.empty()) {
      const auto& vel =
          params_.sensor_to_opensim.invert() * velocities.at(mk_idx);
      v.linear = {vel[0], vel[1], vel[2]};
    }

    if (!accelerations.empty()) {
      const auto& acc =
          params_.sensor_to_opensim.invert() * accelerations.at(mk_idx);
      a.linear = {acc[0], acc[1], acc[2]};
    }

    s.addMarker({static_cast<int>(mk_idx),
                 name,
                 std::numeric_limits<double>::quiet_NaN(),
                 {p, v, a}});
  }

  int lk_id = -1;
  s.addLink({++lk_id, "", 0, 1});
  s.addLink({++lk_id, "", 2, 3});
  s.addLink({++lk_id, "", 3, 4});
  s.addLink({++lk_id, "", 4, 5});
  s.addLink({++lk_id, "", 5, 6});
  s.addLink({++lk_id, "", 6, 7});
  s.addLink({++lk_id, "", 7, 8});
  s.addLink({++lk_id, "", 9, 10});
  s.addLink({++lk_id, "", 10, 11});
  s.addLink({++lk_id, "", 11, 12});
  s.addLink({++lk_id, "", 12, 13});
  s.addLink({++lk_id, "", 13, 14});
  s.addLink({++lk_id, "", 14, 15});
  s.addLink({++lk_id, "", 1, 24});
  s.addLink({++lk_id, "", 24, 25});
  s.addLink({++lk_id, "", 24, 26});
  s.addLink({++lk_id, "", 24, 28});
  s.addLink({++lk_id, "", 26, 27});
  s.addLink({++lk_id, "", 28, 29});
  s.addLink({++lk_id, "", 0, 16});
  s.addLink({++lk_id, "", 16, 17});
  s.addLink({++lk_id, "", 17, 18});
  s.addLink({++lk_id, "", 18, 19});
  s.addLink({++lk_id, "", 0, 20});
  s.addLink({++lk_id, "", 20, 21});
  s.addLink({++lk_id, "", 21, 22});
  s.addLink({++lk_id, "", 22, 23});

  //  // Retain link parent and child markers information, just for
  //  visualization purposes for (const auto& lk :
  //  m_skeleton_group->skeletons.front().links) {
  //    if (s.hasMarker(lk.parent_marker) && s.hasMarker(lk.child_marker)) {
  //      s.addLink({lk.id, lk.name, lk.parent_marker, lk.child_marker});
  //    }
  //  }

  pub_data_->skeleton_group.skeletons.push_back(
      hiros::skeletons::utils::toMsg(s));
}
