// Custom external dependencies
#include "skeletons/types.h"
#include "skeletons/utils.h"

// Internal dependencies
#include "opensim_ik_solver/Consumer.h"
#include "opensim_ik_solver/utils.h"

hiros::opensim_ik::Consumer::Consumer(SkelGroupToPubDataQueuePtr t_queue_ptr,
                                      const OpenSim::Model& t_model,
                                      const IKToolParameters& t_params)
  : m_skeleton_group(nullptr)
  , m_pub_data(nullptr)
  , m_processed(nullptr)
  , m_params(t_params)
  , m_queue_ptr(t_queue_ptr)
{
  // Initialize marker names
  m_marker_names = utils::getMarkerNames(t_model);

  // Initialize IMU names
  m_orientation_names = utils::getOrientationNames(t_model);

  // Initialize IK Tool
  m_rt_ik_tool = std::make_unique<hiros::opensim_ik::RTIKTool>(t_model, m_params);

}

void hiros::opensim_ik::Consumer::runSingleFrameIK()
{
  m_queue_ptr->takeNextToConsume(m_skeleton_group, m_pub_data, m_processed);
  runIK();
  m_queue_ptr->notifyOutputReady(m_processed);
}

void hiros::opensim_ik::Consumer::runIK()
{
  OpenSim::MarkersReference marker_refs;
  OpenSim::OrientationsReference orientation_refs;

  // TODO: currently only run IK on the first skeleton

  if (m_params.use_marker_positions && !m_skeleton_group->skeletons.front().markers.empty()) {
    marker_refs = utils::toMarkersReference(*m_skeleton_group, m_marker_names, m_params.sensor_to_opensim);
  }

  if (m_params.use_link_orientations && !m_skeleton_group->skeletons.front().links.empty()) {
    orientation_refs =
      utils::toOrientationsReference(*m_skeleton_group, m_orientation_names, m_params.sensor_to_opensim);
  }

  if (m_rt_ik_tool->runSingleFrameIK(marker_refs, orientation_refs)) {
    fillJointAngles();
    fillSkeletonGroup();
  }
}

void hiros::opensim_ik::Consumer::fillJointAngles()
{
  m_pub_data->joint_angles.header.stamp = ros::Time(m_rt_ik_tool->getState().getTime());
  m_pub_data->joint_angles.name = m_rt_ik_tool->getJointAngleNames();
  m_pub_data->joint_angles.position = m_rt_ik_tool->getJointAnglePositions(true);
}

void hiros::opensim_ik::Consumer::fillSkeletonGroup()
{
  auto names = m_rt_ik_tool->getMarkerNames();
  auto positions = m_rt_ik_tool->getMarkerPositions();
  auto velocities = m_rt_ik_tool->getMarkerVelocities();
  auto accelerations = m_rt_ik_tool->getMarkerAccelerations();

  m_pub_data->skeleton_group.header.stamp = ros::Time::now();
  m_pub_data->skeleton_group.header.frame_id = m_skeleton_group->header.frame_id;

  hiros::skeletons::types::Skeleton s(m_skeleton_group->skeletons.front().id,
                                      m_skeleton_group->skeletons.front().src_time.toSec(),
                                      m_skeleton_group->skeletons.front().src_frame,
                                      static_cast<unsigned int>(names.size()));

  for (unsigned int mk_idx = 0; mk_idx < names.size(); ++mk_idx) {
    const auto& name = names.at(mk_idx);

    hiros::skeletons::types::Pose p;
    hiros::skeletons::types::Velocity v;
    hiros::skeletons::types::Acceleration a;

    if (!positions.empty()) {
      const auto& pos = m_params.sensor_to_opensim.invert() * positions.at(mk_idx);
      p.position = {pos[0], pos[1], pos[2]};
    }

    if (!velocities.empty()) {
      const auto& vel = m_params.sensor_to_opensim.invert() * velocities.at(mk_idx);
      v.linear = {vel[0], vel[1], vel[2]};
    }

    if (!accelerations.empty()) {
      const auto& acc = m_params.sensor_to_opensim.invert() * accelerations.at(mk_idx);
      a.linear = {acc[0], acc[1], acc[2]};
    }

    s.addMarker({static_cast<int>(mk_idx), name, std::numeric_limits<double>::quiet_NaN(), {p, v, a}});
  }

  m_pub_data->skeleton_group.skeletons.push_back(hiros::skeletons::utils::toMsg(s));
}
