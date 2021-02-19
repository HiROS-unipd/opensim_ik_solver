#include "opensim_ik_solver/Consumer.h"

hiros::opensim_ik::Consumer::Consumer(const OpenSim::Model& t_model,
                                      const double& t_accuracy,
                                      const SimTK::Rotation& t_sensor_to_opensim,
                                      const std::vector<std::string>& t_joint_names)
  : m_processed(nullptr)
  , m_rotation_table(nullptr)
  , m_joint_state(nullptr)
{
  m_joint_names = std::move(t_joint_names);
  m_rt_imu_ik_tool = std::make_unique<hiros::opensim_ik::RTIMUIKTool>(t_model, t_accuracy, t_sensor_to_opensim);
}

void hiros::opensim_ik::Consumer::runSingleFrameIK()
{
  queue.take_next_to_consume(m_rotation_table, m_joint_state, m_processed);
  runIK();
  queue.notify_output_ready(m_processed);
}

void hiros::opensim_ik::Consumer::runIK()
{
  m_rt_imu_ik_tool->runSingleFrameIK(*m_rotation_table);
  m_joint_state->header.stamp = ros::Time(m_rt_imu_ik_tool->getState().getTime());
  m_joint_state->name = m_joint_names;
  m_joint_state->position = m_rt_imu_ik_tool->getJointPositions(true);
}
