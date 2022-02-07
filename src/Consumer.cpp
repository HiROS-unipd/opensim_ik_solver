#include "opensim_ik_solver/Consumer.h"

hiros::opensim_ik::Consumer::Consumer(OrRefJointStateQueuePtr t_queue_ptr,
                                      const OpenSim::Model& t_model,
                                      const double& t_accuracy,
                                      const SimTK::Rotation& t_sensor_to_opensim)
  : m_processed(nullptr)
  , m_orientations_reference(nullptr)
  , m_joint_state(nullptr)
  , m_queue_ptr(t_queue_ptr)
{
  m_rt_ik_tool = std::make_unique<hiros::opensim_ik::RTIKTool>(t_model, t_accuracy, t_sensor_to_opensim);
}

void hiros::opensim_ik::Consumer::runSingleFrameIK()
{
  m_queue_ptr->takeNextToConsume(m_orientations_reference, m_joint_state, m_processed);
  runIK();
  m_queue_ptr->notifyOutputReady(m_processed);
}

void hiros::opensim_ik::Consumer::runIK()
{
  m_rt_ik_tool->runSingleFrameIK(*m_orientations_reference);
  m_joint_state->header.stamp = ros::Time(m_rt_ik_tool->getState().getTime());
  m_joint_state->name = m_rt_ik_tool->getJointAngleNames();
  m_joint_state->position = m_rt_ik_tool->getJointAnglePositions(true);
}
