#include "opensim_ik_solver/Publisher.h"

hiros::opensim_ik::Publisher::Publisher(SkelGroupToPubDataQueuePtr t_queue_ptr,
                                        const ros::NodeHandle& t_nh,
                                        const GeneralParameters& t_params)
  : m_nh(t_nh)
  , m_queue_ptr(t_queue_ptr)
{
  m_js_pub = m_nh.advertise<sensor_msgs::JointState>(t_params.out_joint_state_topic, 1);
  m_sg_pub = m_nh.advertise<hiros_skeleton_msgs::SkeletonGroup>(t_params.out_skeleton_group_topic, 1);
}

void hiros::opensim_ik::Publisher::publish()
{
  const auto& pub_data = m_queue_ptr->pop();

  m_js_pub.publish(pub_data.joint_angles);
  m_sg_pub.publish(pub_data.skeleton_group);
}
