#include "opensim_ik_solver/Publisher.h"

hiros::opensim_ik::Publisher::Publisher(SkelGroupJointStateQueuePtr t_queue_ptr,
                                        const ros::NodeHandle& t_nh,
                                        const std::string& t_topic_name)
  : m_nh(t_nh)
  , m_queue_ptr(t_queue_ptr)
{
  m_pub = m_nh.advertise<sensor_msgs::JointState>(t_topic_name, 1);
}

void hiros::opensim_ik::Publisher::publish()
{
  m_pub.publish(m_queue_ptr->pop());
}
