#ifndef hiros_opensim_ik_solver_Publisher_h
#define hiros_opensim_ik_solver_Publisher_h

// ROS dependencies
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

// Internal dependencies
#include "opensim_ik_solver/PublisherData.h"
#include "opensim_ik_solver/Queue.h"

namespace hiros {
  namespace opensim_ik {

    class Publisher
    {
    public:
      Publisher(SkelGroupToPubDataQueuePtr t_queue_ptr,
                const ros::NodeHandle& t_nh,
                const std::string& t_joint_state_topic = "joint_state",
                const std::string& t_skeleton_group_topic = "skeleton_group");

      void publish();

    private:
      ros::NodeHandle m_nh;
      ros::Publisher m_js_pub;
      ros::Publisher m_sg_pub;

      SkelGroupToPubDataQueuePtr m_queue_ptr;
    };

  } // namespace opensim_ik
} // namespace hiros

#endif
