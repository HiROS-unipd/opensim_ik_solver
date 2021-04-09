#ifndef hiros_opensim_ik_solver_Publisher_h
#define hiros_opensim_ik_solver_Publisher_h

// ROS dependencies
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

// OpenSim dependencies
#include "OpenSim/Simulation/InverseKinematicsSolver.h"

// Internal dependencies
#include "Queue.h"

namespace hiros {
  namespace opensim_ik {

    typedef std::shared_ptr<Queue<OpenSim::OrientationsReference, sensor_msgs::JointState>> OrRefJointStateQueuePtr;

    class Publisher
    {
    public:
      Publisher(OrRefJointStateQueuePtr t_queue_ptr,
                const ros::NodeHandle& t_nh,
                const std::string& t_topic_name = "joint_state");

      void publish();

    private:
      ros::NodeHandle m_nh;
      ros::Publisher m_pub;

      OrRefJointStateQueuePtr m_queue_ptr;
    };

  } // namespace opensim_ik
} // namespace hiros

#endif
