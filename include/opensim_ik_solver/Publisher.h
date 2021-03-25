#ifndef hiros_opensim_ik_solver_Publisher_h
#define hiros_opensim_ik_solver_Publisher_h

// ROS dependencies
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

// OpenSim dependencies
#include "OpenSim/Simulation/InverseKinematicsSolver.h"

// Internal dependencies
#include "Queue.h"

extern hiros::opensim_ik::Queue<OpenSim::TimeSeriesTable_<SimTK::Rotation>, sensor_msgs::JointState> queue;

namespace hiros {
  namespace opensim_ik {

    class Publisher
    {
    public:
      Publisher(const ros::NodeHandle& t_nh, const std::string& t_topic_name = "joint_state");

      void publish();

    private:
      ros::NodeHandle m_nh;
      ros::Publisher m_pub;
    };

  } // namespace opensim_ik
} // namespace hiros

#endif
