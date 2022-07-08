#ifndef hiros_opensim_ik_solver_IKSolver_h
#define hiros_opensim_ik_solver_IKSolver_h

// ROS dependencies
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

// Skeleton messages dependencies
#include "hiros_skeleton_msgs/SkeletonGroup.h"

// Internal dependencies
#include "opensim_ik_solver/PublisherData.h"
#include "opensim_ik_solver/Queue.h"
#include "opensim_ik_solver/RTIKTool.h"
#include "opensim_ik_solver/RTIMUPlacer.h"
#include "opensim_ik_solver/utils.h"

#define BASH_MSG_RESET "\033[0m"
#define BASH_MSG_GREEN "\033[32m"

namespace hiros {
  namespace opensim_ik {

    class IKSolver
    {
    public:
      IKSolver();
      virtual ~IKSolver() {}

      void start();
      void run();

    private:
      void getRosParams();
      void setupRos();
      void initializeIMUPlacer();
      void initializeThreads();
      void calibrateIMUs(const hiros_skeleton_msgs::SkeletonGroup& t_msg);

      void startConsumer();
      void startPublisher();

      void callback(const hiros_skeleton_msgs::SkeletonGroup& t_msg);

      GeneralParameters m_general_params;
      IMUPlacerParameters m_imu_placer_params;
      IKToolParameters m_ik_tool_params;

      OpenSim::Model m_model;
      std::unique_ptr<hiros::opensim_ik::RTIMUPlacer> m_rt_imu_placer;
      std::unique_ptr<hiros::opensim_ik::RTIKTool> m_rt_ik_tool;

      bool m_configured;

      ros::NodeHandle m_nh;
      std::string m_node_namespace;
      ros::Subscriber m_orientations_sub;
      bool m_initialized;

      SkelGroupToPubDataQueue m_queue;
    };

  } // namespace opensim_ik
} // namespace hiros

#endif
