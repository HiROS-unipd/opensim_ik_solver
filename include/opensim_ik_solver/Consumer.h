#ifndef hiros_opensim_ik_solver_Consumer_h
#define hiros_opensim_ik_solver_Consumer_h

// ROS dependencies
#include "sensor_msgs/JointState.h"

// OpenSim dependencies
#include "OpenSim/Simulation/InverseKinematicsSolver.h"

// Custom external dependencies
#include "hiros_skeleton_msgs/SkeletonGroup.h"

// Internal dependencies
#include "opensim_ik_solver/Queue.h"
#include "opensim_ik_solver/RTIKTool.h"

namespace hiros {
  namespace opensim_ik {

    typedef std::shared_ptr<Queue<hiros_skeleton_msgs::SkeletonGroup, sensor_msgs::JointState>>
      SkelGroupJointStateQueuePtr;

    class Consumer
    {
    public:
      Consumer(SkelGroupJointStateQueuePtr t_queue_ptr,
               const OpenSim::Model& t_model,
               const double& t_accuracy = 1e-4,
               const SimTK::Rotation& t_sensor_to_opensim = SimTK::Rotation());

      void runSingleFrameIK();

    private:
      void runIK();

      std::shared_ptr<bool> m_processed;
      std::shared_ptr<hiros_skeleton_msgs::SkeletonGroup> m_skeleton_group;
      std::shared_ptr<sensor_msgs::JointState> m_joint_state;

      std::unique_ptr<hiros::opensim_ik::RTIKTool> m_rt_ik_tool;

      SkelGroupJointStateQueuePtr m_queue_ptr;
    };

  } // namespace opensim_ik
} // namespace hiros

#endif
