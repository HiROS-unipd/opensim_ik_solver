#ifndef hiros_opensim_ik_solver_Consumer_h
#define hiros_opensim_ik_solver_Consumer_h

// ROS dependencies
#include "sensor_msgs/JointState.h"

// OpenSim dependencies
#include "OpenSim/Simulation/InverseKinematicsSolver.h"

// Custom external dependencies
#include "hiros_skeleton_msgs/SkeletonGroup.h"

// Internal dependencies
#include "opensim_ik_solver/PublisherData.h"
#include "opensim_ik_solver/Queue.h"
#include "opensim_ik_solver/RTIKTool.h"

namespace hiros {
  namespace opensim_ik {

    class Consumer
    {
    public:
      Consumer(SkelGroupToPubDataQueuePtr t_queue_ptr,
               const OpenSim::Model& t_model,
               bool t_use_marker_positions,
               bool t_use_link_orientations,
               const double& t_accuracy = 1e-4,
               const SimTK::Rotation& t_sensor_to_opensim = SimTK::Rotation());

      void runSingleFrameIK();

      void fillJointAngles();
      void fillSkeletonGroup();

    private:
      void runIK();

      std::shared_ptr<bool> m_processed;
      std::shared_ptr<hiros_skeleton_msgs::SkeletonGroup> m_skeleton_group;
      std::shared_ptr<PublisherData> m_pub_data;

      std::unique_ptr<hiros::opensim_ik::RTIKTool> m_rt_ik_tool;

      bool m_use_marker_positions;
      bool m_use_link_orientations;
      SimTK::Rotation m_sensor_to_opensim;

      std::vector<std::string> m_marker_names;

      SkelGroupToPubDataQueuePtr m_queue_ptr;
    };

  } // namespace opensim_ik
} // namespace hiros

#endif
