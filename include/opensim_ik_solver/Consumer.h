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
#include "opensim_ik_solver/utils.h"

namespace hiros {
  namespace opensim_ik {

    class Consumer
    {
    public:
      Consumer(SkelGroupToPubDataQueuePtr t_queue_ptr, const OpenSim::Model& t_model, const IKToolParameters& t_params);

      void runSingleFrameIK();

    private:
      void runIK();

      void fillJointAngles();
      void fillSkeletonGroup();

      std::shared_ptr<hiros_skeleton_msgs::SkeletonGroup> m_skeleton_group;
      std::shared_ptr<PublisherData> m_pub_data;
      std::shared_ptr<bool> m_processed;

      std::vector<std::string> m_marker_names;

      IKToolParameters m_params;

      std::unique_ptr<hiros::opensim_ik::RTIKTool> m_rt_ik_tool;

      SkelGroupToPubDataQueuePtr m_queue_ptr;
    };

  } // namespace opensim_ik
} // namespace hiros

#endif
