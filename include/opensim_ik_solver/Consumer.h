#ifndef hiros_opensim_ik_solver_Consumer_h
#define hiros_opensim_ik_solver_Consumer_h

// ROS dependencies
#include "sensor_msgs/msg/joint_state.hpp"

// OpenSim dependencies
#include "OpenSim/Simulation/InverseKinematicsSolver.h"

// Custom external dependencies
#include "hiros_skeleton_msgs/msg/skeleton_group.hpp"

// Internal dependencies
#include "opensim_ik_solver/PublisherData.h"
#include "opensim_ik_solver/Queue.h"
#include "opensim_ik_solver/RTIKTool.h"
#include "opensim_ik_solver/utils.h"

namespace hiros {
namespace opensim_ik {

class Consumer {
 public:
  Consumer(SkelGroupToPubDataQueuePtr queue_ptr, const OpenSim::Model& model,
           const IKToolParameters& params);

  void runSingleFrameIK();

 private:
  void runIK();

  void fillJointAngles();
  void fillSkeletonGroup();

  std::shared_ptr<hiros_skeleton_msgs::msg::SkeletonGroup> skeleton_group_{};
  std::shared_ptr<PublisherData> pub_data_{};
  std::shared_ptr<bool> processed_{};

  std::vector<std::string> marker_names_{};
  std::vector<std::string> orientation_names_{};

  IKToolParameters params_{};

  std::unique_ptr<hiros::opensim_ik::RTIKTool> rt_ik_tool_{};

  SkelGroupToPubDataQueuePtr queue_ptr_{};
};

}  // namespace opensim_ik
}  // namespace hiros

#endif
