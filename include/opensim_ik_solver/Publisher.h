#ifndef hiros_opensim_ik_solver_Publisher_h
#define hiros_opensim_ik_solver_Publisher_h

// ROS dependencies
#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/joint_state.hpp"

// Internal dependencies
#include "opensim_ik_solver/PublisherData.h"
#include "opensim_ik_solver/Queue.h"
#include "opensim_ik_solver/utils.h"

namespace hiros {
namespace opensim_ik {

class Publisher {
 public:
  Publisher(SkelGroupToPubDataQueuePtr queue_ptr,
            std::shared_ptr<rclcpp::Node> node,
            const GeneralParameters& params);

  void publish();

 private:
  std::shared_ptr<rclcpp::Node> node_{};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_{};
  rclcpp::Publisher<hiros_skeleton_msgs::msg::SkeletonGroup>::SharedPtr
      sg_pub_{};

  SkelGroupToPubDataQueuePtr queue_ptr_{};
};

}  // namespace opensim_ik
}  // namespace hiros

#endif
