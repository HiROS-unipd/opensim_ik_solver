#include "opensim_ik_solver/Publisher.h"

hiros::opensim_ik::Publisher::Publisher(SkelGroupToPubDataQueuePtr queue_ptr,
                                        std::shared_ptr<rclcpp::Node> node,
                                        const GeneralParameters& params)
    : node_(node), queue_ptr_(queue_ptr) {
  js_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      params.out_joint_state_topic, 10);
  sg_pub_ = node_->create_publisher<hiros_skeleton_msgs::msg::SkeletonGroup>(
      params.out_skeleton_group_topic, 10);
}

void hiros::opensim_ik::Publisher::publish() {
  const auto& pub_data = queue_ptr_->pop();

  js_pub_->publish(pub_data.joint_angles);
  sg_pub_->publish(pub_data.skeleton_group);
}
