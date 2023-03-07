#ifndef hiros_opensim_ik_solver_IKSolver_h
#define hiros_opensim_ik_solver_IKSolver_h

// ROS dependencies
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// Skeleton messages dependencies
#include "hiros_skeleton_msgs/msg/skeleton_group.hpp"

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

class IKSolver : public rclcpp::Node {
 public:
  IKSolver();
  ~IKSolver();

 private:
  template <typename T>
  bool getParam(const std::string& name, T& parameter) {
    declare_parameter<T>(name, parameter);
    return get_parameter(name, parameter);
  }

  void start();
  void stop() const;
  void configure();

  void getParams();
  void setupRosTopics();
  void initializeIMUPlacer();
  void initializeThreads();
  bool calibrateIMUs(const hiros_skeleton_msgs::msg::SkeletonGroup& msg);

  void startConsumer();
  void startPublisher();

  void callback(const hiros_skeleton_msgs::msg::SkeletonGroup& msg);

  GeneralParameters general_params_{};
  IMUPlacerParameters imu_placer_params_{};
  IKToolParameters ik_tool_params_{};

  OpenSim::Model model_{};
  std::unique_ptr<hiros::opensim_ik::RTIMUPlacer> rt_imu_placer_{};
  std::unique_ptr<hiros::opensim_ik::RTIKTool> rt_ik_tool_{};

  rclcpp::Subscription<hiros_skeleton_msgs::msg::SkeletonGroup>::SharedPtr
      sub_{};

  bool initialized_{false};

  SkelGroupToPubDataQueue queue_{};
};

}  // namespace opensim_ik
}  // namespace hiros

#endif
