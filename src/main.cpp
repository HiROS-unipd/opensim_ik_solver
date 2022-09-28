#include "opensim_ik_solver/IKSolver.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hiros::opensim_ik::IKSolver>());
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}
