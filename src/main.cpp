// ROS dependencies
#include "ros/ros.h"

// Internal dependencies
#include "opensim_ik_solver/IKSolver.h"

int main(int argc, char* argv[])
{
  std::string node_name = "hiros_opensim_ik_solver";
  ros::init(argc, argv, node_name);

  hiros::opensim_ik::IKSolver ik_solver;
  ik_solver.start();
  ik_solver.run();

  exit(EXIT_SUCCESS);
}
