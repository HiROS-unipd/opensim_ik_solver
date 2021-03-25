#ifndef hiros_opensim_ik_solver_Consumer_h
#define hiros_opensim_ik_solver_Consumer_h

// ROS dependencies
#include "sensor_msgs/JointState.h"

// OpenSim dependencies
#include "OpenSim/Simulation/InverseKinematicsSolver.h"

// Internal dependencies
#include "opensim_ik_solver/Queue.h"
#include "opensim_ik_solver/RTIMUIKTool.h"

extern hiros::opensim_ik::Queue<OpenSim::TimeSeriesTable_<SimTK::Rotation>, sensor_msgs::JointState> queue;

namespace hiros {
  namespace opensim_ik {

    class Consumer
    {
    public:
      Consumer(const OpenSim::Model& t_model,
               const double& t_accuracy = 1e-4,
               const SimTK::Rotation& t_sensor_to_opensim = SimTK::Rotation(),
               const std::vector<std::string>& t_joint_names = {});

      void runSingleFrameIK();

    private:
      void runIK();

      std::shared_ptr<bool> m_processed;
      std::shared_ptr<OpenSim::TimeSeriesTable_<SimTK::Rotation>> m_rotation_table;
      std::shared_ptr<sensor_msgs::JointState> m_joint_state;
      std::vector<std::string> m_joint_names;

      std::unique_ptr<hiros::opensim_ik::RTIMUIKTool> m_rt_imu_ik_tool;
    };

  } // namespace opensim_ik
} // namespace hiros

#endif
