#ifndef hiros_opensim_ik_solver_IKSolver_h
#define hiros_opensim_ik_solver_IKSolver_h

// ROS dependencies
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

// Xsens MTw Wrapper dependencies
#include "hiros_xsens_mtw_wrapper/MIMUArray.h"

// Internal dependencies
#include "opensim_ik_solver/Queue.h"
#include "opensim_ik_solver/RTIMUIKTool.h"
#include "opensim_ik_solver/RTIMUPlacer.h"
#include "opensim_ik_solver/utils.h"

#define BASH_MSG_RESET "\033[0m"
#define BASH_MSG_GREEN "\033[32m"

namespace hiros {
  namespace opensim_ik {

    typedef Queue<OpenSim::TimeSeriesTable_<SimTK::Rotation>, sensor_msgs::JointState> SimTKRotJointStateQueue;
    typedef std::shared_ptr<SimTKRotJointStateQueue> SimTKRotJointStateQueuePtr;

    struct GeneralParameters
    {
      int n_threads;
      std::string input_topic;
      SimTK::Rotation sensor_to_opensim;
    };

    struct IMUPlacerParameters
    {
      std::string model_path;
      bool perform_model_calibration;
      bool perform_heading_correction;
      std::string base_imu_label;
      std::string base_heading_axis;
      bool save_calibrated_model;
      bool visualize_calibration;
    };

    struct IMUIKToolParameters
    {
      double accuracy;
      bool visualize_motion;
    };

    class IKSolver
    {
    public:
      IKSolver();
      virtual ~IKSolver() {}

      void start();
      void run();

    private:
      void getRosParams();
      void setupRos();
      void initializeIMUPlacer();
      void initializeModel();
      void initializeThreads();
      void calibrateIMUs(const hiros_xsens_mtw_wrapper::MIMUArray& t_msg);
      void initializeIKTool();
      void initializeJointStateNames();

      void startConsumer();
      void startPublisher();

      sensor_msgs::JointState getJointStateMsg();

      OpenSim::TimeSeriesTable_<SimTK::Quaternion> toQuaternionsTable(const hiros_xsens_mtw_wrapper::MIMUArray& t_msg);
      OpenSim::TimeSeriesTable_<SimTK::Rotation> toRotationsTable(const hiros_xsens_mtw_wrapper::MIMUArray& t_msg);

      void orientationsCallback(const hiros_xsens_mtw_wrapper::MIMUArray& t_msg);

      GeneralParameters m_general_params;
      IMUPlacerParameters m_imu_placer_params;
      IMUIKToolParameters m_imu_ik_tool_params;

      OpenSim::Model m_model;
      std::unique_ptr<hiros::opensim_ik::RTIMUPlacer> m_rt_imu_placer;
      std::unique_ptr<hiros::opensim_ik::RTIMUIKTool> m_rt_imu_ik_tool;

      bool m_configured;

      ros::NodeHandle m_nh;
      std::string m_node_namespace;

      std::vector<std::string> m_joint_names;

      ros::Subscriber m_orientations_sub;
      //      ros::Publisher m_joint_states_pub;

      bool m_initialized;

      SimTKRotJointStateQueue m_queue;
    };

  } // namespace opensim_ik
} // namespace hiros

#endif
