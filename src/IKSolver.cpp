// Custom external packages dependencies
#include "skeletons/utils.h"

// Internal dependencies
#include "opensim_ik_solver/Consumer.h"
#include "opensim_ik_solver/IKSolver.h"
#include "opensim_ik_solver/Publisher.h"
#include "opensim_ik_solver/utils.h"

hiros::opensim_ik::IKSolver::IKSolver()
  : m_configured(false)
  , m_nh("~")
  , m_node_namespace(m_nh.getNamespace())
  , m_initialized(false)
{}

void hiros::opensim_ik::IKSolver::start()
{
  ROS_INFO_STREAM("OpenSim IK Solver... Starting");

  getRosParams();
  setupRos();
  initializeIMUPlacer();
}

void hiros::opensim_ik::IKSolver::run()
{
  ROS_INFO_STREAM(BASH_MSG_GREEN << "OpenSim IK Solver... RUNNING" << BASH_MSG_RESET);

  ros::spin();
}

void hiros::opensim_ik::IKSolver::getRosParams()
{
  ROS_DEBUG_STREAM("OpenSim IK Solver... Configuring Solver");

  m_nh.getParam("n_threads", m_general_params.n_threads);
  m_nh.getParam("input_topic", m_general_params.input_topic);
  double sensor_to_opensim_x, sensor_to_opensim_y, sensor_to_opensim_z;
  m_nh.getParam("sensor_to_opensim_rotation_x", sensor_to_opensim_x);
  m_nh.getParam("sensor_to_opensim_rotation_y", sensor_to_opensim_y);
  m_nh.getParam("sensor_to_opensim_rotation_z", sensor_to_opensim_z);
  m_general_params.sensor_to_opensim = SimTK::Rotation(SimTK::BodyOrSpaceType::SpaceRotationSequence,
                                                       sensor_to_opensim_x,
                                                       SimTK::XAxis,
                                                       sensor_to_opensim_y,
                                                       SimTK::YAxis,
                                                       sensor_to_opensim_z,
                                                       SimTK::ZAxis);

  m_nh.getParam("model_path", m_imu_placer_params.model_path);
  m_nh.getParam("perform_model_calibration", m_imu_placer_params.perform_model_calibration);
  m_nh.getParam("perform_heading_correction", m_imu_placer_params.perform_heading_correction);
  m_nh.getParam("base_imu_label", m_imu_placer_params.base_imu_label);
  m_nh.getParam("base_heading_axis", m_imu_placer_params.base_heading_axis);
  m_nh.getParam("save_calibrated_model", m_imu_placer_params.save_calibrated_model);
  m_nh.getParam("visualize_calibration", m_imu_placer_params.visualize_calibration);

  m_nh.getParam("accuracy", m_ik_tool_params.accuracy);
  m_nh.getParam("visualize_motion", m_ik_tool_params.visualize_motion);
}

void hiros::opensim_ik::IKSolver::setupRos()
{
  ROS_DEBUG_STREAM("OpenSim IK Solver... Setting up ROS topics");

  m_orientations_sub = m_nh.subscribe(m_general_params.input_topic, 10, &IKSolver::orientationsCallback, this);
  while (m_orientations_sub.getNumPublishers() == 0 && !ros::isShuttingDown()) {
    ROS_WARN_STREAM_DELAYED_THROTTLE(10, "OpenSim IK Solver... No input messages on " << m_general_params.input_topic);
  }
}

void hiros::opensim_ik::IKSolver::initializeIMUPlacer()
{
  m_rt_imu_placer = std::make_unique<hiros::opensim_ik::RTIMUPlacer>(m_imu_placer_params.model_path,
                                                                     m_general_params.sensor_to_opensim);

  if (m_imu_placer_params.perform_heading_correction) {
    m_rt_imu_placer->performHeadingCorrection(m_imu_placer_params.base_imu_label,
                                              m_imu_placer_params.base_heading_axis);
  }

  if (m_imu_placer_params.visualize_calibration) {
    m_rt_imu_placer->enableVisualizer();
  }
}

void hiros::opensim_ik::IKSolver::initializeModel()
{
  m_model = OpenSim::Model(m_imu_placer_params.model_path);
  m_model.finalizeFromProperties();
}

void hiros::opensim_ik::IKSolver::initializeThreads()
{
  for (int i = 0; i < m_general_params.n_threads; ++i) {
    std::thread consumer(&hiros::opensim_ik::IKSolver::startConsumer, this);
    consumer.detach();
  }

  std::thread publisher(&hiros::opensim_ik::IKSolver::startPublisher, this);
  publisher.detach();
}

void hiros::opensim_ik::IKSolver::calibrateIMUs(const hiros_skeleton_msgs::SkeletonGroup& t_msg)
{
  ROS_INFO_STREAM("OpenSim IK Solver... Placing IMUs on model");

  m_rt_imu_placer->runCalibration(utils::toQuaternionsTable(t_msg));

  m_model = m_rt_imu_placer->getCalibratedModel();
  m_model.finalizeFromProperties();

  if (m_imu_placer_params.save_calibrated_model) {
    unsigned long ix = m_imu_placer_params.model_path.rfind(".osim");
    std::string front = m_imu_placer_params.model_path.substr(0, ix);
    std::string back = m_imu_placer_params.model_path.substr(ix);
    std::string calibrated_model_path = front + "_calibrated" + back;

    ROS_INFO_STREAM("OpenSim IK Solver... Saving calibrated model to " << calibrated_model_path);
    m_model.print(calibrated_model_path);
  }

  ROS_INFO_STREAM(BASH_MSG_GREEN << "OpenSim IK Solver... CALIBRATED" << BASH_MSG_RESET);
}

void hiros::opensim_ik::IKSolver::initializeIKTool()
{
  m_rt_ik_tool = std::make_unique<hiros::opensim_ik::RTIKTool>(
    m_model, m_ik_tool_params.accuracy, m_general_params.sensor_to_opensim);

  if (m_ik_tool_params.visualize_motion) {
    m_rt_ik_tool->enableVisualizer();
  }
}

void hiros::opensim_ik::IKSolver::startConsumer()
{
  Consumer c{
    SkelGroupJointStateQueuePtr(&m_queue), m_model, m_ik_tool_params.accuracy, m_general_params.sensor_to_opensim};

  while (ros::ok()) {
    c.runSingleFrameIK();
  }
}

void hiros::opensim_ik::IKSolver::startPublisher()
{
  Publisher p{SkelGroupJointStateQueuePtr(&m_queue), m_nh};

  while (ros::ok()) {
    p.publish();
  }
}

void hiros::opensim_ik::IKSolver::orientationsCallback(const hiros_skeleton_msgs::SkeletonGroup& t_msg)
{
  if (!m_initialized) {
    if (m_imu_placer_params.perform_model_calibration) {
      calibrateIMUs(t_msg);
    }
    else {
      initializeModel();
    }

    initializeIKTool();
    initializeThreads();

    m_initialized = true;
  }
  else {
    m_queue.push(t_msg);
  }
}
