// Internal dependencies
#include "opensim_ik_solver/IKSolver.h"

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

  m_nh.getParam("accuracy", m_imu_ik_tool_params.accuracy);
  m_nh.getParam("visualize_motion", m_imu_ik_tool_params.visualize_motion);
}

void hiros::opensim_ik::IKSolver::setupRos()
{
  ROS_DEBUG_STREAM("OpenSim IK Solver... Setting up ROS topics");

  m_orientations_sub = m_nh.subscribe(m_general_params.input_topic, 10, &IKSolver::orientationsCallback, this);
  while (m_orientations_sub.getNumPublishers() == 0 && !ros::isShuttingDown()) {
    ROS_WARN_STREAM_DELAYED_THROTTLE(10, "OpenSim IK Solver... No input messages on " << m_general_params.input_topic);
  }

  m_joint_states_pub = m_nh.advertise<sensor_msgs::JointState>("joint_states", 1);
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

void hiros::opensim_ik::IKSolver::calibrateIMUs(const hiros_xsens_mtw_wrapper::MIMUArray& t_msg)
{
  ROS_INFO_STREAM("OpenSim IK Solver... Placing IMUs on model");

  m_rt_imu_placer->runCalibration(toQuaternionsTable(t_msg));

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
  m_rt_imu_ik_tool = std::make_unique<hiros::opensim_ik::RTIMUIKTool>(
    m_model, m_imu_ik_tool_params.accuracy, m_general_params.sensor_to_opensim);

  if (m_imu_ik_tool_params.visualize_motion) {
    m_rt_imu_ik_tool->enableVisualizer();
  }
}

void hiros::opensim_ik::IKSolver::initializeJointStateNames()
{
  for (auto& coord : m_model.getComponentList<OpenSim::Coordinate>()) {
    m_joint_names.push_back(coord.getName());
  }
}

OpenSim::TimeSeriesTable_<SimTK::Quaternion>
hiros::opensim_ik::IKSolver::toQuaternionsTable(const hiros_xsens_mtw_wrapper::MIMUArray& t_msg)
{
  unsigned long n_cols = t_msg.mimus.size();

  std::vector<double> time{
    std::max_element(t_msg.mimus.begin(),
                     t_msg.mimus.end(),
                     [](const decltype(t_msg.mimus)::value_type& m1, const decltype(t_msg.mimus)::value_type& m2) {
                       return m1.imu.header.stamp.toSec() < m2.imu.header.stamp.toSec();
                     })
      ->imu.header.stamp.toSec()};

  SimTK::Matrix_<SimTK::Quaternion> quaternion_matrix(1, static_cast<int>(n_cols), SimTK::Quaternion());
  std::vector<std::string> imu_labels;
  imu_labels.reserve(n_cols);

  for (unsigned long col = 0; col < n_cols; ++col) {
    quaternion_matrix.updElt(0, static_cast<int>(col)) = SimTK::Quaternion(t_msg.mimus.at(col).imu.orientation.w,
                                                                           t_msg.mimus.at(col).imu.orientation.x,
                                                                           t_msg.mimus.at(col).imu.orientation.y,
                                                                           t_msg.mimus.at(col).imu.orientation.z);
    imu_labels.push_back(t_msg.mimus.at(col).imu.header.frame_id);
  }

  return OpenSim::TimeSeriesTable_<SimTK::Quaternion>(time, quaternion_matrix, imu_labels);
}

OpenSim::TimeSeriesTable_<SimTK::Rotation>
hiros::opensim_ik::IKSolver::toRotationsTable(const hiros_xsens_mtw_wrapper::MIMUArray& t_msg)
{
  unsigned long n_cols = t_msg.mimus.size();

  std::vector<double> time{
    std::max_element(t_msg.mimus.begin(),
                     t_msg.mimus.end(),
                     [](const decltype(t_msg.mimus)::value_type& m1, const decltype(t_msg.mimus)::value_type& m2) {
                       return m1.imu.header.stamp.toSec() < m2.imu.header.stamp.toSec();
                     })
      ->imu.header.stamp.toSec()};

  SimTK::Matrix_<SimTK::Rotation> rotation_matrix(1, static_cast<int>(n_cols), SimTK::Rotation());
  std::vector<std::string> imu_labels;
  imu_labels.reserve(n_cols);

  for (unsigned long col = 0; col < n_cols; ++col) {
    rotation_matrix.updElt(0, static_cast<int>(col)) =
      SimTK::Rotation(SimTK::Quaternion(t_msg.mimus.at(col).imu.orientation.w,
                                        t_msg.mimus.at(col).imu.orientation.x,
                                        t_msg.mimus.at(col).imu.orientation.y,
                                        t_msg.mimus.at(col).imu.orientation.z));
    imu_labels.push_back(t_msg.mimus.at(col).imu.header.frame_id);
  }

  return OpenSim::TimeSeriesTable_<SimTK::Rotation>(time, rotation_matrix, imu_labels);
}

sensor_msgs::JointState hiros::opensim_ik::IKSolver::getJointStateMsg()
{
  sensor_msgs::JointState out_msg;

  out_msg.header.stamp = ros::Time(m_rt_imu_ik_tool->getState().getTime());
  out_msg.name = m_joint_names;
  out_msg.position = m_rt_imu_ik_tool->getJointPositions();

  return out_msg;
}

void hiros::opensim_ik::IKSolver::orientationsCallback(const hiros_xsens_mtw_wrapper::MIMUArray& t_msg)
{
  if (!m_initialized) {
    if (m_imu_placer_params.perform_model_calibration) {
      calibrateIMUs(t_msg);
    }
    else {
      initializeModel();
    }

    initializeIKTool();
    initializeJointStateNames();

    m_initialized = true;
  }
  else {
    m_rt_imu_ik_tool->runSingleFrameIK(toRotationsTable(t_msg));
    m_joint_states_pub.publish(getJointStateMsg());
  }
}
