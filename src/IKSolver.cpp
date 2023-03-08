// Custom external packages dependencies
#include "skeletons/utils.h"

// Internal dependencies
#include "opensim_ik_solver/Consumer.h"
#include "opensim_ik_solver/IKSolver.h"
#include "opensim_ik_solver/Publisher.h"
#include "opensim_ik_solver/utils.h"

hiros::opensim_ik::IKSolver::IKSolver() : Node("hiros_opensim_ik_solver") {
  start();
}

hiros::opensim_ik::IKSolver::~IKSolver() { stop(); }

void hiros::opensim_ik::IKSolver::start() {
  configure();

  RCLCPP_INFO_STREAM(get_logger(),
                     BASH_MSG_GREEN << "Running" << BASH_MSG_RESET);
}

void hiros::opensim_ik::IKSolver::stop() const {
  RCLCPP_INFO_STREAM(get_logger(),
                     BASH_MSG_GREEN << "Stopped" << BASH_MSG_RESET);

  rclcpp::shutdown();
}

void hiros::opensim_ik::IKSolver::configure() {
  getParams();
  setupRosTopics();

  initializeIMUPlacer();
}

void hiros::opensim_ik::IKSolver::getParams() {
  // General Parameters
  getParam("n_threads", general_params_.n_threads);
  getParam("use_marker_positions", general_params_.use_marker_positions);
  getParam("use_link_orientations", general_params_.use_link_orientations);
  getParam("model_path", general_params_.model_path);
  getParam("model_calibration", general_params_.model_calibration);
  getParam("input_topic", general_params_.input_topic);
  getParam("out_joint_state_topic", general_params_.out_joint_state_topic);
  getParam("out_skeleton_group_topic",
           general_params_.out_skeleton_group_topic);

  if (!general_params_.use_marker_positions &&
      !general_params_.use_link_orientations) {
    RCLCPP_FATAL_STREAM(
        get_logger(),
        "Either 'use_marker_positions' or 'use_link_orientations' must be set "
        "to 'true'. Closing");
    stop();
    exit(EXIT_FAILURE);
  }

  if (general_params_.model_path.empty()) {
    RCLCPP_FATAL_STREAM(get_logger(),
                        "Non-empty 'model_path' is required. Closing");
    stop();
    exit(EXIT_FAILURE);
  }
  model_ = OpenSim::Model(general_params_.model_path);

  // IMU Placer Parameters
  if (general_params_.model_calibration) {
    if (!general_params_.use_link_orientations) {
      RCLCPP_WARN_STREAM(get_logger(),
                         "Model calibration requires 'use_link_orientations' "
                         "to be 'true'. Skipping");

      general_params_.model_calibration = false;
    } else {
      getParam("heading_correction", imu_placer_params_.heading_correction);
      getParam("base_imu_label", imu_placer_params_.base_imu_label);
      getParam("base_heading_axis", imu_placer_params_.base_heading_axis);
      getParam("use_marker_based_ik_as_initial_pose",
               imu_placer_params_.use_marker_based_ik_as_initial_pose);
      getParam("save_calibrated_model",
               imu_placer_params_.save_calibrated_model);
      getParam("visualize_calibration", imu_placer_params_.use_visualizer);

      if (imu_placer_params_.save_calibrated_model) {
        unsigned long ix = general_params_.model_path.rfind(".osim");
        std::string front = general_params_.model_path.substr(0, ix);
        std::string back = general_params_.model_path.substr(ix);
        imu_placer_params_.calibrated_model_path = front + "_calibrated" + back;
      }
    }
  }

  // IK Tools Parameters
  getParam("accuracy", ik_tool_params_.accuracy);
  getParam("visualize_ik", ik_tool_params_.use_visualizer);

  if (general_params_.use_marker_positions) {
    getParam("markers_weight", ik_tool_params_.markers_weight);
  }

  if (general_params_.use_link_orientations) {
    getParam("orientations_weight", ik_tool_params_.orientations_weight);
  }

  double sensor_to_opensim_x, sensor_to_opensim_y, sensor_to_opensim_z;
  getParam("sensor_to_opensim_rotation_x", sensor_to_opensim_x);
  getParam("sensor_to_opensim_rotation_y", sensor_to_opensim_y);
  getParam("sensor_to_opensim_rotation_z", sensor_to_opensim_z);
  ik_tool_params_.sensor_to_opensim =
      SimTK::Rotation(SimTK::BodyOrSpaceType::SpaceRotationSequence,
                      sensor_to_opensim_x, SimTK::XAxis, sensor_to_opensim_y,
                      SimTK::YAxis, sensor_to_opensim_z, SimTK::ZAxis);

  ik_tool_params_.use_marker_positions = general_params_.use_marker_positions;
  ik_tool_params_.use_link_orientations = general_params_.use_link_orientations;

  parseLinksConfig();
}

void hiros::opensim_ik::IKSolver::parseLinksConfig() {
  std::vector<std::string> links{};
  if (!getParam("links", links)) {
    return;
  }

  LinkInfo li{};
  for (const auto& link : links) {
    getParam(link + ".id", li.id);
    getParam(link + ".name", li.name);
    getParam(link + ".parent_joint_id", li.parent_joint_id);
    getParam(link + ".child_joint_id", li.child_joint_id);
    ik_tool_params_.links_info.push_back(li);
  }
}

void hiros::opensim_ik::IKSolver::setupRosTopics() {
  sub_ = create_subscription<hiros_skeleton_msgs::msg::SkeletonGroup>(
      general_params_.input_topic, 10,
      std::bind(&IKSolver::callback, this, std::placeholders::_1));
}

void hiros::opensim_ik::IKSolver::initializeIMUPlacer() {
  if (general_params_.model_calibration) {
    rt_imu_placer_ = std::make_unique<hiros::opensim_ik::RTIMUPlacer>(
        model_, imu_placer_params_);
  }
}

void hiros::opensim_ik::IKSolver::initializeThreads() {
  for (int i = 0; i < general_params_.n_threads; ++i) {
    std::thread consumer(&hiros::opensim_ik::IKSolver::startConsumer, this);
    consumer.detach();
  }

  std::thread publisher(&hiros::opensim_ik::IKSolver::startPublisher, this);
  publisher.detach();
}

bool hiros::opensim_ik::IKSolver::calibrateIMUs(
    const hiros_skeleton_msgs::msg::SkeletonGroup& msg) {
  if (general_params_.model_calibration) {
    auto quat_table =
        utils::toQuaternionsTable(msg, utils::getOrientationNames(model_),
                                  ik_tool_params_.sensor_to_opensim);

    if (utils::isNaN(quat_table)) {
      return false;
    }

    auto markers_ref =
        imu_placer_params_.use_marker_based_ik_as_initial_pose
            ? utils::toMarkersReference(msg, utils::getMarkerNames(model_),
                                        ik_tool_params_.sensor_to_opensim)
            : OpenSim::MarkersReference();
    rt_imu_placer_->runCalibration(quat_table, markers_ref);

    model_ = rt_imu_placer_->getCalibratedModel();
    model_.finalizeFromProperties();  // TODO: might be useless

    if (imu_placer_params_.save_calibrated_model) {
      rt_imu_placer_->saveModel();
    }

    RCLCPP_INFO_STREAM(get_logger(),
                       BASH_MSG_GREEN << "Calibrated" << BASH_MSG_RESET);
  }

  return true;
}

void hiros::opensim_ik::IKSolver::startConsumer() {
  Consumer c{queue_ptr_, model_, ik_tool_params_};

  while (rclcpp::ok()) {
    c.runSingleFrameIK();
  }
}

void hiros::opensim_ik::IKSolver::startPublisher() {
  Publisher p{queue_ptr_, shared_from_this(), general_params_};

  while (rclcpp::ok()) {
    p.publish();
  }
}

void hiros::opensim_ik::IKSolver::callback(
    const hiros_skeleton_msgs::msg::SkeletonGroup& msg) {
  if (!initialized_) {
    if (general_params_.model_calibration) {
      if (!calibrateIMUs(msg)) {
        return;
      }
      ik_tool_params_.heading_rot_vec = rt_imu_placer_->getHeadingRotVec();
    }

    initializeThreads();

    initialized_ = true;
  }

  queue_ptr_->push(msg);
}
