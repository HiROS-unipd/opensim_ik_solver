// Internal dependencies
#include "opensim_ik_solver/RTIKTool.h"

hiros::opensim_ik::RTIKTool::RTIKTool(const IKToolParameters& params)
    : params_(params) {
  params_.use_visualizer = false;  // TODO: currently not supported
}

hiros::opensim_ik::RTIKTool::RTIKTool(const OpenSim::Model& model,
                                      const IKToolParameters& params)
    : RTIKTool(params) {
  setModel(model);
}

hiros::opensim_ik::RTIKTool::~RTIKTool() {}

void hiros::opensim_ik::RTIKTool::setModel(const OpenSim::Model& model) {
  model_ = std::make_unique<OpenSim::Model>(model);
  model_->finalizeFromProperties();
  initialized_ = false;
}

void hiros::opensim_ik::RTIKTool::enableVisualizer() {
  params_.use_visualizer = true;
  model_->setUseVisualizer(params_.use_visualizer);
}

void hiros::opensim_ik::RTIKTool::disableVisualizer() {
  params_.use_visualizer = false;
  model_->setUseVisualizer(params_.use_visualizer);
}

bool hiros::opensim_ik::RTIKTool::runSingleFrameIK(
    const OpenSim::MarkersReference& marker_refs,
    const OpenSim::OrientationsReference& orientation_refs) {
  updateReference(marker_refs, orientation_refs);
  return runSingleFrameIK();
}

bool hiros::opensim_ik::RTIKTool::runSingleFrameIK(
    const OpenSim::OrientationsReference& orientation_refs) {
  updateReference(orientation_refs);
  return runSingleFrameIK();
}

void hiros::opensim_ik::RTIKTool::updateReference(
    const OpenSim::MarkersReference& marker_refs,
    const OpenSim::OrientationsReference& orientation_refs) {
  marker_refs_ = std::make_shared<OpenSim::MarkersReference>(marker_refs);
  orientation_refs_ =
      std::make_shared<OpenSim::OrientationsReference>(orientation_refs);
}

void hiros::opensim_ik::RTIKTool::updateReference(
    const OpenSim::OrientationsReference& orientation_refs) {
  orientation_refs_ =
      std::make_shared<OpenSim::OrientationsReference>(orientation_refs);
}

bool hiros::opensim_ik::RTIKTool::runSingleFrameIK() {
  bool ik_failed = true;

  if (params_.use_marker_positions) {
    if (marker_refs_ && marker_refs_->getNumFrames() == 1) {
      if (!initialized_) {
        initialize();
      }

      state_->updTime() = marker_refs_->getValidTimeRange().get(0);
      ik_failed = false;
    }

    if (initialized_) {
      ik_solver_->updateMarkersReference(marker_refs_);
    }
  }

  if (params_.use_link_orientations) {
    if (orientation_refs_ && orientation_refs_->getTimes().size() == 1) {
      if (!initialized_) {
        initialize();
      }

      state_->updTime() = orientation_refs_->getTimes().front();
      ik_failed = false;
    }

    if (initialized_) {
      ik_solver_->updateOrientationsReference(orientation_refs_);
    }
  }

  marker_refs_.reset();
  orientation_refs_.reset();

  if (ik_failed) {
    return false;
  }

  ik_solver_->assemble(*state_);
  model_->realizeReport(*state_);

  if (params_.use_visualizer) {
    model_->getVisualizer().show(*state_);
  }

  return true;
}

std::string hiros::opensim_ik::RTIKTool::getJointAngleName(int idx) const {
  if (idx < 0 || idx >= model_->getCoordinateSet().getSize()) {
    std::cerr << "RTIKTool Warning: Joint angle with index " << idx
              << " not found" << std::endl;
    return std::string();
  }

  return model_->getCoordinateSet().get(idx).getName();
}

double hiros::opensim_ik::RTIKTool::getJointAngleValue(int idx,
                                                       bool in_degrees) const {
  if (idx < 0 || idx >= model_->getCoordinateSet().getSize()) {
    std::cerr << "RTIKTool Warning: Joint angle with index " << idx
              << " not found" << std::endl;
    return std::numeric_limits<double>::quiet_NaN();
  }

  double val = model_->getCoordinateSet().get(idx).getStateVariableValue(
      *state_, "value");

  if (in_degrees && model_->getCoordinateSet().get(idx).getMotionType() ==
                        OpenSim::Coordinate::MotionType::Rotational) {
    return val * static_cast<double>(SimTK_RADIAN_TO_DEGREE);
  }
  return val;
}

double hiros::opensim_ik::RTIKTool::getJointAngleVelocity(
    int idx, bool in_degrees) const {
  if (idx < 0 || idx >= model_->getCoordinateSet().getSize()) {
    std::cerr << "RTIKTool Warning: Joint angle with index " << idx
              << " not found" << std::endl;
    return std::numeric_limits<double>::quiet_NaN();
  }

  double val = model_->getCoordinateSet().get(idx).getStateVariableValue(
      *state_, "speed");

  // TODO: check if it works
  if (in_degrees && model_->getCoordinateSet().get(idx).getMotionType() ==
                        OpenSim::Coordinate::MotionType::Rotational) {
    return val * static_cast<double>(SimTK_RADIAN_TO_DEGREE);
  }
  return val;
}

std::vector<std::string> hiros::opensim_ik::RTIKTool::getJointAngleNames()
    const {
  auto n_joint_angles = model_->getCoordinateSet().getSize();
  if (n_joint_angles <= 0) {
    std::cerr << "RTIKTool Warning: No joint angle names found" << std::endl;
  }

  std::vector<std::string> values;
  values.reserve(static_cast<size_t>(n_joint_angles));
  for (int ja_idx = 0; ja_idx < n_joint_angles; ++ja_idx) {
    values.push_back(getJointAngleName(ja_idx));
  }
  return values;
}

std::vector<double> hiros::opensim_ik::RTIKTool::getJointAngleValues(
    bool in_degrees) const {
  auto n_joint_angles = model_->getCoordinateSet().getSize();
  if (n_joint_angles <= 0) {
    std::cerr << "RTIKTool Warning: No joint angle positions found"
              << std::endl;
  }

  std::vector<double> values;
  values.reserve(static_cast<size_t>(n_joint_angles));
  for (int ja_idx = 0; ja_idx < n_joint_angles; ++ja_idx) {
    values.push_back(getJointAngleValue(ja_idx, in_degrees));
  }
  return values;
}

std::vector<double> hiros::opensim_ik::RTIKTool::getJointAngleVelocities(
    bool in_degrees) const {
  auto n_joint_angles = model_->getCoordinateSet().getSize();
  if (n_joint_angles <= 0) {
    std::cerr << "RTIKTool Warning: No joint angle velocities found"
              << std::endl;
  }

  std::vector<double> values;
  values.reserve(static_cast<size_t>(n_joint_angles));
  for (int ja_idx = 0; ja_idx < n_joint_angles; ++ja_idx) {
    values.push_back(getJointAngleVelocity(ja_idx, in_degrees));
  }
  return values;
}

std::string hiros::opensim_ik::RTIKTool::getMarkerName(int idx) const {
  if (idx < 0 || idx >= model_->getMarkerSet().getSize()) {
    std::cerr << "RTIKTool Warning: Marker with index " << idx << " not found"
              << std::endl;
    return std::string();
  }

  return model_->getMarkerSet().get(idx).getName();
}

SimTK::Vec3 hiros::opensim_ik::RTIKTool::getMarkerPosition(int idx) const {
  if (idx < 0 || idx >= model_->getMarkerSet().getSize()) {
    std::cerr << "RTIKTool Warning: Marker with index " << idx << " not found"
              << std::endl;
    return SimTK::Vec3(std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::quiet_NaN());
  }

  return model_->getMarkerSet().get(idx).getLocationInGround(*state_);
}

SimTK::Vec3 hiros::opensim_ik::RTIKTool::getMarkerVelocity(int idx) const {
  if (idx < 0 || idx >= model_->getMarkerSet().getSize()) {
    std::cerr << "RTIKTool Warning: Marker with index " << idx << " not found"
              << std::endl;
    return SimTK::Vec3(std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::quiet_NaN());
  }

  return model_->getMarkerSet().get(idx).getVelocityInGround(*state_);
}

SimTK::Vec3 hiros::opensim_ik::RTIKTool::getMarkerAcceleration(int idx) const {
  if (idx < 0 || idx >= model_->getMarkerSet().getSize()) {
    std::cerr << "RTIKTool Warning: Marker with index " << idx << " not found"
              << std::endl;
    return SimTK::Vec3(std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::quiet_NaN());
  }

  return model_->getMarkerSet().get(idx).getAccelerationInGround(*state_);
}

std::vector<std::string> hiros::opensim_ik::RTIKTool::getMarkerNames() const {
  auto n_markers = model_->getMarkerSet().getSize();
  if (n_markers <= 0) {
    std::cerr << "RTIKTool Warning: No marker names found" << std::endl;
  }

  std::vector<std::string> values;
  values.reserve(static_cast<size_t>(n_markers));
  for (int mk_idx = 0; mk_idx < n_markers; ++mk_idx) {
    values.push_back(getMarkerName(mk_idx));
  }
  return values;
}

std::vector<SimTK::Vec3> hiros::opensim_ik::RTIKTool::getMarkerPositions()
    const {
  auto n_markers = model_->getMarkerSet().getSize();
  if (n_markers <= 0) {
    std::cerr << "RTIKTool Warning: No marker positions found" << std::endl;
  }

  std::vector<SimTK::Vec3> values;
  values.reserve(static_cast<size_t>(n_markers));
  for (int mk_idx = 0; mk_idx < n_markers; ++mk_idx) {
    values.push_back(getMarkerPosition(mk_idx));
  }
  return values;
}

std::vector<SimTK::Vec3> hiros::opensim_ik::RTIKTool::getMarkerVelocities()
    const {
  auto n_markers = model_->getMarkerSet().getSize();
  if (n_markers <= 0) {
    std::cerr << "RTIKTool Warning: No marker velocities found" << std::endl;
  }

  std::vector<SimTK::Vec3> values;
  values.reserve(static_cast<size_t>(n_markers));
  for (int mk_idx = 0; mk_idx < n_markers; ++mk_idx) {
    values.push_back(getMarkerVelocity(mk_idx));
  }
  return values;
}

std::vector<SimTK::Vec3> hiros::opensim_ik::RTIKTool::getMarkerAccelerations()
    const {
  auto n_markers = model_->getMarkerSet().getSize();
  if (n_markers <= 0) {
    std::cerr << "RTIKTool Warning: No marker acccelerations found"
              << std::endl;
  }

  std::vector<SimTK::Vec3> values;
  values.reserve(static_cast<size_t>(n_markers));
  for (int mk_idx = 0; mk_idx < n_markers; ++mk_idx) {
    values.push_back(getMarkerAcceleration(mk_idx));
  }
  return values;
}

void hiros::opensim_ik::RTIKTool::initialize() {
  if (!model_) {
    std::cerr << "RTIKTool Error: A model must be set before running IK"
              << std::endl;
    exit(EXIT_FAILURE);
  }

  state_ = std::make_unique<SimTK::State>(model_->initSystem());
  //  *m_state = m_model->initSystem(); TODO: ?

  std::shared_ptr<OpenSim::MarkersReference> marker_refs;
  std::shared_ptr<OpenSim::OrientationsReference> orientation_refs;

  if (params_.use_marker_positions && marker_refs_ != nullptr &&
      marker_refs_->getNumRefs() > 0) {
    state_->updTime() = marker_refs_->getValidTimeRange().get(0);
    marker_refs = marker_refs_;
  }
  if (params_.use_link_orientations && orientation_refs_ != nullptr &&
      orientation_refs_->getNumRefs() > 0) {
    state_->updTime() = orientation_refs_->getTimes().front();
    orientation_refs = orientation_refs_;
  }

  SimTK::Array_<OpenSim::CoordinateReference> coordinate_refs;

  ik_solver_ = std::make_unique<OpenSim::InverseKinematicsSolver>(
      *model_.get(), marker_refs, orientation_refs, coordinate_refs);
  ik_solver_->updateMarkersWeight(params_.markers_weight);
  ik_solver_->updateOrientationsWeight(params_.orientations_weight);
  ik_solver_->setAccuracy(params_.accuracy);
  ik_solver_->assemble(*state_);

  if (params_.use_visualizer) {
    model_->updVisualizer().updSimbodyVisualizer().setWindowTitle(
        model_->getName() + " - Inverse Kinematics");
    model_->updVisualizer().updSimbodyVisualizer().setDesiredFrameRate(
        std::numeric_limits<double>::max());
    model_->updVisualizer().updSimbodyVisualizer().setShowFrameRate(true);
  }

  initialized_ = true;
}
