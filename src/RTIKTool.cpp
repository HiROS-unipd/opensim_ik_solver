// Internal dependencies
#include "opensim_ik_solver/RTIKTool.h"

hiros::opensim_ik::RTIKTool::RTIKTool(const IKToolParameters& t_params)
  : m_initialized(false)
  , m_params(t_params)
{
  m_params.use_visualizer = false; // TODO: currently not supported
}

hiros::opensim_ik::RTIKTool::RTIKTool(const OpenSim::Model& t_model, const IKToolParameters& t_params)
  : RTIKTool(t_params)
{
  setModel(t_model);
}

hiros::opensim_ik::RTIKTool::~RTIKTool() {}

void hiros::opensim_ik::RTIKTool::setModel(const OpenSim::Model& t_model)
{
  m_model = std::make_unique<OpenSim::Model>(t_model);
  m_model->finalizeFromProperties();
  m_initialized = false;
}

void hiros::opensim_ik::RTIKTool::enableVisualizer()
{
  m_params.use_visualizer = true;
  m_model->setUseVisualizer(m_params.use_visualizer);
}

void hiros::opensim_ik::RTIKTool::disableVisualizer()
{
  m_params.use_visualizer = false;
  m_model->setUseVisualizer(m_params.use_visualizer);
}

bool hiros::opensim_ik::RTIKTool::runSingleFrameIK(const OpenSim::MarkersReference& t_marker_refs,
                                                   const OpenSim::OrientationsReference& t_orientation_refs)
{
  updateReference(t_marker_refs, t_orientation_refs);
  return runSingleFrameIK();
}

bool hiros::opensim_ik::RTIKTool::runSingleFrameIK(const OpenSim::OrientationsReference& t_orientation_refs)
{
  updateReference(t_orientation_refs);
  return runSingleFrameIK();
}

void hiros::opensim_ik::RTIKTool::updateReference(const OpenSim::MarkersReference& t_marker_refs,
                                                  const OpenSim::OrientationsReference& t_orientation_refs)
{
  m_marker_refs = std::make_shared<OpenSim::MarkersReference>(t_marker_refs);
  m_orientation_refs = std::make_shared<OpenSim::OrientationsReference>(t_orientation_refs);
}

void hiros::opensim_ik::RTIKTool::updateReference(const OpenSim::OrientationsReference& t_orientation_refs)
{
  m_orientation_refs = std::make_shared<OpenSim::OrientationsReference>(t_orientation_refs);
}

bool hiros::opensim_ik::RTIKTool::runSingleFrameIK()
{
  bool ik_failed = true;

  if (m_params.use_marker_positions) {
    if (m_marker_refs == nullptr || m_marker_refs->getNumFrames() == 0) {
      std::cerr << "RTIKTool Warning: the marker reference is empty. Skipping" << std::endl;
    }
    else if (m_marker_refs->getNumFrames() > 1) {
      std::cerr << "RTIKTool Warning: the marker reference contains multiple frames. Skipping" << std::endl;
    }
    else {
      if (!m_initialized) {
        initialize();
      }

      m_ik_solver->updateMarkersReference(m_marker_refs);
      m_state->updTime() = m_marker_refs->getValidTimeRange().get(0);
      ik_failed = false;
    }
  }

  if (m_params.use_link_orientations) {
    if (m_orientation_refs == nullptr || m_orientation_refs->getTimes().empty()) {
      std::cerr << "RTIKTool Warning: the orientation reference is empty. Skipping" << std::endl;
    }
    else if (m_orientation_refs->getTimes().size() > 1) {
      std::cerr << "RTIKTool Warning: the orientation reference contains multiple frames. Skipping" << std::endl;
    }
    else {
      if (!m_initialized) {
        initialize();
      }

      m_ik_solver->updateOrientationsReference(m_orientation_refs);
      m_state->updTime() = m_orientation_refs->getTimes().front();
      ik_failed = false;
    }
  }

  m_marker_refs.reset();
  m_orientation_refs.reset();

  if (ik_failed) {
    return false;
  }

  m_ik_solver->assemble(*m_state);
  m_model->realizeReport(*m_state);

  if (m_params.use_visualizer) {
    m_model->getVisualizer().show(*m_state);
  }

  return true;
}

std::string hiros::opensim_ik::RTIKTool::getJointAngleName(int t_idx) const
{
  if (t_idx < 0 || t_idx >= m_model->getCoordinateSet().getSize()) {
    std::cerr << "RTIKTool Warning: Joint angle with index " << t_idx << " not found" << std::endl;
    return std::string();
  }

  return m_model->getCoordinateSet().get(t_idx).getName();
}

double hiros::opensim_ik::RTIKTool::getJointAnglePosition(int t_idx, bool t_in_degrees) const
{
  if (t_idx < 0 || t_idx >= m_model->getCoordinateSet().getSize()) {
    std::cerr << "RTIKTool Warning: Joint angle with index " << t_idx << " not found" << std::endl;
    return std::numeric_limits<double>::quiet_NaN();
  }

  double val = m_model->getCoordinateSet().get(t_idx).getStateVariableValue(*m_state, "value");

  if (t_in_degrees
      && m_model->getCoordinateSet().get(t_idx).getMotionType() == OpenSim::Coordinate::MotionType::Rotational) {
    return val * static_cast<double>(SimTK_RADIAN_TO_DEGREE);
  }
  return val;
}

double hiros::opensim_ik::RTIKTool::getJointAngleVelocity(int t_idx, bool t_in_degrees) const
{
  if (t_idx < 0 || t_idx >= m_model->getCoordinateSet().getSize()) {
    std::cerr << "RTIKTool Warning: Joint angle with index " << t_idx << " not found" << std::endl;
    return std::numeric_limits<double>::quiet_NaN();
  }

  double val = m_model->getCoordinateSet().get(t_idx).getStateVariableValue(*m_state, "speed");

  // TODO: check if it works
  if (t_in_degrees
      && m_model->getCoordinateSet().get(t_idx).getMotionType() == OpenSim::Coordinate::MotionType::Rotational) {
    return val * static_cast<double>(SimTK_RADIAN_TO_DEGREE);
  }
  return val;
}

std::vector<std::string> hiros::opensim_ik::RTIKTool::getJointAngleNames() const
{
  auto n_joint_angles = m_model->getCoordinateSet().getSize();
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

std::vector<double> hiros::opensim_ik::RTIKTool::getJointAnglePositions(bool t_in_degrees) const
{
  auto n_joint_angles = m_model->getCoordinateSet().getSize();
  if (n_joint_angles <= 0) {
    std::cerr << "RTIKTool Warning: No joint angle positions found" << std::endl;
  }

  std::vector<double> values;
  values.reserve(static_cast<size_t>(n_joint_angles));
  for (int ja_idx = 0; ja_idx < n_joint_angles; ++ja_idx) {
    values.push_back(getJointAnglePosition(ja_idx, t_in_degrees));
  }
  return values;
}

std::vector<double> hiros::opensim_ik::RTIKTool::getJointAngleVelocities(bool t_in_degrees) const
{
  auto n_joint_angles = m_model->getCoordinateSet().getSize();
  if (n_joint_angles <= 0) {
    std::cerr << "RTIKTool Warning: No joint angle velocities found" << std::endl;
  }

  std::vector<double> values;
  values.reserve(static_cast<size_t>(n_joint_angles));
  for (int ja_idx = 0; ja_idx < n_joint_angles; ++ja_idx) {
    values.push_back(getJointAngleVelocity(ja_idx, t_in_degrees));
  }
  return values;
}

std::string hiros::opensim_ik::RTIKTool::getMarkerName(int t_idx) const
{
  if (t_idx < 0 || t_idx >= m_model->getMarkerSet().getSize()) {
    std::cerr << "RTIKTool Warning: Marker with index " << t_idx << " not found" << std::endl;
    return std::string();
  }

  return m_model->getMarkerSet().get(t_idx).getName();
}

SimTK::Vec3 hiros::opensim_ik::RTIKTool::getMarkerPosition(int t_idx) const
{
  if (t_idx < 0 || t_idx >= m_model->getMarkerSet().getSize()) {
    std::cerr << "RTIKTool Warning: Marker with index " << t_idx << " not found" << std::endl;
    return SimTK::Vec3(std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::quiet_NaN());
  }

  return m_model->getMarkerSet().get(t_idx).getLocationInGround(*m_state);
}

SimTK::Vec3 hiros::opensim_ik::RTIKTool::getMarkerVelocity(int t_idx) const
{
  if (t_idx < 0 || t_idx >= m_model->getMarkerSet().getSize()) {
    std::cerr << "RTIKTool Warning: Marker with index " << t_idx << " not found" << std::endl;
    return SimTK::Vec3(std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::quiet_NaN());
  }

  return m_model->getMarkerSet().get(t_idx).getVelocityInGround(*m_state);
}

SimTK::Vec3 hiros::opensim_ik::RTIKTool::getMarkerAcceleration(int t_idx) const
{
  if (t_idx < 0 || t_idx >= m_model->getMarkerSet().getSize()) {
    std::cerr << "RTIKTool Warning: Marker with index " << t_idx << " not found" << std::endl;
    return SimTK::Vec3(std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::quiet_NaN());
  }

  return m_model->getMarkerSet().get(t_idx).getAccelerationInGround(*m_state);
}

std::vector<std::string> hiros::opensim_ik::RTIKTool::getMarkerNames() const
{
  auto n_markers = m_model->getMarkerSet().getSize();
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

std::vector<SimTK::Vec3> hiros::opensim_ik::RTIKTool::getMarkerPositions() const
{
  auto n_markers = m_model->getMarkerSet().getSize();
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

std::vector<SimTK::Vec3> hiros::opensim_ik::RTIKTool::getMarkerVelocities() const
{
  auto n_markers = m_model->getMarkerSet().getSize();
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

std::vector<SimTK::Vec3> hiros::opensim_ik::RTIKTool::getMarkerAccelerations() const
{
  auto n_markers = m_model->getMarkerSet().getSize();
  if (n_markers <= 0) {
    std::cerr << "RTIKTool Warning: No marker acccelerations found" << std::endl;
  }

  std::vector<SimTK::Vec3> values;
  values.reserve(static_cast<size_t>(n_markers));
  for (int mk_idx = 0; mk_idx < n_markers; ++mk_idx) {
    values.push_back(getMarkerAcceleration(mk_idx));
  }
  return values;
}

void hiros::opensim_ik::RTIKTool::initialize()
{
  if (!m_model) {
    std::cerr << "RTIKTool Error: A model must be set before running IK" << std::endl;
    exit(EXIT_FAILURE);
  }

  m_state = std::make_unique<SimTK::State>(m_model->initSystem());
  *m_state = m_model->initSystem();

  std::shared_ptr<OpenSim::MarkersReference> marker_refs;
  std::shared_ptr<OpenSim::OrientationsReference> orientation_refs;

  if (m_params.use_marker_positions && m_marker_refs->getNumRefs() > 0) {
    m_state->updTime() = m_marker_refs->getValidTimeRange().get(0);
    marker_refs = m_marker_refs;
  }
  if (m_params.use_link_orientations && m_orientation_refs->getNumRefs() > 0) {
    m_state->updTime() = m_orientation_refs->getTimes().front();
    orientation_refs = m_orientation_refs;
  }

  SimTK::Array_<OpenSim::CoordinateReference> coordinate_refs;

  m_ik_solver =
    std::make_unique<OpenSim::InverseKinematicsSolver>(*m_model.get(), marker_refs, orientation_refs, coordinate_refs);
  m_ik_solver->updateMarkersWeight(m_params.markers_weight);
  m_ik_solver->updateOrientationsWeight(m_params.orientations_weight);
  m_ik_solver->setAccuracy(m_params.accuracy);
  m_ik_solver->assemble(*m_state);

  if (m_params.use_visualizer) {
    m_model->updVisualizer().updSimbodyVisualizer().setWindowTitle(m_model->getName() + " - Inverse Kinematics");
    m_model->updVisualizer().updSimbodyVisualizer().setDesiredFrameRate(std::numeric_limits<double>::max());
    m_model->updVisualizer().updSimbodyVisualizer().setShowFrameRate(true);
  }

  m_initialized = true;
}
