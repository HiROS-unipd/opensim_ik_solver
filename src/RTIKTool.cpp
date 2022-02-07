// Internal dependencies
#include "opensim_ik_solver/RTIKTool.h"

hiros::opensim_ik::RTIKTool::RTIKTool(const double& t_accuracy, const SimTK::Rotation& t_sensor_to_opensim)
  : m_initialized(false)
  , m_sensor_to_opensim(t_sensor_to_opensim)
  , m_accuracy(t_accuracy)
  , m_use_visualizer(false)
{}

hiros::opensim_ik::RTIKTool::RTIKTool(const OpenSim::Model& t_model,
                                      const double& t_accuracy,
                                      const SimTK::Rotation& t_sensor_to_opensim)
  : RTIKTool(t_accuracy, t_sensor_to_opensim)
{
  setModel(t_model);
}

hiros::opensim_ik::RTIKTool::~RTIKTool() {}

void hiros::opensim_ik::RTIKTool::setModel(const OpenSim::Model& t_model)
{
  m_model = std::make_unique<OpenSim::Model>(t_model);
  m_model->finalizeFromProperties();
}

void hiros::opensim_ik::RTIKTool::enableVisualizer()
{
  m_use_visualizer = true;
  m_model->setUseVisualizer(m_use_visualizer);
}

bool hiros::opensim_ik::RTIKTool::runSingleFrameIK(const OpenSim::OrientationsReference& t_orientation_refs)
{
  updateOrientationsReference(t_orientation_refs);
  return runSingleFrameIK();
}

bool hiros::opensim_ik::RTIKTool::runSingleFrameIK(
  const OpenSim::TimeSeriesTable_<SimTK::Rotation_<double>>& t_orientations,
  const OpenSim::Set<OpenSim::OrientationWeight>* t_weights)
{
  updateOrientationsReference(t_orientations, t_weights);
  return runSingleFrameIK();
}

void hiros::opensim_ik::RTIKTool::updateOrientationsReference(const OpenSim::OrientationsReference& t_orientation_refs)
{
  m_orientation_refs = std::make_shared<OpenSim::OrientationsReference>(t_orientation_refs);
}

void hiros::opensim_ik::RTIKTool::updateOrientationsReference(
  const OpenSim::TimeSeriesTable_<SimTK::Rotation_<double>>& t_orientations,
  const OpenSim::Set<OpenSim::OrientationWeight>* t_weights)
{
  OpenSim::TimeSeriesTable_<SimTK::Rotation_<double>> orientations = t_orientations;

  for (int i = 0; i < static_cast<int>(orientations.getNumColumns()); ++i) {
    orientations.updRowAtIndex(0)[i] = m_sensor_to_opensim * orientations.updRowAtIndex(0)[i];
  }

  m_orientation_refs =
    std::make_shared<OpenSim::OrientationsReference>(OpenSim::OrientationsReference(orientations, t_weights));
};

bool hiros::opensim_ik::RTIKTool::runSingleFrameIK()
{
  if (m_orientation_refs == nullptr) {
    return false;
  }

  if (m_orientation_refs->getTimes().empty()) {
    std::cout << "Warning: the orientation reference is empty. Skipping" << std::endl;
    return false;
  }

  if (m_orientation_refs->getTimes().size() > 1) {
    std::cout << "Warning: the orientation reference contains multiple frames. Skipping" << std::endl;
    return false;
  }

  if (!m_initialized) {
    initialize();
  }

  m_ik_solver->updateOrientationsReference(m_orientation_refs);
  m_state->updTime() = m_orientation_refs->getTimes().front();
  m_ik_solver->track(*m_state);

  if (m_use_visualizer) {
    m_model->getVisualizer().show(*m_state);
  }

  m_orientation_refs.reset();

  return true;
}

std::string hiros::opensim_ik::RTIKTool::getJointAngleName(int t_idx) const
{
  if (t_idx < 0 || t_idx >= m_model->getCoordinateSet().getSize()) {
    std::cout << "Joint angle with index " << t_idx << " not found" << std::endl;
    return std::string();
  }

  return m_model->getCoordinateSet().get(t_idx).getName();
}

double hiros::opensim_ik::RTIKTool::getJointAnglePosition(int t_idx, bool t_in_degrees) const
{
  if (t_idx < 0 || t_idx >= m_model->getCoordinateSet().getSize()) {
    std::cout << "Joint angle with index " << t_idx << " not found" << std::endl;
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
    std::cout << "Joint angle with index " << t_idx << " not found" << std::endl;
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
  std::vector<double> values;
  values.reserve(static_cast<size_t>(n_joint_angles));
  for (int ja_idx = 0; ja_idx < n_joint_angles; ++ja_idx) {
    values.push_back(getJointAngleVelocity(ja_idx, t_in_degrees));
  }
  return values;
}

void hiros::opensim_ik::RTIKTool::initialize()
{
  if (!m_model) {
    std::cerr << "RTIKTool... Error: A model must be set before running IK." << std::endl;
    exit(EXIT_FAILURE);
  }

  m_state = std::make_unique<SimTK::State>(m_model->initSystem());
  *m_state = m_model->initSystem();
  m_state->updTime() = m_orientation_refs->getTimes().front();

  OpenSim::MarkersReference marker_refs;
  SimTK::Array_<OpenSim::CoordinateReference> coordinate_refs;

  m_ik_solver = std::make_unique<OpenSim::InverseKinematicsSolver>(
    *m_model.get(), marker_refs, *m_orientation_refs.get(), coordinate_refs);
  m_ik_solver->setAccuracy(m_accuracy);
  m_ik_solver->assemble(*m_state);

  if (m_use_visualizer) {
    m_model->updVisualizer().updSimbodyVisualizer().setWindowTitle(m_model->getName() + " - Inverse Kinematics");
    m_model->updVisualizer().updSimbodyVisualizer().setDesiredFrameRate(std::numeric_limits<double>::max());
    m_model->updVisualizer().updSimbodyVisualizer().setShowFrameRate(true);
  }

  m_initialized = true;
}
