// Internal dependencies
#include "opensim_ik_solver/RTIMUIKTool.h"

hiros::opensim_ik::RTIMUIKTool::RTIMUIKTool(const double& t_accuracy, const SimTK::Rotation& t_sensor_to_opensim)
  : m_initialized(false)
  , m_sensor_to_opensim(t_sensor_to_opensim)
  , m_accuracy(t_accuracy)
  , m_use_visualizer(false)
{
  m_orientation_refs = std::make_unique<OpenSim::OrientationsReference>();
}

hiros::opensim_ik::RTIMUIKTool::RTIMUIKTool(const OpenSim::Model& t_model,
                                            const double& t_accuracy,
                                            const SimTK::Rotation& t_sensor_to_opensim)
  : RTIMUIKTool(t_accuracy, t_sensor_to_opensim)
{
  setModel(t_model);

  m_orientation_refs = std::make_unique<OpenSim::OrientationsReference>();
}

hiros::opensim_ik::RTIMUIKTool::~RTIMUIKTool() {}

void hiros::opensim_ik::RTIMUIKTool::setModel(const OpenSim::Model& t_model)
{
  m_model = std::make_unique<OpenSim::Model>(t_model);
  m_model->finalizeFromProperties();
}

void hiros::opensim_ik::RTIMUIKTool::enableVisualizer()
{
  m_use_visualizer = true;
  m_model->setUseVisualizer(m_use_visualizer);
}

bool hiros::opensim_ik::RTIMUIKTool::runSingleFrameIK(const OpenSim::OrientationsReference& t_orientation_refs)
{
  updateOrientationsReference(t_orientation_refs);
  return runSingleFrameIK();
}

bool hiros::opensim_ik::RTIMUIKTool::runSingleFrameIK(
  const OpenSim::TimeSeriesTable_<SimTK::Rotation_<double>>& t_orientations,
  const OpenSim::Set<OpenSim::OrientationWeight>* t_weights)
{
  updateOrientationsReference(t_orientations, t_weights);
  return runSingleFrameIK();
}

void hiros::opensim_ik::RTIMUIKTool::updateOrientationsReference(
  const OpenSim::OrientationsReference& t_orientation_refs)
{
  m_orientation_refs = std::make_unique<OpenSim::OrientationsReference>(t_orientation_refs);
}

void hiros::opensim_ik::RTIMUIKTool::updateOrientationsReference(
  const OpenSim::TimeSeriesTable_<SimTK::Rotation_<double>>& t_orientations,
  const OpenSim::Set<OpenSim::OrientationWeight>* t_weights)
{
  OpenSim::TimeSeriesTable_<SimTK::Rotation_<double>> orientations = t_orientations;

  for (int i = 0; i < static_cast<int>(orientations.getNumColumns()); ++i) {
    orientations.updRowAtIndex(0)[i] = m_sensor_to_opensim * orientations.updRowAtIndex(0)[i];
  }

  m_orientation_refs =
    std::make_unique<OpenSim::OrientationsReference>(OpenSim::OrientationsReference(orientations, t_weights));
};

bool hiros::opensim_ik::RTIMUIKTool::runSingleFrameIK()
{
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

  m_ik_solver->updateOrientationsReference(*m_orientation_refs);
  m_state->updTime() = m_orientation_refs->getTimes().front();
  m_ik_solver->track(*m_state);

  if (m_use_visualizer) {
    m_model->getVisualizer().show(*m_state);
  }

  m_orientation_refs.reset();

  return true;
}

void hiros::opensim_ik::RTIMUIKTool::initialize()
{
  if (!m_model) {
    std::cerr << "RTIMUIKTool... Error: A model must be set before running IK." << std::endl;
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
