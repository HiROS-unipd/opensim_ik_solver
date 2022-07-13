// OpenSim dependencies
#include "OpenSim/Simulation/InverseKinematicsSolver.h"
#include "OpenSim/Simulation/OpenSense/OpenSenseUtilities.h"

// Internal dependencies
#include "opensim_ik_solver/RTIMUPlacer.h"

hiros::opensim_ik::RTIMUPlacer::RTIMUPlacer(const IMUPlacerParameters& t_params)
  : m_initialized(false)
  , m_calibrated(false)
  , m_params(t_params)
  , m_direction_on_imu(SimTK::ZAxis)
{}

hiros::opensim_ik::RTIMUPlacer::RTIMUPlacer(const OpenSim::Model& t_model, const IMUPlacerParameters& t_params)
  : RTIMUPlacer(t_params)
{
  setModel(t_model);
}

hiros::opensim_ik::RTIMUPlacer::~RTIMUPlacer() {}

void hiros::opensim_ik::RTIMUPlacer::setModel(const OpenSim::Model& t_model)
{
  m_model = std::make_unique<OpenSim::Model>(t_model);
  m_model->finalizeFromProperties();
  m_initialized = false;
}

void hiros::opensim_ik::RTIMUPlacer::initializeHeadingCorrection()
{
  if (!m_params.heading_correction) {
    std::cerr << "RTIMUPlacer Error: trying to perform heading correction after setting parameter 'heading_correction' "
                 "to false."
              << std::endl;
  }
  else {
    setDirectionOnImu();
  }
}

void hiros::opensim_ik::RTIMUPlacer::enableVisualizer()
{
  m_params.use_visualizer = true;
  m_model->setUseVisualizer(m_params.use_visualizer);
}

void hiros::opensim_ik::RTIMUPlacer::disableVisualizer()
{
  m_params.use_visualizer = false;
  m_model->setUseVisualizer(m_params.use_visualizer);
}

bool hiros::opensim_ik::RTIMUPlacer::runCalibration(
  const OpenSim::TimeSeriesTable_<SimTK::Quaternion>& t_orientations_table)
{
  updateOrientationsTable(t_orientations_table);
  return runCalibration();
}

OpenSim::Model& hiros::opensim_ik::RTIMUPlacer::getCalibratedModel() const
{
  if (!m_calibrated) {
    OPENSIM_THROW(OpenSim::Exception,
                  "Attempt to retrieve calibrated model without invoking RTIMUPlacer::runSingleFrameCalibration.");
  }
  return *m_model;
}

void hiros::opensim_ik::RTIMUPlacer::saveModel(const std::string& t_calibrated_model_path)
{
  if (!m_params.save_calibrated_model) {
    std::cerr << "RTIMUPlacer Warning: Parameter 'save_calibrated_model' was set to false. Overriding" << std::endl;
    m_params.save_calibrated_model = true;
  }

  std::cout << "RTIMUPlacer Info: Saving calibrated model to " << t_calibrated_model_path << std::endl;
  m_model->print(t_calibrated_model_path);
}

void hiros::opensim_ik::RTIMUPlacer::saveModel()
{
  return saveModel(m_params.calibrated_model_path);
}

void hiros::opensim_ik::RTIMUPlacer::setDirectionOnImu()
{
  std::string imu_axis = OpenSim::IO::Lowercase(m_params.base_heading_axis);

  int direction = (imu_axis.front() == '-') ? -1 : 1;

  switch (imu_axis.back()) {
    case 'x':
      m_direction_on_imu = SimTK::CoordinateDirection(SimTK::XAxis, direction);
      break;

    case 'y':
      m_direction_on_imu = SimTK::CoordinateDirection(SimTK::YAxis, direction);
      break;

    case 'z':
      m_direction_on_imu = SimTK::CoordinateDirection(SimTK::ZAxis, direction);
      break;

    default:
      OPENSIM_THROW(OpenSim::Exception,
                    "Invalid specification of heading axis '" + m_params.base_heading_axis + "' found.");
  }
}

void hiros::opensim_ik::RTIMUPlacer::updateOrientationsTable(
  const OpenSim::TimeSeriesTable_<SimTK::Quaternion>& t_orientations_table)
{
  m_orientations_table = std::make_unique<OpenSim::TimeSeriesTable_<SimTK::Quaternion>>(t_orientations_table);
}

bool hiros::opensim_ik::RTIMUPlacer::runCalibration()
{
  if (m_orientations_table->getIndependentColumn().empty()) {
    std::cout << "RTIMUPlacer... Warning: The orientation table is empty. Skipping." << std::endl;
    return false;
  }

  if (!m_initialized) {
    initialize();
  }

  // TODO: check if works
  m_state = std::make_unique<SimTK::State>(m_model->initSystem());
  m_state->updTime() = m_orientations_table->getIndependentColumn().front();

  // Default pose of the model
  m_model->realizePosition(*m_state.get());

  if (m_params.heading_correction) {
    applyHeadingCorrection();
  }

  m_orientations_data = OpenSim::OpenSenseUtilities::convertQuaternionsToRotations(*m_orientations_table.get());
  m_imu_labels = m_orientations_data.getColumnLabels();
  m_times = m_orientations_data.getIndependentColumn();

  if (m_times.size() == 1) {
    m_rotations = m_orientations_data.updRowAtIndex(0);
  }
  else {
    m_rotations = m_orientations_data.averageRow(m_times.front(), m_times.back());
  }

  // Compute the transform of each of the IMU bodies in ground
  computeTransforms();

  // Compute the relative offset of each IMU relative to the body it is attached to
  computeOffsets();

  m_model->finalizeConnections();

  if (m_params.use_visualizer) {
    visualizeCalibratedModel();
  }

  return m_calibrated = true;
}

void hiros::opensim_ik::RTIMUPlacer::initialize()
{
  if (!m_model) {
    OPENSIM_THROW(OpenSim::Exception, "No model passed to RTIMUPlacer. Call setModel() method.");
  }

  initializeHeadingCorrection();
  m_model->setUseVisualizer(m_params.use_visualizer);

  m_initialized = true;
}

bool hiros::opensim_ik::RTIMUPlacer::applyHeadingCorrection()
{
  try {
    // Compute the rotation matrix so that (e.g. "pelvis_imu" + SimTK::ZAxis) lines up with model forward (+X)
    SimTK::Vec3 heading_rotation_vec = OpenSim::OpenSenseUtilities::computeHeadingCorrection(
      *m_model.get(), *m_state.get(), *m_orientations_table.get(), m_params.base_imu_label, m_direction_on_imu);

    SimTK::Rotation heading_rotation(SimTK::BodyOrSpaceType::SpaceRotationSequence,
                                     heading_rotation_vec[0],
                                     SimTK::XAxis,
                                     heading_rotation_vec[1],
                                     SimTK::YAxis,
                                     heading_rotation_vec[2],
                                     SimTK::ZAxis);

    OpenSim::OpenSenseUtilities::rotateOrientationTable(*m_orientations_table.get(), heading_rotation);

    return true;
  }
  catch (const OpenSim::Exception& ex) {
    std::cout << "RTIMUPlacer... Warning: IMU '" << m_params.base_imu_label
              << "' not found for heading correction. Skipping." << std::endl;

    return false;
  }
}

void hiros::opensim_ik::RTIMUPlacer::computeTransforms()
{
  m_bodies.reserve(m_imu_labels.size());

  for (const auto& imu_name : m_imu_labels) {
    auto physical_offset_frame = m_model->findComponent<OpenSim::PhysicalOffsetFrame>(imu_name);

    if (physical_offset_frame) {
      auto body_name = physical_offset_frame->getParentFrame().getName();

      auto body = m_model->findComponent<OpenSim::Body>(body_name);

      if (body) {
        m_bodies.push_back(const_cast<OpenSim::Body*>(body));
        m_imu_bodies_in_ground.emplace(imu_name, body->getTransformInGround(*m_state.get()).R());
      }
    }
  }
}

void hiros::opensim_ik::RTIMUPlacer::computeOffsets()
{
  unsigned long imu_index = 0;
  for (const auto& imu_name : m_imu_labels) {
    if (m_imu_bodies_in_ground.find(imu_name) != m_imu_bodies_in_ground.end()) {
      SimTK::Rotation r_fb = ~m_imu_bodies_in_ground.at(imu_name) * m_rotations[static_cast<int>(imu_index)];

      OpenSim::PhysicalOffsetFrame* imu_offset =
        const_cast<OpenSim::PhysicalOffsetFrame*>(m_model->findComponent<OpenSim::PhysicalOffsetFrame>(imu_name));

      if (imu_offset) {
        SimTK::Transform offset_tf = imu_offset->getOffsetTransform();
        offset_tf.updR() = r_fb;
        imu_offset->setOffsetTransform(offset_tf);

        if (imu_offset->get_translation() == SimTK::Vec3(0, 0, 0)) {
          auto body = m_bodies.at(imu_index);

          if (body) {
            imu_offset->upd_translation() = body->getMassCenter();
          }
        }
      }
      else {
        auto body = m_bodies.at(imu_index);
        SimTK::Vec3 p_fb(0, 0, 0);

        if (body) {
          p_fb = body->getMassCenter();
        }

        imu_offset = new OpenSim::PhysicalOffsetFrame(imu_name, *m_bodies.at(imu_index), SimTK::Transform(r_fb, p_fb));
        m_bodies.at(imu_index)->addComponent(imu_offset);

        // Create an IMU object in the model connected to imu_offset
        OpenSim::IMU* model_imu = new OpenSim::IMU();
        model_imu->setName(imu_name);
        model_imu->connectSocket_frame(*imu_offset);
        m_model->addModelComponent(model_imu);
      }

      // Add brick representing the IMU if not already present
      if (!m_model->hasComponent(imu_name + "_geom_1")) {
        auto* brick = new OpenSim::Brick(SimTK::Vec3(0.023, 0.015, 0.006));
        brick->setColor(SimTK::Orange);
        imu_offset->attachGeometry(brick);
      }
    }
    imu_index++;
  }
}

void hiros::opensim_ik::RTIMUPlacer::visualizeCalibratedModel()
{
  if (!m_model->getUseVisualizer()) {
    m_model->setUseVisualizer(true);
  }

  SimTK::State& s = m_model->initSystem();
  s.updTime() = m_times.front();

  OpenSim::MarkersReference m_refs{};
  OpenSim::OrientationsReference o_refs(m_orientations_data);
  SimTK::Array_<OpenSim::CoordinateReference> coord_refs{};

  OpenSim::InverseKinematicsSolver ik_solver(*m_model.get(), m_refs, o_refs, coord_refs);
  ik_solver.assemble(s);

  m_model->updVisualizer().updSimbodyVisualizer().setWindowTitle(m_model->getName() + " - IMU Placer");
  m_model->getVisualizer().show(s);
}
