// OpenSim dependencies
#include "OpenSim/Simulation/InverseKinematicsSolver.h"
#include "OpenSim/Simulation/OpenSense/OpenSenseUtilities.h"

// Internal dependencies
#include "opensim_ik_solver/RTIMUPlacer.h"

hiros::opensim_ik::RTIMUPlacer::RTIMUPlacer(const IMUPlacerParameters& params)
    : params_(params) {}

hiros::opensim_ik::RTIMUPlacer::RTIMUPlacer(const OpenSim::Model& model,
                                            const IMUPlacerParameters& params)
    : RTIMUPlacer(params) {
  setModel(model);

  IKToolParameters ik_tool_params;
  ik_tool_params.accuracy = 1.e-4;
  ik_tool_params.use_marker_positions = true;
  rt_ik_tool_ =
      std::make_unique<hiros::opensim_ik::RTIKTool>(*model_, ik_tool_params);
}

hiros::opensim_ik::RTIMUPlacer::~RTIMUPlacer() {}

void hiros::opensim_ik::RTIMUPlacer::setModel(const OpenSim::Model& model) {
  model_ = std::make_unique<OpenSim::Model>(model);
  model_->finalizeFromProperties();
  initialized_ = false;
}

void hiros::opensim_ik::RTIMUPlacer::initializeHeadingCorrection() {
  if (!params_.heading_correction) {
    std::cerr << "RTIMUPlacer Error: trying to perform heading correction "
                 "after setting parameter 'heading_correction' "
                 "to false."
              << std::endl;
  } else {
    setDirectionOnImu();
  }
}

void hiros::opensim_ik::RTIMUPlacer::enableVisualizer() {
  params_.use_visualizer = true;
  model_->setUseVisualizer(params_.use_visualizer);
}

void hiros::opensim_ik::RTIMUPlacer::disableVisualizer() {
  params_.use_visualizer = false;
  model_->setUseVisualizer(params_.use_visualizer);
}

bool hiros::opensim_ik::RTIMUPlacer::runCalibration(
    const OpenSim::TimeSeriesTable_<SimTK::Quaternion>& orientations_table,
    const OpenSim::MarkersReference& markers_reference) {
  updateOrientationsTable(orientations_table);
  updateMarkersReference(markers_reference);
  return runCalibration();
}

OpenSim::Model& hiros::opensim_ik::RTIMUPlacer::getCalibratedModel() const {
  if (!calibrated_) {
    OPENSIM_THROW(OpenSim::Exception,
                  "Attempt to retrieve calibrated model without invoking "
                  "RTIMUPlacer::runSingleFrameCalibration.");
  }
  return *model_;
}

void hiros::opensim_ik::RTIMUPlacer::saveModel(
    const std::string& calibrated_model_path) {
  if (!params_.save_calibrated_model) {
    std::cerr << "RTIMUPlacer Warning: Parameter 'save_calibrated_model' was "
                 "set to false. Overriding"
              << std::endl;
    params_.save_calibrated_model = true;
  }

  std::cout << "RTIMUPlacer Info: Saving calibrated model to "
            << calibrated_model_path << std::endl;
  model_->print(calibrated_model_path);
}

void hiros::opensim_ik::RTIMUPlacer::saveModel() {
  return saveModel(params_.calibrated_model_path);
}

void hiros::opensim_ik::RTIMUPlacer::setDirectionOnImu() {
  std::string imu_axis = OpenSim::IO::Lowercase(params_.base_heading_axis);

  int direction = (imu_axis.front() == '-') ? -1 : 1;

  switch (imu_axis.back()) {
    case 'x':
      direction_on_imu_ = SimTK::CoordinateDirection(SimTK::XAxis, direction);
      break;

    case 'y':
      direction_on_imu_ = SimTK::CoordinateDirection(SimTK::YAxis, direction);
      break;

    case 'z':
      direction_on_imu_ = SimTK::CoordinateDirection(SimTK::ZAxis, direction);
      break;

    default:
      OPENSIM_THROW(OpenSim::Exception,
                    "Invalid specification of heading axis '" +
                        params_.base_heading_axis + "' found.");
  }
}

void hiros::opensim_ik::RTIMUPlacer::updateMarkersReference(
    const OpenSim::MarkersReference& markers_reference) {
  if (markers_reference.getNumFrames() <= 0) {
    return;
  }

  markers_reference_ =
      std::make_unique<OpenSim::MarkersReference>(markers_reference);
}

void hiros::opensim_ik::RTIMUPlacer::updateOrientationsTable(
    const OpenSim::TimeSeriesTable_<SimTK::Quaternion>& orientations_table) {
  orientations_table_ =
      std::make_unique<OpenSim::TimeSeriesTable_<SimTK::Quaternion>>(
          orientations_table);
}

bool hiros::opensim_ik::RTIMUPlacer::runCalibration() {
  if (orientations_table_->getIndependentColumn().empty()) {
    std::cerr
        << "RTIMUPlacer... Warning: The orientation table is empty. Skipping."
        << std::endl;
    return false;
  }

  if (!initialized_) {
    initialize();
  }

  if (markers_reference_) {
    rt_ik_tool_->runSingleFrameIK(*markers_reference_);
    state_ = std::make_unique<SimTK::State>(rt_ik_tool_->getState());
    model_->initSystem();
  } else {
    // TODO: check if works
    state_ = std::make_unique<SimTK::State>(model_->initSystem());
    state_->updTime() = orientations_table_->getIndependentColumn()
                            .front();  // TODO: can remove?
  }

  // Default/marker-based pose of the model
  model_->realizePosition(*state_.get());

  if (params_.heading_correction) {
    applyHeadingCorrection();
  }

  orientations_data_ =
      OpenSim::OpenSenseUtilities::convertQuaternionsToRotations(
          *orientations_table_.get());
  imu_labels_ = orientations_data_.getColumnLabels();
  times_ = orientations_data_.getIndependentColumn();

  if (times_.size() == 1) {
    rotations_ = orientations_data_.updRowAtIndex(0);
  } else {
    rotations_ = orientations_data_.averageRow(times_.front(), times_.back());
  }

  // Compute the transform of each of the IMU bodies in ground
  computeTransforms();

  // Compute the relative offset of each IMU relative to the body it is attached
  // to
  computeOffsets();

  model_->finalizeConnections();

  if (params_.use_visualizer) {
    visualizeCalibratedModel();
  }

  return calibrated_ = true;
}

void hiros::opensim_ik::RTIMUPlacer::initialize() {
  if (!model_) {
    OPENSIM_THROW(OpenSim::Exception,
                  "No model passed to RTIMUPlacer. Call setModel() method.");
  }

  if (params_.heading_correction) {
    initializeHeadingCorrection();
  }

  model_->setUseVisualizer(params_.use_visualizer);

  initialized_ = true;
}

bool hiros::opensim_ik::RTIMUPlacer::applyHeadingCorrection() {
  try {
    // Compute the rotation matrix so that (e.g. "pelvis_imu" + SimTK::ZAxis)
    // lines up with model forward (+X)
    heading_rot_vec_ = OpenSim::OpenSenseUtilities::computeHeadingCorrection(
        *model_.get(), *state_.get(), *orientations_table_.get(),
        params_.base_imu_label, direction_on_imu_);

    SimTK::Rotation heading_rotation(
        SimTK::BodyOrSpaceType::SpaceRotationSequence, heading_rot_vec_[0],
        SimTK::XAxis, heading_rot_vec_[1], SimTK::YAxis, heading_rot_vec_[2],
        SimTK::ZAxis);

    OpenSim::OpenSenseUtilities::rotateOrientationTable(
        *orientations_table_.get(), heading_rotation);

    return true;
  } catch (const OpenSim::Exception& ex) {
    std::cerr << "RTIMUPlacer... Warning: IMU '" << params_.base_imu_label
              << "' not found for heading correction. Skipping." << std::endl;

    return false;
  }
}

void hiros::opensim_ik::RTIMUPlacer::computeTransforms() {
  bodies_.reserve(imu_labels_.size());

  for (const auto& imu_name : imu_labels_) {
    auto physical_offset_frame =
        model_->findComponent<OpenSim::PhysicalOffsetFrame>(imu_name);

    if (physical_offset_frame) {
      auto body_name = physical_offset_frame->getParentFrame().getName();

      auto body = model_->findComponent<OpenSim::Body>(body_name);

      if (body) {
        bodies_.push_back(const_cast<OpenSim::Body*>(body));
        imu_bodies_in_ground_.emplace(
            imu_name, body->getTransformInGround(*state_.get()).R());
      }
    }
  }
}

void hiros::opensim_ik::RTIMUPlacer::computeOffsets() {
  unsigned long imu_index = 0;
  for (const auto& imu_name : imu_labels_) {
    if (imu_bodies_in_ground_.find(imu_name) != imu_bodies_in_ground_.end()) {
      SimTK::Rotation r_fb = ~imu_bodies_in_ground_.at(imu_name) *
                             rotations_[static_cast<int>(imu_index)];

      OpenSim::PhysicalOffsetFrame* imu_offset =
          const_cast<OpenSim::PhysicalOffsetFrame*>(
              model_->findComponent<OpenSim::PhysicalOffsetFrame>(imu_name));

      if (imu_offset) {
        SimTK::Transform offset_tf = imu_offset->getOffsetTransform();
        offset_tf.updR() = r_fb;
        imu_offset->setOffsetTransform(offset_tf);

        if (imu_offset->get_translation() == SimTK::Vec3(0, 0, 0)) {
          auto body = bodies_.at(imu_index);

          if (body) {
            imu_offset->upd_translation() = body->getMassCenter();
          }
        }
      } else {
        auto body = bodies_.at(imu_index);
        SimTK::Vec3 p_fb(0, 0, 0);

        if (body) {
          p_fb = body->getMassCenter();
        }

        imu_offset = new OpenSim::PhysicalOffsetFrame(
            imu_name, *bodies_.at(imu_index), SimTK::Transform(r_fb, p_fb));
        bodies_.at(imu_index)->addComponent(imu_offset);

        // TODO: are these 5 lines useful?
        // Create an IMU object in the model connected to imu_offset
        OpenSim::IMU* model_imu = new OpenSim::IMU();
        model_imu->setName(imu_name);
        model_imu->connectSocket_frame(*imu_offset);
        model_->addModelComponent(model_imu);
      }

      // Add brick representing the IMU if not already present
      if (!model_->hasComponent(imu_name + "_geom_1")) {
        auto* brick = new OpenSim::Brick(SimTK::Vec3(0.023, 0.015, 0.006));
        brick->setColor(SimTK::Orange);
        imu_offset->attachGeometry(brick);
      }
    }
    imu_index++;
  }
}

void hiros::opensim_ik::RTIMUPlacer::visualizeCalibratedModel() {
  if (!model_->getUseVisualizer()) {
    model_->setUseVisualizer(true);
  }

  SimTK::State& s = model_->initSystem();
  s.updTime() = times_.front();

  OpenSim::MarkersReference m_refs{};
  OpenSim::OrientationsReference o_refs(orientations_data_);
  SimTK::Array_<OpenSim::CoordinateReference> coord_refs{};

  OpenSim::InverseKinematicsSolver ik_solver(*model_.get(), m_refs, o_refs,
                                             coord_refs);
  ik_solver.assemble(s);

  model_->updVisualizer().updSimbodyVisualizer().setWindowTitle(
      model_->getName() + " - IMU Placer");
  model_->getVisualizer().show(s);
}
