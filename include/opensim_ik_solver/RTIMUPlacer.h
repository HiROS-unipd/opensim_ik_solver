#ifndef hiros_opensim_ik_solver_RTIMUPlacer_h
#define hiros_opensim_ik_solver_RTIMUPlacer_h

// OpenSim dependencies
#include "OpenSim/Simulation/Model/Model.h"
#include "OpenSim/Simulation/OrientationsReference.h"

// Internal dependencies
#include "opensim_ik_solver/RTIKTool.h"
#include "opensim_ik_solver/utils.h"

namespace hiros {
namespace opensim_ik {

class RTIMUPlacer {
 public:
  RTIMUPlacer(const IMUPlacerParameters& params);
  RTIMUPlacer(const OpenSim::Model& model, const IMUPlacerParameters& params);

  virtual ~RTIMUPlacer();

  void setModel(const OpenSim::Model& model);
  void initializeHeadingCorrection();
  void enableVisualizer();
  void disableVisualizer();

  inline SimTK::Vec3 getHeadingRotVec() const { return heading_rot_vec_; }

  bool runCalibration(
      const OpenSim::TimeSeriesTable_<SimTK::Quaternion>& orientations_table,
      const OpenSim::MarkersReference& markers_reference = {});

  OpenSim::Model& getCalibratedModel() const;
  void saveModel(const std::string& calibrated_model_path);
  void saveModel();

 private:
  void setDirectionOnImu();

  void updateOrientationsTable(
      const OpenSim::TimeSeriesTable_<SimTK::Quaternion>& orientations_table);
  void updateMarkersReference(
      const OpenSim::MarkersReference& markers_reference);
  bool runCalibration();

  void initialize();
  bool applyHeadingCorrection();
  void computeTransforms();
  void computeOffsets();
  void visualizeCalibratedModel();

  bool initialized_{false};
  bool calibrated_{false};

  IMUPlacerParameters params_{};

  SimTK::CoordinateDirection direction_on_imu_{SimTK::ZAxis};
  std::vector<std::string> imu_labels_{};
  std::vector<double> times_{};
  SimTK::RowVector_<SimTK::Rotation> rotations_{};
  std::vector<OpenSim::Body*> bodies_{};
  std::map<std::string, SimTK::Rotation> imu_bodies_in_ground_{};

  std::unique_ptr<OpenSim::MarkersReference> markers_reference_{};
  std::unique_ptr<OpenSim::TimeSeriesTable_<SimTK::Quaternion>>
      orientations_table_{};
  OpenSim::TimeSeriesTable_<SimTK::Rotation> orientations_data_{};

  std::unique_ptr<hiros::opensim_ik::RTIKTool> rt_ik_tool_{};

  SimTK::Vec3 heading_rot_vec_{};

  std::unique_ptr<SimTK::State> state_{};
  std::unique_ptr<OpenSim::Model> model_{};
};

}  // namespace opensim_ik
}  // namespace hiros

#endif
