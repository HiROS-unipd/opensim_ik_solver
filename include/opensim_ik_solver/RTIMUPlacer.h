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

    class RTIMUPlacer
    {
    public:
      RTIMUPlacer(const IMUPlacerParameters& t_params);
      RTIMUPlacer(const OpenSim::Model& t_model, const IMUPlacerParameters& t_params);

      virtual ~RTIMUPlacer();

      void setModel(const OpenSim::Model& t_model);
      void initializeHeadingCorrection();
      void enableVisualizer();
      void disableVisualizer();

      inline SimTK::Vec3 getHeadingRotVec() const { return m_heading_rot_vec; }

      bool runCalibration(const OpenSim::TimeSeriesTable_<SimTK::Quaternion>& t_orientations_table,
                          const OpenSim::MarkersReference& t_markers_reference = {});

      OpenSim::Model& getCalibratedModel() const;
      void saveModel(const std::string& t_calibrated_model_path);
      void saveModel();

    private:
      void setDirectionOnImu();

      void updateOrientationsTable(const OpenSim::TimeSeriesTable_<SimTK::Quaternion>& t_orientations_table);
      void updateMarkersReference(const OpenSim::MarkersReference& t_markers_reference);
      bool runCalibration();

      void initialize();
      bool applyHeadingCorrection();
      void computeTransforms();
      void computeOffsets();
      void visualizeCalibratedModel();

      bool m_initialized;
      bool m_calibrated;

      IMUPlacerParameters m_params;

      SimTK::CoordinateDirection m_direction_on_imu;
      std::vector<std::string> m_imu_labels;
      std::vector<double> m_times;
      SimTK::RowVector_<SimTK::Rotation> m_rotations;
      std::vector<OpenSim::Body*> m_bodies;
      std::map<std::string, SimTK::Rotation> m_imu_bodies_in_ground;

      std::unique_ptr<OpenSim::MarkersReference> m_markers_reference;
      std::unique_ptr<OpenSim::TimeSeriesTable_<SimTK::Quaternion>> m_orientations_table;
      OpenSim::TimeSeriesTable_<SimTK::Rotation> m_orientations_data;

      std::unique_ptr<hiros::opensim_ik::RTIKTool> m_rt_ik_tool;

      SimTK::Vec3 m_heading_rot_vec;

      std::unique_ptr<SimTK::State> m_state;
      std::unique_ptr<OpenSim::Model> m_model;
    };

  } // namespace opensim_ik
} // namespace hiros

#endif
