#ifndef hiros_opensim_ik_solver_RTIMUPlacer_h
#define hiros_opensim_ik_solver_RTIMUPlacer_h

// OpenSim dependencies
#include "OpenSim/Simulation/Model/Model.h"
#include "OpenSim/Simulation/OrientationsReference.h"

namespace hiros {
  namespace opensim_ik {

    class RTIMUPlacer
    {
    public:
      RTIMUPlacer(const std::string& t_model_file_path = "",
                  const SimTK::Rotation& t_sensor_to_opensim = SimTK::Rotation());

      virtual ~RTIMUPlacer();

      void setModel(const OpenSim::Model& t_model);
      void performHeadingCorrection(const std::string& t_base_imu_label, const std::string& t_base_heading_axis);
      void enableVisualizer();
      void disableVisualizer();

      bool runCalibration(const OpenSim::TimeSeriesTable_<SimTK::Quaternion>& t_orientations_table);

      OpenSim::Model& getCalibratedModel() const;

    private:
      void setDirectionOnImu(const std::string& t_base_heading_axis);

      void updateOrientationsTable(const OpenSim::TimeSeriesTable_<SimTK::Quaternion>& t_orientations_table);
      bool runCalibration();

      void initialize();
      void applyHeadingCorrection();
      void computeTransforms();
      void computeOffsets();
      void visualizeCalibratedModel();

      bool m_initialized;
      bool m_calibrated;
      std::string m_model_file_path;
      SimTK::Rotation m_sensor_to_opensim;
      bool m_perform_heading_correction;
      std::string m_base_imu_label;
      std::string m_base_heading_axis;
      bool m_use_visualizer;

      SimTK::CoordinateDirection m_direction_on_imu;
      std::vector<std::string> m_imu_labels;
      std::vector<double> m_times;
      SimTK::RowVector_<SimTK::Rotation> m_rotations;
      std::vector<OpenSim::Body*> m_bodies;
      std::map<std::string, SimTK::Rotation> m_imu_bodies_in_ground;

      std::unique_ptr<OpenSim::TimeSeriesTable_<SimTK::Quaternion>> m_orientations_table;
      OpenSim::TimeSeriesTable_<SimTK::Rotation> m_orientations_data;

      std::unique_ptr<SimTK::State> m_state;
      std::unique_ptr<OpenSim::Model> m_model;
    };

  } // namespace opensim_ik
} // namespace hiros

#endif
