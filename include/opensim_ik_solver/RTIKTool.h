#ifndef hiros_opensim_ik_solver_RTIKTool_h
#define hiros_opensim_ik_solver_RTIKTool_h

// OpenSim dependencies
#include "OpenSim/Simulation/InverseKinematicsSolver.h"
#include "OpenSim/Simulation/Model/Model.h"

namespace hiros {
  namespace opensim_ik {

    class RTIKTool
    {
    public:
      RTIKTool(const double& t_accuracy = 1e-4, const SimTK::Rotation& t_sensor_to_opensim = SimTK::Rotation());
      RTIKTool(const OpenSim::Model& t_model,
               const double& t_accuracy = 1e-4,
               const SimTK::Rotation& t_sensor_to_opensim = SimTK::Rotation());

      virtual ~RTIKTool();

      void setModel(const OpenSim::Model& t_model);
      void enableVisualizer();

      bool runSingleFrameIK(const OpenSim::OrientationsReference& t_orientation_refs);
      bool runSingleFrameIK(const OpenSim::TimeSeriesTable_<SimTK::Rotation_<double>>& t_orientations,
                            const OpenSim::Set<OpenSim::OrientationWeight>* t_weights = nullptr);

      inline SimTK::State getState() { return *m_state.get(); }
      double getJointPosition(const std::string& t_jointName, int t_idx = -1, bool t_in_degrees = false);
      double getJointVelocity(const std::string& t_jointName, int t_idx = -1, bool t_in_degrees = false);
      std::vector<double> getJointPositions(bool t_in_degrees = false);
      std::vector<double> getJointVelocities(bool t_in_degrees = false);

    private:
      void updateOrientationsReference(const OpenSim::OrientationsReference& t_orientation_refs);
      void updateOrientationsReference(const OpenSim::TimeSeriesTable_<SimTK::Rotation_<double>>& t_orientations,
                                       const OpenSim::Set<OpenSim::OrientationWeight>* t_weights = nullptr);

      bool runSingleFrameIK();

      void initialize();

      bool m_initialized;
      std::unique_ptr<OpenSim::Model> m_model;

      OpenSim::Array<std::string> m_coordinate_names;
      SimTK::Rotation m_sensor_to_opensim;
      double m_accuracy;
      bool m_use_visualizer;

      std::shared_ptr<OpenSim::OrientationsReference> m_orientation_refs;

      std::unique_ptr<OpenSim::InverseKinematicsSolver> m_ik_solver;
      std::unique_ptr<SimTK::State> m_state;
    };

  } // namespace opensim_ik
} // namespace hiros

#endif