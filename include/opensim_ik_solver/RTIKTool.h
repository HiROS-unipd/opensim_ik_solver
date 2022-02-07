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

      inline SimTK::State getState() const { return *m_state.get(); }

      std::string getJointAngleName(int t_idx) const;
      double getJointAnglePosition(int t_idx, bool t_in_degrees = false) const;
      double getJointAngleVelocity(int t_idx, bool t_in_degrees = false) const;
      std::vector<std::string> getJointAngleNames() const;
      std::vector<double> getJointAnglePositions(bool t_in_degrees = false) const;
      std::vector<double> getJointAngleVelocities(bool t_in_degrees = false) const;

      std::string getMarkerName(int t_idx) const;
      SimTK::Vec3 getMarkerPosition(int t_idx) const;
      SimTK::Vec3 getMarkerVelocity(int t_idx) const;
      SimTK::Vec3 getMarkerAcceleration(int t_idx) const;
      std::vector<std::string> getMarkerNames() const;
      std::vector<SimTK::Vec3> getMarkerPositions() const;
      std::vector<SimTK::Vec3> getMarkerVelocities() const;
      std::vector<SimTK::Vec3> getMarkerAccelerations() const;

    private:
      void updateOrientationsReference(const OpenSim::OrientationsReference& t_orientation_refs);
      void updateOrientationsReference(const OpenSim::TimeSeriesTable_<SimTK::Rotation_<double>>& t_orientations,
                                       const OpenSim::Set<OpenSim::OrientationWeight>* t_weights = nullptr);

      bool runSingleFrameIK();

      void initialize();

      bool m_initialized;
      std::unique_ptr<OpenSim::Model> m_model;

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
