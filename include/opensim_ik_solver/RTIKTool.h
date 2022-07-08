#ifndef hiros_opensim_ik_solver_RTIKTool_h
#define hiros_opensim_ik_solver_RTIKTool_h

// OpenSim dependencies
#include "OpenSim/Simulation/InverseKinematicsSolver.h"
#include "OpenSim/Simulation/Model/Model.h"

// Internal dependencies
#include "opensim_ik_solver/utils.h"

namespace hiros {
  namespace opensim_ik {

    class RTIKTool
    {
    public:
      RTIKTool(const IKToolParameters& t_params);
      RTIKTool(const OpenSim::Model& t_model, const IKToolParameters& t_params);

      virtual ~RTIKTool();

      void setModel(const OpenSim::Model& t_model);
      void enableVisualizer();
      void disableVisualizer();

      bool runSingleFrameIK(const OpenSim::MarkersReference& t_marker_refs,
                            const OpenSim::OrientationsReference& t_orientation_refs = {});
      bool runSingleFrameIK(const OpenSim::OrientationsReference& t_orientation_refs);

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
      void updateReference(const OpenSim::MarkersReference& t_marker_refs,
                           const OpenSim::OrientationsReference& t_orientation_refs = {});
      void updateReference(const OpenSim::OrientationsReference& t_orientation_refs);

      bool runSingleFrameIK();

      void initialize();

      bool m_initialized;

      IKToolParameters m_params;

      std::shared_ptr<OpenSim::MarkersReference> m_marker_refs;
      std::shared_ptr<OpenSim::OrientationsReference> m_orientation_refs;

      std::unique_ptr<OpenSim::InverseKinematicsSolver> m_ik_solver;
      std::unique_ptr<OpenSim::Model> m_model;
      std::unique_ptr<SimTK::State> m_state;
    };

  } // namespace opensim_ik
} // namespace hiros

#endif
