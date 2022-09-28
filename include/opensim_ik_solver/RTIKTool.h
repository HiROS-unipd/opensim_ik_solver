#ifndef hiros_opensim_ik_solver_RTIKTool_h
#define hiros_opensim_ik_solver_RTIKTool_h

// OpenSim dependencies
#include "OpenSim/Simulation/InverseKinematicsSolver.h"
#include "OpenSim/Simulation/Model/Model.h"

// Internal dependencies
#include "opensim_ik_solver/utils.h"

namespace hiros {
namespace opensim_ik {

class RTIKTool {
 public:
  RTIKTool(const IKToolParameters& params);
  RTIKTool(const OpenSim::Model& model, const IKToolParameters& params);

  virtual ~RTIKTool();

  void setModel(const OpenSim::Model& model);
  void enableVisualizer();
  void disableVisualizer();

  bool runSingleFrameIK(
      const OpenSim::MarkersReference& marker_refs,
      const OpenSim::OrientationsReference& orientation_refs = {});
  bool runSingleFrameIK(const OpenSim::OrientationsReference& orientation_refs);

  inline SimTK::State getState() const { return *state_.get(); }

  std::string getJointAngleName(int idx) const;
  double getJointAngleValue(int idx, bool in_degrees = false) const;
  double getJointAngleVelocity(int idx, bool in_degrees = false) const;
  std::vector<std::string> getJointAngleNames() const;
  std::vector<double> getJointAngleValues(bool in_degrees = false) const;
  std::vector<double> getJointAngleVelocities(bool in_degrees = false) const;

  std::string getMarkerName(int idx) const;
  SimTK::Vec3 getMarkerPosition(int idx) const;
  SimTK::Vec3 getMarkerVelocity(int idx) const;
  SimTK::Vec3 getMarkerAcceleration(int idx) const;
  std::vector<std::string> getMarkerNames() const;
  std::vector<SimTK::Vec3> getMarkerPositions() const;
  std::vector<SimTK::Vec3> getMarkerVelocities() const;
  std::vector<SimTK::Vec3> getMarkerAccelerations() const;

 private:
  void updateReference(
      const OpenSim::MarkersReference& marker_refs,
      const OpenSim::OrientationsReference& orientation_refs = {});
  void updateReference(const OpenSim::OrientationsReference& orientation_refs);

  bool runSingleFrameIK();

  void initialize();

  bool initialized_{false};

  IKToolParameters params_{};

  std::shared_ptr<OpenSim::MarkersReference> marker_refs_{};
  std::shared_ptr<OpenSim::OrientationsReference> orientation_refs_{};

  std::unique_ptr<OpenSim::InverseKinematicsSolver> ik_solver_{};
  std::unique_ptr<OpenSim::Model> model_{};
  std::unique_ptr<SimTK::State> state_{};
};

}  // namespace opensim_ik
}  // namespace hiros

#endif
