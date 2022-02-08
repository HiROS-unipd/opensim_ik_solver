#ifndef hiros_opensim_ik_solver_PublisherData_h
#define hiros_opensim_ik_solver_PublisherData_h

// ROS dependencies
#include "sensor_msgs/JointState.h"

// OpenSim dependencies
#include "OpenSim/Simulation/OrientationsReference.h"

// Custom external dependencies
#include "hiros_skeleton_msgs/SkeletonGroup.h"

// Internal dependencies
#include "opensim_ik_solver/Queue.h"

namespace hiros {
  namespace opensim_ik {

    struct PublisherData
    {
      sensor_msgs::JointState joint_angles;
      hiros_skeleton_msgs::SkeletonGroup skeleton_group;
    };

    typedef Queue<hiros_skeleton_msgs::SkeletonGroup, PublisherData> SkelGroupToPubDataQueue;
    typedef std::shared_ptr<SkelGroupToPubDataQueue> SkelGroupToPubDataQueuePtr;

  } // namespace opensim_ik
} // namespace hiros

#endif
