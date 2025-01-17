# Hi-ROS OpenSim IK Solver


## Dependencies
* [OpenSim](https://github.com/opensim-org/opensim-core) (tested with OpenSim 4.3)
* [Hi-ROS skeleton_msgs](https://gitlab.com/hi-ros/skeleton_msgs)


## Patch OpenSim
Before running the ROS node OpenSim must be patched.

Add the following code in OpenSim [InverseKinematicsSolver.h](https://github.com/opensim-org/opensim-core/blob/4.3/OpenSim/Simulation/InverseKinematicsSolver.h):

- after [line 234](https://github.com/opensim-org/opensim-core/blob/4.3/OpenSim/Simulation/InverseKinematicsSolver.h#L234):

```c++
inline void updateMarkersReference(std::shared_ptr<MarkersReference> newMarkersReference)
{
    _markersReference = newMarkersReference;
}
inline void updateOrientationsReference(std::shared_ptr<OrientationsReference> newOrientationsReference)
{
    _orientationsReference = newOrientationsReference;
}

inline void updateMarkersWeight(const SimTK::Real& newMarkersWeight)
{
    _markers_weight = newMarkersWeight;
}
inline void updateOrientationsWeight(const SimTK::Real& newOrientationsWeight)
{
    _orientations_weight = newOrientationsWeight;
}
```

- after [line 269](https://github.com/opensim-org/opensim-core/blob/4.3/OpenSim/Simulation/InverseKinematicsSolver.h#L269):

```c++
// Markers/orientations relative weights
SimTK::Real _markers_weight{1.};
SimTK::Real _orientations_weight{1.};
```

Edit the following lines in OpenSim [InverseKinematicsSolver.cpp](https://github.com/opensim-org/opensim-core/blob/4.3/OpenSim/Simulation/InverseKinematicsSolver.cpp):

- [line 424](https://github.com/opensim-org/opensim-core/blob/4.3/OpenSim/Simulation/InverseKinematicsSolver.cpp#L424):
```c++
updAssembler().adoptAssemblyGoal(condOwner.release(), _markers_weight);
```

- [line 466](https://github.com/opensim-org/opensim-core/blob/4.3/OpenSim/Simulation/InverseKinematicsSolver.cpp#L466):
```c++
updAssembler().adoptAssemblyGoal(condOwner.release(), _orientations_weight);
```


## Usage
Skeleton's marker names must correspond to the virtual markers defined in the model.
Skeleton's link names must correspond to the virtual IMUs defined in the model.

```
ros2 launch hiros_opensim_ik_solver default.launch.py
```


## Citation
Please cite the following paper:
```
M. Guidolin, M. Vanuzzo, S. Michieletto and M. Reggiani, "Enhancing Real-Time Body Pose Estimation in Occluded Environments Through Multimodal Musculoskeletal Modeling," in IEEE Robotics and Automation Letters, vol. 9, no. 12, pp. 10748-10755.
```

Bib citation source:
```bibtex
@ARTICLE{10714023,
  author={Guidolin, Mattia and Vanuzzo, Michael and Michieletto, Stefano and Reggiani, Monica},
  journal={IEEE Robotics and Automation Letters}, 
  title={Enhancing Real-Time Body Pose Estimation in Occluded Environments Through Multimodal Musculoskeletal Modeling}, 
  year={2024},
  volume={9},
  number={12},
  pages={10748-10755},
  keywords={Real-time systems;Accuracy;Optimization;Biological system modeling;Pose estimation;Cameras;Robot sensing systems;Safety;Biomechanics;Tracking;Human detection and tracking;human-robot collaboration;multi-modal perception for HRI;RGB-D perception;sensor fusion},
  doi={10.1109/LRA.2024.3478569}
}
```
