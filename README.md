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


## Launch files
**default.launch**
Contains the default values for each parameter

**custom\_configuration\_example.launch**
Contains an example on how to set some parameters of choice


## Usage
Skeleton's marker names must correspond to the virtual markers defined in the model.
Skeleton's link names must correspond to the virtual IMUs defined in the model.

```
roslaunch hiros_opensim_ik_solver custom_configuration_example.launch
```
