# Hi-ROS OpenSim IK Solver


## Dependencies
* [OpenSim](https://github.com/opensim-org/opensim-core) (tested with OpenSim 4.2)
* [Hi-ROS skeleton_msgs](https://gitlab.com/hi-ros/skeleton_msgs)


## Patch OpenSim
Before running the ROS node OpenSim must be patched.
Add the following code after line 234 in OpenSim [InverseKinematicsSolver.h](https://github.com/opensim-org/opensim-core/blob/4.2/OpenSim/Simulation/InverseKinematicsSolver.h#L234):

```c++
inline void updateMarkersReference(std::shared_ptr<MarkersReference> newMarkersReference)
{
    _markersReference = newMarkersReference;
}
inline void updateOrientationsReference(std::shared_ptr<OrientationsReference> newOrientationsReference)
{
    _orientationsReference = newOrientationsReference;
}
```


## Launch files
**hiros\_opensim\_ik\_solver\_default.launch**
Contains the default values for each parameter

**custom\_configuration\_example.launch**
Contains an example on how to set some parameters of choice


## Usage
Each hiros_skeleton_msgs/MIMU/imu/header/frame_id must correspond to one of the IMU names set in the model to be used.
```
roslaunch hiros_opensim_ik_solver custom_configuration_example.launch
```


## Single-thread Performance
The system has been tested on an Intel Core i9-9900K CPU @ 3.60GHz, 62.6 GiB memory.
Results (Rajagopal's model, 8 IMUs):

| Solver accuracy | Average time per frame |
| :---:           | :---:                  |
| 1e-01           | 1.6 ms                 |
| 1e-02           | 2.7 ms                 |
| 1e-03           | 3.6 ms                 |
| 1e-04           | 8.7 ms                 |
| 1e-05           | 13.9 ms                |
| 1e-06           | 16.1 ms                |
| 1e-07           | 18.0 ms                |
