# opensim_ik_solver


## Dependencies
* [opensim-core](https://github.com/opensim-org/opensim-core.git)
* [xsens_mtw_wrapper](https://gitlab.com/hi-ros/xsens_mtw_wrapper)


## Patch OpenSim
Before running the ROS node OpenSim must be patched.
Add the following code after line 205 in OpenSim [InverseKinematicsSolver.h](https://github.com/opensim-org/opensim-core/blob/master/OpenSim/Simulation/InverseKinematicsSolver.h):

```c++
inline void updateMarkersReference(const OpenSim::MarkersReference& newMarkersReference)
{
    _markersReference = newMarkersReference;
}
inline void updateOrientationsReference(const OpenSim::OrientationsReference& newOrientationsReference)
{
    _orientationsReference = newOrientationsReference;
}
```

## Set Xsens sensor labels
Before running the ROS node Xsens [sensor_labels.yaml](https://gitlab.com/hi-ros/xsens_mtw_wrapper/-/blob/master/config/sensor_labels.yaml) must be configured.
Each IMU label must correspond to the IMU name set in the model to be used.


## Launch files
**hiros\_opensim\_ik\_solver\_default.launch**
Contains the default values for each parameter

**custom\_configuration\_example.launch**
Contains an example on how to set some parameters of choice


## Usage
```
roslaunch hiros_xsens_mtw_wrapper custom_configuration_example.launch
roslaunch hiros_opensim_ik_solver custom_configuration_example.launch
```


## Performance
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
