# ROSBIM

Authors: Tobit Flatscher, [Michael Terzer](michael.terzer@fraunhofer.it) Fraunhofer Italia 2022-2024

ROSBIM is an interface from ROS 2 to Building Information Modeling (BIM). Explore in depth every directory/package to get to know more. Especially the metapackage  
ROSBIM contains ReadMe's about the code design and contribution-guidelines.

# Installation and usage

This README contains the few basic steps to install and run an example of ROSBIM.

## Installation

The repository comes with a docker setup that is divided into two containers. The base container contains installations of IfcOpenShell, OpenCollada, OCCT and pythonocc-core from source. The compilation of these repositories is time intensive wherefore the prebuild image can be pulled from the dockerhub here.

```
docker compose -f docker/docker-compose-app.yml up
```

### Run an example

The repository comes with an example setup that launches the rosbim_manager, two rosbim plugins, a map_server and an RViz2 instance.

```
colcon build && source install/setup.bash && ros2 launch rosbim_example_models crane_hall_v10.launch.py
```

### Licence

ROSBIM is licensed under the terms of the Apache License 2.0. The project has recieved financial support by the Horizon 2020 EU Project [CONCERT](https://concertproject.eu/).

### Citation

```
@article{Terzer2024,
 title = {Facilitated Construction Robot Programming based on Building Information Modelling},
 author = {Terzer, Michael and Flatscher, Tobit and Magri, Marco and Garbin, Simone and Emig, Julius and Giusti, Andrea},
 journal = {10th International Conference on Control, Decision and Information Technologies},
 year = {2024},
 note = {to be published},
 publisher={IEEE}
}
```
