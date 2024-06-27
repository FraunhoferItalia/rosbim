# rosbim_manager - Package

Author(s): Tobit Flatscher (2022)

## Description

This package holds the ROS wrapper for around the `bim_interface` library, allowing the user to manage a BIM model with a backend (e.g. IfcOpenShell) and exposing its functionality over dynamically loadable plug-ins.

Similar to `ros_control` there is a manager `rosbim_manager` that can be started with

```
$ ros2 run rosbim_manager rosbim_manager.py --bim_file "/path/to/bim_file"
```

(that then exposes services for loading and unloading a BIM file) and a spawner

```
$ ros2 run rosbim_manager spawner.py --name "plugin_package_name"
```

that allows you to directly spawn a plug-in given by the name of the package where the plug-in is located in.
