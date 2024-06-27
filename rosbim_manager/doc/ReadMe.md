# rosbim_manager - Package

Author(s): Tobit Flatscher (March 2022)

## Description

This package holds the **plug-in manager** for the `rosbim` package. The plug-in manager is inspired by the way `**rqt**` ([\[1\]](https://github.com/ros-visualization/rqt/blob/kinetic-devel/rqt_gui_py/src/rqt_gui_py/plugin.py) [\[2\]](https://github.com/ros-visualization/rqt/tree/kinetic-devel/rqt_gui/src/rqt_gui)) handles **plug-in discovery** and the services and spawner architecture is inspired by the `ros_control` `controller_manager` ([\[3\]](http://wiki.ros.org/controller_manager)).

Similar to the `ros_control` `controller_manager` it offers the following functionality:

- `load`: load plug-in
- `unload`: unload plug-in
- `start`: start plug-in
- `stop`: stop plug-in
- `spawn`: load and start plug-in
- `kill`: stop and unload all plug-ins
- `list`: list all the loaded plug-ins as well as if it is running or paused

The `spawner` can be used to contact the manager and let it spawn a particular plug-in. Similarly there is also an unspawner available.
