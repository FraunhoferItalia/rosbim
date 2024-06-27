# ROSBIM - Package

Author(s): Tobit Flatscher, [Michael Terzer](mailto:michael.terzer@fraunhofer.it) and [Simone Garbin](mailto:simone.garbin@fraunhofer.it) (January 2022)

## General code design

The robotic middleware to be considered for the code design is the [**Robot Operating System (ROS)**](https://www.ros.org/), especially [ROS2.](https://design.ros2.org/)

Common BIM software and tools, such as [IfcOpenShell](https://github.com/IfcOpenShell/IfcOpenShell) (C++, Python), is generally either written in Python due to its simplicity or offer simple interfaces in other languages like [BimServer](https://github.com/opensourceBIM/BIMserver) (SQL, Java). As a BIM library supporting the ISO-standardised IFC format currently there does not seem to be an alternative to IfcOpenShell.

Due to the common availability of wrappers for running code from other languages inside Python (such as [PyBind11](https://github.com/pybind/pybind11) and [BoostBind](https://www.boost.org/doc/libs/1_64_0/libs/bind/doc/html/bind.html)), the lack of need for performance as well as the simplicity of the language and the flexibility of the language, Python was chosen as a weapon of choice for this library.

For the ROS layer **standardised messages** for communication have to be defined. The ROS layer is plug-in based with pre-defined functions for the different plug-ins (`init`, `on_activate` and `on_deactivate`) that will be called in the process of handling the plug-in.

In order to simplify the process all visualization will be done inside ROS tools with [Rviz](http://wiki.ros.org/rviz) and [Rviz plugins](http://docs.ros.org/en/kinetic/api/rviz_plugin_tutorials/html/).
