# ROSBIM - Contributing

Author(s): Tobit Flatscher, [Michael Terzer](mailto:michael.terzer@fraunhofer.it) and [Simone Garbin](mailto:simone.garbin@fraunhofer.it) (July 2022)

## Introduction

ROSBIM consists of two main parts:

- A **library** `BimInterface` that is instantiated as a global variable by the manager and might have different **back-ends** such as the IFC one based on `IfcOpenShell` implementing this functionality for different BIM-files. This library is independent of ROS and might also be used as a standalone.
- Several **plug-ins** that use that library to expose certain functionality to ROS.

Everything around it such as the `RosbimManager` are only wrappers that expose this functionality in a handy way.

## Contributing a ROSBIM plug-in

When adding a new plug-in one has to think about **which functionality should be exposed in library level and which one on plug-in level**. For several different use cases this has been done in the `CodeDesign.md`.

### Adding library level functionality

First implement the corresponding functionality on library level by adding a corresponding **virtual method to the** `**BimInterface**` inside `bim_interface/bim_interface.py`. Follow up by making an **implementation of the virtual method in each individual back-end**.

### Adding the plug-in package

After having added the corresponding library functionality you are read to implement your plug-in as a package: **Create a new package**, either inside the workspace or if the plug-in is consider essential inside the folder `rosbim_plugins`. Currently each plug-in is discovered by its **prefix** `**rosbim_**` and by the fact that it contains a Python file with the same name as the package itself, which contains a class that inherits from `PluginBase` (`bim_interface.plugin_base`). This rudimentary detection mechanism might be changed to a detection mechanism with a `plugin.xml` in the future.

The implementation of the plug-in will have to follow the `PluginBase` class, **implementing an** `**_init**`**,** `**_start**` **and** `**_stop**` **functions** that are responsible for initialization, starting and stopping the plug-in. The back-end can be accessed through the global `**_backend**` **variable** from `bim_interface.backend` and is secured by the `**_backend_lock**` **recursive lock**. For simplicity you can use the `@wrapt.synchronized` decorator with the `_backend_lock` to protect against race conditions, instead of acquiring and releasing the lock manually every time. This might block the changing of a back-end though for a significant time in case a plug-in would be computationally expensive.

**Each plug-in should expose its own functionality via ROS services and topics**. These services are generally initialized in `_init`, started in `_start` and finally stopped in `_stop`.

## Contributing a ROSBIM back-end

If you want to add a new back-end, please insert a new folder into the `bim_interface/backends` folder and create a class that **inherits from** `**BimInterface**` located in `bim_interface.bim_interface`. Each of the **virtual functions** of `BimInterface` will have to be **implemented**.

Then continue to add the back-end to `**bim_interface.backend**`, by including the class there, **adding a new entry to the enum** and creating a new connection between the enum and the corresponding back-end inside the `**switch_backend**` **function**.
