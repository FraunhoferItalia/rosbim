# bim_interface - Package

Author(s): Tobit Flatscher (March 2022)

This is a markdown document and is best viewed in a dedicated markdown editor such as [Typora](https://typora.io/).



## Description

This package holds the individual BIM interfaces for ROSBIM, such as `ifcopenshell` interface based on the IfcOpenShell package. This code is designed in such a way that it can also be used independently without ROS as a wrapper library with a standardised interface. A ROS wrapper is provided by the `rosbim_manager` package.

The mechanism of loading back-ends is designed similar to how the Python **`matplotlib`** handles its **backends** ([[1]](https://github.com/matplotlib/matplotlib/tree/main/lib/matplotlib/backends) [[2]](https://github.com/matplotlib/matplotlib/blob/main/lib/matplotlib/backend_bases.py) [[3]](https://github.com/matplotlib/matplotlib/blob/main/lib/matplotlib/backend_managers.py) and in particular [[4]](https://github.com/matplotlib/matplotlib/blob/main/lib/matplotlib/backend_bases.py#L73:L88) and [[5]](https://github.com/matplotlib/matplotlib/blob/main/lib/matplotlib/backend_bases.py#L145)) with a simple global variable and a lock that protects it in case of multi-threaded access.

```
src/bim_interface/
├── backend.py        # Holds the shared backend variable and its lock for safe multi-threading
├── bim_interface.py  # Holds the pure abstract BimInterface class for any backend implementation
├── mesh_format.py    # Holds an enumeration array with different mesh formats
├── plugin_base.py    # Holds the pure abstract base class for any plugin
├── plugin_manager.py # Holds the plug-in manager that manages the lifecycle of each individual plug-in
└── backends/
    └── ifcopenshell/
        ├── export_geometry_impl.py # Implementation class for the ExportGeometry plug-in
        ├── export_map_impl.py      # Implementation class for the ExportMap plug-in
        └── ifcopenshell.py         # The IfcOpenShell-based backend
```

