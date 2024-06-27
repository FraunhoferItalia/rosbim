# ROSBIM Export Map Plugin

Author: [Michael Terzer](michael.terzer@fraunhofer.it) Fraunhofer Italia 2024

This plugin can be used to export an Occupancy Grid Map from a given BIM file in IFC format. 

### Spawning the plugin

The `spawner.py` from the rosbim_manager can be used to load and start the plugin. The status of the plugin should hence be RUNNING after calling following command:

```
ros2 run rosbim_manager spawner.py --name rosbim_export_map_plugin
```

### Calling the service

In order to export an Occupancy Grid Map call the service:

```
ros2 service call /export_map_plugin_node/export_map rosbim_export_map_plugin/srv/ExportMapService "resolution: 0.01
section_heights: [0.1]
```

The result must be the path to the .png and .yaml file of the exported map.
