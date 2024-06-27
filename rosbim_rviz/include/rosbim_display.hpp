/**
 * \file rosbim_display.hpp
 * \mainpage
 *   The display panel for displaying BIM geometries inside RViz using ROSBIM
 *   Based on http://docs.ros.org/en/melodic/api/rviz_plugin_tutorials/html/display_plugin_tutorial.html
 *
*/

#ifndef ROSBIM_RVIZ_ROSBIM_DISPLAY
#define ROSBIM_RVIZ_ROSBIM_DISPLAY

#include <QObject>
#include <boost/shared_ptr.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbim_export_geometry_plugin/srv/export_geometry_service.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/tf_frame_property.hpp>
#include <string>
#include <vector>

#include "properties/ros_service_property.hpp"
#include "rosbim_visual.hpp"
#include "rviz_common/display.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace rosbim_rviz
{

/**\class RosbimDisplay
   * \brief 
   *   RViz display for a ROSBIM BIM-Model
  */
class RosbimDisplay : public rviz_common::Display
{
  Q_OBJECT
public:
  /**\fn RosbimDisplay
       * \brief
       *   The constructor
      */
  RosbimDisplay();

  /**\fn RosbimDisplay
       * \brief
       *   The destructor
      */
  virtual ~RosbimDisplay();

protected:
  /**\fn onInitialize
       * \brief
       *   Initialize the display
      */
  void onInitialize() override;

  /**\fn reset
       * \brief
       *   Reset the internals of the Mesh to an unintialized state
      */
  void reset() override;

  /**\fn connectToServer
       * \brief
       *   Connect to the ROS service server with the corresponding name
       * 
       * \param[in] service_name
       *   The name of the service that should be connected to
       * \return
       *   Boolean value signaling success (true) or failure (false) of the connection
      */
  bool connectToServer(std::string const & service_name);

  /**\fn callService
       * \brief
       *   Parse the corresponding properties and call the ROS service
       * 
       * \return
       *   The response from the service call
      */
  rosbim_export_geometry_plugin::srv::ExportGeometryService::Response callService();

  /**\fn spawnVisual
       * \brief
       *   Spawn the visual given by the file name and add it to the visuals stored inside the class
       * 
       * \param[in] file_name
       *   Geometry file to be spawned as a mesh marker
      */
  void spawnVisual(std::string const & file_name);

  void waitForFuture(
    std::shared_ptr<
      rclcpp::Client<rosbim_export_geometry_plugin::srv::ExportGeometryService>::FutureAndRequestId>
      future);

  /**\fn clearVisuals
       * \brief
       *   Clear the visuals stored inside the class
      */
  void clearVisuals();

private Q_SLOTS:
  /**\fn updateColorAndAlpha
       * \brief
       *   Update color and transparency of all visuals belonging to this display by parsing the 
       *   corresponding property
      */
  void updateColorAndAlpha();

  /**\fn updateReferenceFrame
       * \brief
       *   Transform the geometry to its corresponding position and orientation by parsing the 
       *   corresponding property
      */
  void updateReferenceFrame();

  /**\fn updateScale
       * \brief
       *   Rescale the geometry by parsing hte corresponding property
      */
  void updateScale();

  // Update visuals by contacting the service server
  void updateVisuals();

protected:
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
  rclcpp::QoS qos_profile;
  rclcpp::Client<rosbim_export_geometry_plugin::srv::ExportGeometryService>::SharedPtr
    service_client_;

  rviz_common::properties::ColorProperty * color_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::FloatProperty * scale_property_;
  rviz_common::properties::TfFrameProperty * frame_property_;
  rosbim_rviz::RosServiceProperty * geometry_export_service_property_;
  rviz_common::properties::StringProperty * filter_property_;
  rviz_common::properties::BoolProperty * is_export_separately_property_;
  std::vector<boost::shared_ptr<RosbimVisual>> visuals_;
  std::mutex mutex_;
  std::thread th;
  std::shared_ptr<rclcpp::Node> local_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> local_executor_;
};

}  // namespace rosbim_rviz

#endif  // ROSBIM_RVIZ_ROSBIM_DISPLAY
