// Copyright 2022-2024 Fraunhofer Italia Research

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Copyright 2022 Fraunhofer Italia Research. All Rights Reserved.

#include "rosbim_display.hpp"

#include <OgreColourValue.h>
#include <OgreVector3.h>
#include <QtCore/qobjectdefs.h>

#include <QColor>
#include <QString>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/tf_frame_property.hpp>
#include <string>
#include <thread>

#include "properties/ros_service_property.hpp"
#include "rosbim_visual.hpp"

namespace rosbim_rviz
{

RosbimDisplay::RosbimDisplay() : rviz_ros_node_{}, qos_profile(5), service_client_{}
{
  color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(255, 255, 255), "Color to draw the acceleration arrows.", this,
    SLOT(updateColorAndAlpha()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1.0, "0.0 is fully transparent, 1.0 is fully opaque.", this,
    SLOT(updateColorAndAlpha()));

  scale_property_ = new rviz_common::properties::FloatProperty(
    "Scale", 0.001, "Change the scale of the geometry.", this, SLOT(updateScale()));

  frame_property_ = new rviz_common::properties::TfFrameProperty(
    "Reference Frame", "map", "The TF frame these axes will use for their origin.", this, nullptr,
    true, SLOT(updateReferenceFrame()));

  geometry_export_service_property_ = new RosServiceProperty(
    "Service", "", "rosbim_export_geometry_plugin/srv/ExportGeometryService",
    "rosbim_export_geometry_plugin::srv::ExportGeometryService service to contact.", this,
    SLOT(updateVisuals()));

  filter_property_ = new rviz_common::properties::StringProperty(
    "Filter", "", "Filter for selecting particular building element types", this,
    SLOT(updateVisuals()));

  is_export_separately_property_ = new rviz_common::properties::BoolProperty(
    "Export separately", false,
    "Export each individual element individually or merge them together to a single model", this,
    SLOT(updateVisuals()));

  return;
}

RosbimDisplay::~RosbimDisplay()
{
  reset();
  return;
}

void RosbimDisplay::onInitialize()
{
  rviz_ros_node_ = context_->getRosNodeAbstraction();
  rviz_common::Display::onInitialize();

  // The properties can only be modified in the initialization routine and can't be already performed in the constructor
  frame_property_->setFrameManager(context_->getFrameManager());
  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);
  scale_property_->setMin(0.0);

  local_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  local_node_ = rclcpp::Node::make_shared("rosbim_rviz_export_geometry_service_client");
  local_executor_->add_node(local_node_);
  return;
}

void RosbimDisplay::reset()
{
  rviz_common::Display::reset();
  clearVisuals();
  return;
}

bool RosbimDisplay::connectToServer(std::string const & service_name)
{
  try {
    service_client_ =
      local_node_->create_client<rosbim_export_geometry_plugin::srv::ExportGeometryService>(
        service_name);
  } catch (std::exception & ex) {
    RCLCPP_ERROR_STREAM(
      local_node_->get_logger(), "Could not connect to service with name '" + service_name + "'!");
    return false;
  }
  return true;
}

void RosbimDisplay::waitForFuture(
  std::shared_ptr<
    rclcpp::Client<rosbim_export_geometry_plugin::srv::ExportGeometryService>::FutureAndRequestId>
    future)
{
  RCLCPP_DEBUG_STREAM(local_node_->get_logger(), "Waiting for service response...");

  std::future_status ready;
  do {
    ready = future->wait_for(std::chrono::milliseconds(500));
    local_executor_->spin_some();
  } while (ready != std::future_status::ready);
  RCLCPP_DEBUG_STREAM(local_node_->get_logger(), "Got service response...");

  clearVisuals();

  RCLCPP_DEBUG_STREAM(
    local_node_->get_logger(), "Attempting to spawn respsonsed geometry_files as visuals!");
  auto response = future->get();
  if (response->geometry_files.size() == 0) {
    RCLCPP_ERROR(
      local_node_->get_logger(), "The service response did not contain any geometry-files!");
    return;
  }

  auto list = response->geometry_files;

  for (auto const item : list) {
    // RViz only accepts either file://absolute_path or package://package_name/relative_path
    // This is why we add file://
    spawnVisual("file://" + item);
  }
  RCLCPP_DEBUG_STREAM(rviz_ros_node_.lock()->get_raw_node()->get_logger(), "Finished...");
}

rosbim_export_geometry_plugin::srv::ExportGeometryService::Response RosbimDisplay::callService()
{
  auto req = std::make_shared<rosbim_export_geometry_plugin::srv::ExportGeometryService::Request>();
  req->geometry_filter = filter_property_->getStdString();
  req->is_export_separately = is_export_separately_property_->getBool();
  req->export_format = rosbim_export_geometry_plugin::srv::ExportGeometryService::Request::DAE;
  rosbim_export_geometry_plugin::srv::ExportGeometryService::Response res;
  RCLCPP_INFO_STREAM(
    local_node_->get_logger(), "Requesting geometries... This might take a while...");
  auto result = std::make_shared<
    rclcpp::Client<rosbim_export_geometry_plugin::srv::ExportGeometryService>::FutureAndRequestId>(
    service_client_->async_send_request(req));

  RosbimDisplay::waitForFuture(std::move(result));

  // todo: async thread to call the service
  // is not working atm because of low level GL library issues with multithreading...
  // th = std::thread([result = std::move(result), this] {
  //   RosbimDisplay::waitForFuture(result);
  //   std::this_thread::sleep_for(std::chrono::seconds(1));
  // });

  //th.detach();
  res.success = true;
  return res;
}

void RosbimDisplay::spawnVisual(std::string const & file_name)
{
  ;
  RCLCPP_DEBUG_STREAM(
    rviz_ros_node_.lock()->get_raw_node()->get_logger(),
    "Spawing visual for file_name: " << file_name);

  auto const visual{
    boost::make_shared<RosbimVisual>(context_->getSceneManager(), context_, scene_node_)};

  visual->loadMesh(file_name);

  Ogre::ColourValue color{color_property_->getOgreColor()};
  float const alpha{alpha_property_->getFloat()};
  color.a = alpha;
  visual->setColor(color);

  float const scale{scale_property_->getFloat()};
  visual->setScale(Ogre::Vector3{Ogre::Real{scale}});
  visuals_.push_back(visual);
  return;
}

void RosbimDisplay::clearVisuals()
{
  RCLCPP_DEBUG_STREAM(local_node_->get_logger(), "Clearing visuals...");
  visuals_.clear();
  return;
}

void RosbimDisplay::updateColorAndAlpha()
{
  Ogre::ColourValue color{color_property_->getOgreColor()};
  float const alpha{alpha_property_->getFloat()};
  color.a = alpha;

  for (auto && v : visuals_) {
    v->setColor(color);
  }
  return;
}

void RosbimDisplay::updateReferenceFrame()
{
  std::string const frame{frame_property_->getStdString()};

  Ogre::Vector3 position{};
  Ogre::Quaternion orientation{};
  rclcpp::Time const time{rviz_ros_node_.lock()->get_raw_node()->get_clock()->now()};
  if (context_->getFrameManager()->getTransform(frame, time, position, orientation)) {
    for (auto && v : visuals_) {
      v->setPosition(position);
      v->setOrientation(orientation);
    }
  } else {
    RCLCPP_ERROR_STREAM(
      rviz_ros_node_.lock()->get_raw_node()->get_logger(),
      "Failed to update reference frame to '" << frame << "'.");
  }
  return;
}

void RosbimDisplay::updateScale()
{
  float const scale{scale_property_->getFloat()};
  for (auto && v : visuals_) {
    v->setScale(Ogre::Vector3{Ogre::Real{scale}});
  }
  return;
}

void RosbimDisplay::updateVisuals()
{
  std::string const service_name{geometry_export_service_property_->getServiceStd()};
  if (service_name.empty()) {
    return;
  }
  bool is_success{connectToServer(service_name)};
  if (!is_success) {
    RCLCPP_ERROR_STREAM(
      local_node_->get_logger(), "Failed to connecto to service '" << service_name << "'.");
    return;
  }

  auto const res{callService()};
  if (!res.success) {
    RCLCPP_ERROR_STREAM(
      local_node_->get_logger(), "Service call to '" << service_name << "' no successful.");
    return;
  }

  return;
}

}  // namespace rosbim_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rosbim_rviz::RosbimDisplay, rviz_common::Display)
