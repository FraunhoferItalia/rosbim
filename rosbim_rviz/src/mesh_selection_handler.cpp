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

#include "mesh_selection_handler.hpp"

#include <OgreQuaternion.h>
#include <OgreVector3.h>
#include <stdio.h>
#include <stdlib.h>

#include <QColor>
#include <QString>
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/interaction/forwards.hpp>
#include <rviz_common/interaction/selection_handler.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/quaternion_property.hpp>
#include <rviz_common/properties/vector_property.hpp>
#include <string>

namespace rosbim_rviz
{

MeshSelectionHandler::MeshSelectionHandler(
  Mesh * mesh, std::string const & name, rviz_common::DisplayContext * context)
: SelectionHandler{context}, mesh_{mesh}, name_{name}
{
  return;
}

MeshSelectionHandler::~MeshSelectionHandler() { return; }

void MeshSelectionHandler::createProperties(
  rviz_common::interaction::Picked const & /*obj*/,
  rviz_common::properties::Property * parent_property)
{
  rviz_common::properties::Property * group = new rviz_common::properties::Property(
    QString::fromStdString(name_), QVariant(), "", parent_property);
  properties_.push_back(group);

  // The individual properties to be displayed
  position_property_ =
    new rviz_common::properties::VectorProperty("Position", getPosition(), "", group);
  position_property_->setReadOnly(true);

  orientation_property_ =
    new rviz_common::properties::QuaternionProperty("Orientation", getOrientation(), "", group);
  orientation_property_->setReadOnly(true);

  scale_property_ = new rviz_common::properties::VectorProperty("Scale", getScale(), "", group);
  scale_property_->childAt(0)->setName("Length");
  scale_property_->childAt(1)->setName("Width");
  scale_property_->childAt(2)->setName("Height");
  scale_property_->setReadOnly(true);

  color_property_ = new rviz_common::properties::ColorProperty("Color", getColor(), "", group);
  color_property_->setReadOnly(true);

  group->expand();
  return;
}

void MeshSelectionHandler::updateProperties()
{
  // Update the properties with corresponding function calls
  position_property_->setVector(getPosition());
  orientation_property_->setQuaternion(getOrientation());
  scale_property_->setVector(getScale());
  color_property_->setColor(getColor());
  return;
}

Ogre::Vector3 MeshSelectionHandler::getPosition()
{
  // extract file name
  std::string position_path = name_;
  std::string delimiter = "//";

  size_t pos = 0;
  std::string token;

  while ((pos = position_path.find(delimiter)) != std::string::npos) {
    token = position_path.substr(0, pos);
    position_path.erase(0, pos + delimiter.length());
  }

  delimiter = ".";

  while ((pos = position_path.find(delimiter)) != std::string::npos) {
    position_path = position_path.substr(0, pos);
  }

  std::fstream newfile;
  newfile.open(position_path + ".txt", std::ios::in);

  float pos_array[3] = {};

  if (newfile.is_open()) {
    std::string s;
    while (getline(newfile, s)) {  //read data from file object and put it into string.
      std::string delimiter = ", ";
      size_t pos = 0;
      std::string token;
      int i = 0;

      while ((pos = s.find(delimiter)) != std::string::npos) {
        token = s.substr(0, pos);
        pos_array[i] = std::stof(token);
        s.erase(0, pos + delimiter.length());
        i++;
      }
      pos_array[2] = std::stof(s);
    }
    newfile.close();
  }

  const Ogre::Vector3 position = Ogre::Vector3(pos_array[0], pos_array[1], pos_array[2]);
  return position;
}

Ogre::Quaternion MeshSelectionHandler::getOrientation() { return mesh_->getOrientation(); }

Ogre::Vector3 MeshSelectionHandler::getScale() const { return mesh_->getScale(); }

QColor MeshSelectionHandler::getColor() const
{
  // Convert Ogre colour to QColor by converting the the value from a floating point value in the
  // range 0.0..1.0 to an integer in the interval 0..255
  Ogre::ColourValue const color{mesh_->getColor()};
  return QColor(
    static_cast<int>(color.r * 255), static_cast<int>(color.g * 255),
    static_cast<int>(color.b * 255), static_cast<int>(color.a * 255));
}

}  // namespace rosbim_rviz
