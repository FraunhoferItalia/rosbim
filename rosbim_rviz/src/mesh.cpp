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


#include "mesh.hpp"

#include <OgreAny.h>
#include <OgreColourValue.h>
#include <OgreEntity.h>
#include <OgreMaterialManager.h>
#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreString.h>
#include <OgreTechnique.h>
#include <OgreVector3.h>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_rendering/mesh_loader.hpp>
#include <string>

#include "mesh_selection_handler.hpp"
#include "rviz_rendering/objects/object.hpp"

namespace rosbim_rviz
{

Mesh::Mesh(
  Ogre::SceneManager * scene_manager, rviz_common::DisplayContext * context,
  Ogre::SceneNode * parent_node)
: rviz_rendering::Object{scene_manager},
  context_{context},
  entity_{nullptr},
  material_{},
  color_{Ogre::ColourValue{1.0, 1.0, 1.0, 1.0}}
{
  if (!parent_node) {
    parent_node = scene_manager->getRootSceneNode();
  }

  scene_node_ = parent_node->createChildSceneNode();
  return;
}

Mesh::~Mesh()
{
  reset();
  return;
}

void Mesh::reset()
{
  if (entity_ != nullptr) {
    scene_manager_->destroyEntity(entity_);
    entity_ = nullptr;
  }

  if (!material_.isNull()) {
    Ogre::MaterialManager::getSingleton().remove(material_->getName());
    material_.setNull();
  }
  handler_ = nullptr;
  return;
}

void Mesh::setPosition(Ogre::Vector3 const & position)
{
  scene_node_->setPosition(position);
  return;
}

Ogre::Vector3 const & Mesh::getPosition() { return scene_node_->getPosition(); }

void Mesh::setOrientation(Ogre::Quaternion const & orientation)
{
  scene_node_->setOrientation(orientation);
  return;
}

Ogre::Quaternion const & Mesh::getOrientation() { return scene_node_->getOrientation(); }

void Mesh::setScale(Ogre::Vector3 const & scale)
{
  scene_node_->setScale(scale);
  return;
}

Ogre::Vector3 const & Mesh::getScale() const { return scene_node_->getScale(); }

void Mesh::setColor(float const r, float const g, float const b, float const a)
{
  setColor(Ogre::ColourValue(r, g, b, a));
  return;
}

void Mesh::setColor(Ogre::ColourValue const & color)
{
  color_ = color;
  // material_->setReceiveShadows(false);
  // material_->getTechnique(0)->setLightingEnabled(true);
  material_->getTechnique(0)->setAmbient(color * 0.5);
  material_->getTechnique(0)->setDiffuse(color);

  if (color.a < 0.9998) {
    material_->getTechnique(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    material_->getTechnique(0)->setDepthWriteEnabled(false);
  } else {
    material_->getTechnique(0)->setSceneBlending(Ogre::SBT_REPLACE);
    material_->getTechnique(0)->setDepthWriteEnabled(true);
  }
  entity_->setMaterial(material_);
  return;
}

Ogre::ColourValue Mesh::getColor() const { return color_; }

void Mesh::setUserData(Ogre::Any const & data) { return; }

void Mesh::loadMesh(std::string const & file_name)
{
  scene_node_->setVisible(false);
  reset();

  if (file_name.empty()) {
    return;
  }

  if (!rviz_rendering::loadMeshFromResource(file_name)) {
    RCLCPP_ERROR_STREAM(
      context_->getRosNodeAbstraction().lock()->get_raw_node()->get_logger(),
      "Could not load '" + file_name + "'!");
    return;
  }

  // Modify name with static counter so that we can have more than one mesh with the same name
  // In the future this might be replaced with a simple FileName_Count
  // For this the std::filesystem library should be used
  static int count{0};
  ++count;
  Ogre::String const mesh_name{std::to_string(count) + "_" + file_name};
  entity_ = scene_manager_->createEntity(mesh_name, Ogre::String{file_name});
  scene_node_->attachObject(entity_);

  material_ = Ogre::MaterialManager::getSingleton().create(
    file_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  setColor(color_);

  handler_ = rviz_common::interaction::createSelectionHandler<MeshSelectionHandler>(this, mesh_name, context_);

  handler_->addTrackedObject(entity_);

  scene_node_->setVisible(true);

  return;
}

}  // namespace rosbim_rviz
