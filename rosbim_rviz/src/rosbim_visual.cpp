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


#include "rosbim_visual.hpp"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_default_plugins/displays/marker/markers/mesh_resource_marker.hpp>

#include "mesh.hpp"

namespace rosbim_rviz
{

RosbimVisual::RosbimVisual(
  Ogre::SceneManager * scene_manager, rviz_common::DisplayContext * context,
  Ogre::SceneNode * parent_node)
: scene_manager_{scene_manager}
{
  frame_node_ = parent_node->createChildSceneNode();
  marker_ = boost::make_shared<Mesh>(scene_manager_, context, frame_node_);
  return;
}

RosbimVisual::~RosbimVisual()
{
  scene_manager_->destroySceneNode(frame_node_);
  return;
}

void RosbimVisual::loadMesh(std::string const & file_name)
{
  marker_->loadMesh(file_name);
  return;
}

void RosbimVisual::setPosition(Ogre::Vector3 const & position)
{
  frame_node_->setPosition(position);
  return;
}

void RosbimVisual::setOrientation(Ogre::Quaternion const & orientation)
{
  frame_node_->setOrientation(orientation);
  return;
}

void RosbimVisual::setScale(Ogre::Vector3 const & scale)
{
  marker_->setScale(scale);
  return;
}

void RosbimVisual::setColor(float const r, float const g, float const b, float const a)
{
  marker_->setColor(r, g, b, a);
  return;
}

void RosbimVisual::setColor(Ogre::ColourValue const & color)
{
  marker_->setColor(color);
  return;
}

}  // namespace rosbim_rviz
