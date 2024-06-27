/**
 * \file rosbim_visual.hpp
 * \mainpage
 *   The visual based on the mesh marker for displaying BIM geometries inside RViz using ROSBIM
 *   Based on http://docs.ros.org/en/melodic/api/rviz_plugin_tutorials/html/display_plugin_tutorial.html
 * 
*/

#ifndef ROSBIM_RVIZ_ROSBIM_VISUAL
#define ROSBIM_RVIZ_ROSBIM_VISUAL

#include <OgreColourValue.h>
#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>

#include <boost/shared_ptr.hpp>
#include <rviz_common/display_context.hpp>
#include <string>

#include "mesh.hpp"

namespace rosbim_rviz
{

/**\class RosbimVisual
   * \brief 
   *   RViz visual to be visualized
   *   Largely similar to the mesh class: Might be merged with it or inherit from it in the future
   *   Currently it is the owner of a mesh but all the member functions are very shallow and just 
   *   forward to the implemention inside the mesh class
  */
class RosbimVisual
{
public:
  /**\fn RosbimVisual
       * \brief
       *   The constructor
       *
       * \param[in] scene_manager
       *   Manager for the organization and rendering of a scene
       * \param[in] context
       *   Context of the display widget responsible for creating this mesh
       * \param[in] parent_node
       *   Parent node in the scene graph
      */
  RosbimVisual(
    Ogre::SceneManager * scene_manager, rviz_common::DisplayContext * display_context,
    Ogre::SceneNode * parent_node);

  RosbimVisual() = delete;

  /**\fn ~Mesh
       * \brief
       *   The destructor
      */
  virtual ~RosbimVisual();

  /**\fn setPosition
       * \brief
       *   Setter for the current position of the mesh
       *
       * \param[in] position
       *   The current position of the mesh relative to its parent frame that it should be moved to
      */
  void setPosition(Ogre::Vector3 const & position);

  /**\fn setOrientation
       * \brief
       *   Setter for the current orientation of the mesh
       *
       * \param[in] orientation
       *   The current orientation of the mesh relative to its parent frame that it should be moved to
      */
  void setOrientation(Ogre::Quaternion const & orientation);

  /**\fn setScale
       * \brief
       *   Setter for the scale of the mesh 
       *
       * \param[in] scale
       *   Scale of the mesh compared to its nominal values that the scale should be set to
      */
  void setScale(Ogre::Vector3 const & scale);

  /**\fn setColor
       * \brief
       *   Setter for the color of the mesh, calls the other overload
       *
       * \param[in] r
       *   Normalized red value [0.0, 1.0]
       * \param[in] g
       *   Normalized green value [0.0, 1.0]
       * \param[in] b
       *   Normalized blue value [0.0, 1.0]
       * \param[in] a
       *   Normalized transparency value [0.0, 1.0]
      */
  void setColor(float const r, float const g, float const b, float const a);

  /**\fn setColor
       * \brief
       *   Setter for the color of the mesh
       *
       * \param[in] color
       *   RGBA value of the mesh color
      */
  void setColor(Ogre::ColourValue const & color);

  /**\fn loadMesh
       * \brief
       *   Load a mesh from a given filename
       *
       * \param[in] file_name
       *   The user data that the object should be set to
       *   RViz only accepts either file://absolute_path or package://package_name/relative_path
      */
  void loadMesh(std::string const & file_name);

protected:
  boost::shared_ptr<Mesh> marker_;
  Ogre::SceneManager * scene_manager_;
  Ogre::SceneNode * frame_node_;
};

// Introduce a couple of convenient polymorphic subclasses for the different types of building elements
// from rosbim_export_geometry_plugin/ExportGeometry

}  // namespace rosbim_rviz

#endif  // ROSBIM_RVIZ_ROSBIM_VISUAL
