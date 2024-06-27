/**
 * \file mesh.hpp
 * \mainpage
 *   A mesh to be visualized inside RViz
 *   Largely based on https://github.com/ros-visualization/rviz/blob/noetic-devel/src/rviz/default_plugin/markers/mesh_resource_marker.cpp
 *
*/

#ifndef ROSBIM_RVIZ_MESH
#define ROSBIM_RVIZ_MESH

#include <OgreAny.h>
#include <OgreColourValue.h>
#include <OgreEntity.h>
#include <OgreMaterial.h>
#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>

#include <QCursor>
#include <boost/shared_ptr.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_rendering/objects/object.hpp>
#include <set>
#include <string>

#include "mesh.hpp"

namespace rosbim_rviz
{

class
  MeshSelectionHandler;  // Forward-declaration of MeshSelectionHandler required for this header to compile

/**\class Mesh
   * \brief 
   *   Class encapsulating a mesh (similar to a mesh marker)
  */
class Mesh : public rviz_rendering::Object
{
public:
  /**\fn Mesh
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
  Mesh(
    Ogre::SceneManager * scene_manager, rviz_common::DisplayContext * context,
    Ogre::SceneNode * parent_node = nullptr);

  /**\fn ~Mesh
       * \brief
       *   The destructor
      */
  virtual ~Mesh();

  /**\fn reset
       * \brief
       *   Reset the internals of the Mesh to an unintialized state
      */
  void reset();

  /**\fn setPosition
       * \brief
       *   Setter for the current position of the mesh
       *
       * \param[in] position
       *   The current position of the mesh relative to its parent frame that it should be moved to
      */
  void setPosition(Ogre::Vector3 const & position) override;

  /**\fn getPosition
       * \brief
       *   Getter for the current position of the mesh
       *
       * \return
       *   The current position of the mesh relative to its parent frame
      */
  Ogre::Vector3 const & getPosition() override;

  /**\fn setOrientation
       * \brief
       *   Setter for the current orientation of the mesh
       *
       * \param[in] orientation
       *   The current orientation of the mesh relative to its parent frame that it should be moved to
      */
  void setOrientation(Ogre::Quaternion const & orientation) override;

  /**\fn getOrientation
       * \brief
       *   Getter for the current orientation of the mesh
       *
       * \return
       *   The current orientation of the mesh relative to its parent frame
      */
  Ogre::Quaternion const & getOrientation() override;

  /**\fn setScale
       * \brief
       *   Setter for the scale of the mesh 
       *
       * \param[in] scale
       *   Scale of the mesh compared to its nominal values that the scale should be set to
      */
  void setScale(Ogre::Vector3 const & scale) override;

  /**\fn getScale
       * \brief
       *   Getter for the scale of the mesh 
       *
       * \return
       *   Current scale of the mesh compared to its nominal values
      */
  Ogre::Vector3 const & getScale() const;

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

  /**\fn getColor
       * \brief
       *   Getter for the color of the mesh
       *
       * \return
       *   RGBA value of the mesh color
      */
  Ogre::ColourValue getColor() const;

  /**\fn setUserData
       * \brief
       *   Setter for user data of this object
       *
       * \param[in] data
       *   The user data that the object should be set to
      */
  void setUserData(Ogre::Any const & data) override;

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
  Ogre::SceneNode * scene_node_;
  rviz_common::DisplayContext * context_;
  Ogre::Entity * entity_;
  Ogre::MaterialPtr material_;
  Ogre::ColourValue color_;
  std::shared_ptr<MeshSelectionHandler> handler_;
};

}  // namespace rosbim_rviz

#endif  // ROSBIM_RVIZ_MESH
