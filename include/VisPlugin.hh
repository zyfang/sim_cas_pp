/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Andrei Haidu, Institute for Artificial Intelligence,
 *  Universität Bremen.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Institute for Artificial Intelligence,
 *     Universität Bremen, nor the names of its contributors may be
 *     used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef VIS_PLUGIN_HH
#define VIS_PLUGIN_HH

#include "gazebo/gazebo.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/rendering/rendering.hh"
#include "mongo/client/dbclient.h"
#include <libconfig.h++>

namespace sim_games
{
/// \brief class VisPlugin
class VisTraj
{
	/// \brief Constructor
	public: VisTraj()
	{

	}

	/// \brief Destructor
	public: virtual ~VisTraj()
	{

	}

	/// \brief Trajectory pose
	public: std::vector<gazebo::math::Pose> poses;

	/// \brief Trajectory timestamps
	public: std::vector<double> timestamps;

	// TODO add entities scene nodes as well here ?
};

/// \brief class VisArrow
class VisArrow
{
	/// \brief Constructor
	public: VisArrow()
	{
		this->arrowNode = NULL;
	}

	/// \brief Constructor
	public: VisArrow(Ogre::SceneManager* _sceneManager,
			const gazebo::math::Vector3 _position,
			const gazebo::math::Quaternion _orientation,
			const gazebo::math::Vector3 _scale,
			const std::string _head_color,
			const std::string _shaft_color)
	{
	    // init the arrow scene node
	    this->arrowNode = _sceneManager->getRootSceneNode()->createChildSceneNode();

	    // arrow head mesh from the gazebo paths
	    Ogre::Entity* head_entity = _sceneManager->createEntity("axis_head");

	    // arrow shaft mesh from the gazebo paths
	    Ogre::Entity* shaft_entity = _sceneManager->createEntity("axis_shaft");

	    // attach objects to the scene nodes
	    this->arrowNode->attachObject(shaft_entity);

	    // offset node for the arrow head, child of the arrow node
	    Ogre::SceneNode* offset_node = this->arrowNode->createChildSceneNode();

	    // attach head entity to the offseted node
	    offset_node->attachObject(head_entity);

	    // offset the node
	    offset_node->setPosition(0,0,0.1);

	    // set materials of the entities
	    head_entity->setMaterialName("Gazebo/" + _head_color);
	    shaft_entity->setMaterialName("Gazebo/" + _shaft_color);

	    // set positions
	    this->SetPosition(_position);

		// set orientation
	    this->SetOrientation(_orientation);

	    // set the arrow scale
	    this->SetScale(_scale);

		// set visible
	    this->SetVisible(true);
	}

	/// \brief Destructor
	public: virtual ~VisArrow()
	{
		delete this->arrowNode;
	}

	/// \brief Set the 3d scale of the arrow
	public: void SetScale(const gazebo::math::Vector3 _scale)
	{
		this->arrowNode->setScale(_scale.x, _scale.y, _scale.z);
	}

	/// \brief Set position of the arrow
	public: void SetPosition(const gazebo::math::Vector3 _pos)
	{
		this->arrowNode->setPosition(_pos.x, _pos.y, _pos.z);
	}

	/// \brief Set the orientation of the arrow
	public: void SetOrientation(const gazebo::math::Quaternion _quat)
	{
		this->arrowNode->setOrientation(_quat.w, _quat.x, _quat.y, _quat.z);
	}

	/// \brief Set position of the arrow
	public: void SetVisible(const bool _flag)
	{
		this->arrowNode->setVisible(_flag);
	}


	/// \brief Arrow shaft Ogre scene node
	private: Ogre::SceneNode* arrowNode;
};


/// \brief class VisPlugin
class VisPlugin : public gazebo::VisualPlugin
{
	/// \brief Constructor
	public: VisPlugin();

	/// \brief Destructor
	public: virtual ~VisPlugin();

	/// \brief Load plugin
	protected: virtual void Load(gazebo::rendering::VisualPtr _parent, sdf::ElementPtr _sdf);

	/// \brief Load config file
	private: void ReadConfigFile();

	/// \brief Get traj from mongo
	private: void GetTraj();

	/// \brief Draw the trajectory
	private: void DrawTraj();

	/// \brief Ogre scene node.
	public: Ogre::SceneNode *sceneNode;

	/// \brief Ogre entity.
	public: Ogre::Entity *entity;

	/// \brief Ogre scene manager.
	public: Ogre::SceneManager *sceneManager;

	/// \brief Ogre scene
	public: gazebo::rendering::ScenePtr scene;

	/// \brief Database name
	private: std::string dbName;

	/// \brief Db collection name
	private: std::string collName;

	/// \brief Db collection name
	private: double stepSize;

	// TODO use as diff class, see issue with push back
	/// \brief Trajectory
//	private: VisTraj* traj;

	/// \brief Trajectory pose
	private: std::vector<gazebo::math::Pose> poses;

	/// \brief Trajectory timestamps
	private: std::vector<double> timestamps;


	// todo super class with vis
	/// \brief Trajectory timestamps
	private: std::vector<VisArrow*> visArrows;


};

}

#endif
