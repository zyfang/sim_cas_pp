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
		this->shaftNode = NULL;
		this->headNode = NULL;
		this->shaftObj = NULL;
		this->headObj = NULL;
	}

	/// \brief Destructor
	public: virtual ~VisArrow()
	{
		delete this->shaftNode;
		delete this->headNode;
		delete this->shaftObj;
		delete this->headObj;
	}

	/// \brief Load
	public: void Load(Ogre::SceneManager* _sceneManager)
	{
		// set the objects
		this->shaftObj = (Ogre::MovableObject*)(
				_sceneManager->createEntity("axis_shaft"));

		this->headObj = (Ogre::MovableObject*)(
				_sceneManager->createEntity("axis_head"));

		// set the obj materials
	    if (dynamic_cast<Ogre::Entity*>(this->shaftObj))
	      ((Ogre::Entity*)shaftObj)->setMaterialName("Gazebo/Blue");

	    if (dynamic_cast<Ogre::Entity*>(this->headObj))
	      ((Ogre::Entity*)headObj)->setMaterialName("Gazebo/Green");

	    // create the scene nodes
	    this->shaftNode = _sceneManager->getRootSceneNode()->createChildSceneNode("shaft_node");
	    this->headNode = _sceneManager->getRootSceneNode()->createChildSceneNode("head_node");

	    // attach objects to the scene nodes
		shaftNode->attachObject(shaftObj);
		headNode->attachObject(headObj);

		// set positions
		shaftNode->setPosition(0, 1, 0.1 + 2);
		headNode->setPosition(0, 1, 0.24 + 2);

		// set visible
		shaftNode->setVisible(true);
		headNode->setVisible(true);
	}

	/// \brief Arrow shaft Ogre movable obj
	private: Ogre::MovableObject *shaftObj;

	/// \brief Arrow head Ogre movable obj
	private: Ogre::MovableObject *headObj;

	/// \brief Arrow shaft Ogre scene node
	private: Ogre::SceneNode* shaftNode;

	/// \brief Arrow head Ogre scene node
	private: Ogre::SceneNode* headNode;

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
	private: VisTraj* traj;

	/// \brief Trajectory pose
	public: std::vector<gazebo::math::Pose> poses;

	/// \brief Trajectory timestamps
	public: std::vector<double> timestamps;

};

}

#endif
