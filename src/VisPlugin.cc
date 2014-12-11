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

#include "VisPlugin.hh"

using namespace sim_games;
using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(VisPlugin)

//////////////////////////////////////////////////
VisPlugin::VisPlugin()
{
}

//////////////////////////////////////////////////
VisPlugin::~VisPlugin()
{
}

//////////////////////////////////////////////////
void VisPlugin::Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf)
{
	std::cout << "******** VIS PLUGIN LOADED *********" << std::endl;

	// set scene manager
	this->sceneManager = _parent->GetScene()->GetManager();

	Ogre::Entity *min_entity;
	Ogre::Entity *max_entity;

	Ogre::SceneNode *min_node;
	Ogre::SceneNode *max_node;


	// set scene node
	min_node = this->sceneManager->getRootSceneNode()->createChildSceneNode(
			"MinSceneNode");

	max_node = this->sceneManager->getRootSceneNode()->createChildSceneNode(
			"MaxSceneNode");

	min_entity = this->sceneManager->createEntity("MinSphere",
			Ogre::SceneManager::PT_SPHERE);

	max_entity = this->sceneManager->createEntity("MaxSphere",
			Ogre::SceneManager::PT_SPHERE);

	min_entity->setMaterialName("Gazebo/Red");

	max_entity->setMaterialName("Gazebo/Red");

	min_entity->setVisible(true);

	max_entity->setVisible(true);

	min_node->setScale(0.001, 0.001, 0.001);

	max_node->setScale(0.001, 0.001, 0.001);

	min_node->setPosition(-0.1, -0.3, 0.0);
	max_node->setPosition(0.9, -1.2, 2.0);

	min_node->attachObject(min_entity);
	max_node->attachObject(max_entity);

	min_node->setVisible(true);
	max_node->setVisible(true);

}






















