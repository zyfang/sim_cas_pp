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
#include <boost/make_shared.hpp>

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

	// get scene
	this->scene = _parent->GetScene();

	// set scene manager
	this->sceneManager = _parent->GetScene()->GetManager();

	// read config file
	VisPlugin::ReadConfigFile();

	// get the trajectory
	VisPlugin::GetTraj();

	// draw the trajectory
	VisPlugin::DrawTraj();
}

//////////////////////////////////////////////////
void VisPlugin::ReadConfigFile()
{
	// create the config
	libconfig::Config cfg;

	// read config file
	try
	{
		cfg.readFile("config.cfg");
	}
	catch(const libconfig::FileIOException &fioex)
	{
		std::cerr << "I/O error while reading file." << std::endl;
	}
	catch(const libconfig::ParseException &pex)
	{
		std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine()
		            				  << " - " << pex.getError() << std::endl;
	}

	// get the variables from the config file
	this->dbName = cfg.lookup("vis.db_name").c_str();
	std::cout << "*VisPlugin* - db_name: " << this->dbName << std::endl;

	// set the collection name
	this->collName = cfg.lookup("vis.coll_name").c_str();
	std::cout << "*VisPlugin* - coll_name: " << this->collName << std::endl;

	// set step size
	this->stepSize = cfg.lookup("vis.step_size");
	std::cout << "*VisPlugin* - step_size: " << this->stepSize << std::endl;
}

//////////////////////////////////////////////////
void VisPlugin::GetTraj()
{
	// mongodb connection
	mongo::DBClientConnection conn;

	// connecto to the host
	conn.connect("localhost");

	// query the db
	mongo::BSONObj query = mongo::BSONObj();

	// mongodb projection
	mongo::BSONObj projection = BSON("_id" << 0);

	// query the given collection
	std::auto_ptr<mongo::DBClientCursor> cursor = conn.query(
			this->dbName + "." + this->collName, query, 0, 0, &projection);

	// init prev_ts
	double prev_ts = 0;

	// loop through the results to get the trajectory
	while (cursor->more())
	{
		// get the current bo
		mongo::BSONObj curr_doc = cursor->next();

		const double curr_ts = curr_doc.getField("timestamp").Number();

		// add the pose to the traj if the stepsize is larger than the limit
		if(curr_ts - prev_ts > this->stepSize)
		{
			// set the prev_ts to the current one
			prev_ts = curr_ts;

			// add the timestamps
			this->timestamps.push_back(curr_ts);

			// get the current position
			const math::Vector3 pos = math::Vector3(
					curr_doc.getFieldDotted("pos.x").Number(),
					curr_doc.getFieldDotted("pos.y").Number(),
					curr_doc.getFieldDotted("pos.z").Number());

			// get the current orientation
			const math::Quaternion rot = math::Quaternion(curr_doc.getFieldDotted("rot.x").Number(),
					curr_doc.getFieldDotted("rot.y").Number(),
					curr_doc.getFieldDotted("rot.z").Number());

			// add pose to the traj
			this->poses.push_back(math::Pose(pos, rot));

			// TODo try setters getters?
//			this->traj->poses.push_back(math::Pose(pos, rot));


		}

	}
}

//////////////////////////////////////////////////
void VisPlugin::DrawTraj()
{
	// ogre entities
	std::vector<Ogre::Entity*> entities;

	// ogre scene nodes
	std::vector<Ogre::SceneNode*> scene_nodes;

	// index for unique naming
	unsigned int i = 0;

//	// loop through the traj points
//	for(std::vector<math::Pose>::const_iterator it = this->poses.begin();
//			it != this->poses.end(); ++it)
//	{
//		// stringstream for unique naming
//		std::ostringstream ss;
//		ss << i;
//
//		// create current entity
//		entities.push_back(
//				this->sceneManager->createEntity("TrajEntity" + ss.str(),Ogre::SceneManager::PT_SPHERE));
//
//		// set the color
//		entities.back()->setMaterialName("Gazebo/Red");
//
//		// make it visible
//		entities.back()->setVisible(true);
//
//		// create current scene node
//		scene_nodes.push_back(
//				this->sceneManager->getRootSceneNode()->createChildSceneNode("TrajNode" + ss.str()));
//
//		// set node scale
//		scene_nodes.back()->setScale(0.0001, 0.0001, 0.0001);
//
//		// ogre position
//		const Ogre::Vector3 node_pos = Ogre::Vector3((*it).pos.x, (*it).pos.y, (*it).pos.z);
//
//		// set node positon
//		scene_nodes.back()->setPosition(node_pos);
//
//		// attach entity to node
//		scene_nodes.back()->attachObject(entities.back());
//
//		// make node visible
//		scene_nodes.back()->setVisible(true);
//
//		// increment node nr
//		i++;
//	};

	std::cout << "*VisPlugin* - " << scene_nodes.size() << " markers drawn" << std::endl;

	const math::Vector3 scale = math::Vector3(0.1, 0.1, 0.05);

	// loop through the traj points
	for(std::vector<math::Pose>::const_iterator it = this->poses.begin();
			it != this->poses.end(); ++it)
	{
		this->visArrows.push_back(new VisArrow(
				this->sceneManager,(*it).pos, (*it).rot, scale, "Green", "Blue"));
	}

}









































