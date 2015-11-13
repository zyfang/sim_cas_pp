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

#include "LogRaw.hh"

using namespace sg_pp;
using namespace gazebo;
using namespace mongo;

//////////////////////////////////////////////////
LogRaw::LogRaw(const gazebo::physics::WorldPtr _world, 
		const std::string _db_name,
        const std::string _coll_name,
        const std::string _connection_name,
        const int _timeoffset)
	: world(_world)
	, dbName(_db_name)
    , collName(_coll_name)
	, connName(_connection_name)
    , TIME_OFFSET(_timeoffset)
{
	// get the world models
	this->models = this->world->GetModels();

	// get the world models
	this->contactManagerPtr = this->world->GetPhysicsEngine()->GetContactManager();

	// get values from the config file
	LogRaw::ReadConfigFile();
}

//////////////////////////////////////////////////
LogRaw::~LogRaw()
{

}

//////////////////////////////////////////////////
void LogRaw::ReadConfigFile()
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

	this->writeAll = cfg.lookup("raw.write_all");
	std::cout << "*LogRaw* - write_all_transf: " << this->writeAll << std::endl;

	// in case thresholds are set for logging the tf
	if (!this->writeAll)
	{
		this->distTh = cfg.lookup("raw.dist_th");
		std::cout << "*LogRaw* - dist_th: " << this->distTh << std::endl;

		this->angleTh = cfg.lookup("raw.angular_th");
		std::cout << "*LogRaw* - angular_th: " << this->angleTh << std::endl;
	}

}

//////////////////////////////////////////////////
void LogRaw::WriteRawData()
{
    // compute simulation time in milliseconds
    const double timestamp_ms = this->world->GetSimTime().Double() +TIME_OFFSET;
    // const double timestamp_ms = this->world->GetSimTime().nsec / 1000000.0 + this->world->GetSimTime().sec * 1000.0;

    // get all the contacts from the physics engine
    const std::vector<physics::Contact*> _contacts = this->contactManagerPtr->GetContacts();

    // document bson object
    BSONObj _doc_bo;

    // bson array model builder
    BSONArrayBuilder _bson_model_arr_builder;

    //////////////////////////////////////////////////
	// loop trough all the models
	for (unsigned int i = 0; i < this->models.size(); i++ )
	{
		// check if model needs to be written to the db
		if(LogRaw::CheckThreshold(this->models.at(i)))
		{
			// get the links vector from the current model
			const physics::Link_V _links = this->models.at(i)->GetLinks();

			// bson array builder
			BSONArrayBuilder _link_arr_builder;

			//////////////////////////////////////////////////
			// loop through the links
			for (unsigned int j = 0; j < _links.size(); j++)
			{
				// get the collisions of the current link
				const physics::Collision_V _collisions = _links.at(j)->GetCollisions();

				// bson array builder
				BSONArrayBuilder _collision_arr_builder;



				//////////////////////////////////////////////////
				// loop through the collisions
				for (unsigned int k = 0; k < _collisions.size(); k++)
				{
					// bson array builder
					BSONArrayBuilder _contacts_arr_builder;

					// TODO weird contact detection errors when running rawTh and raw threads concurently
					// contacts get shifted around in the world, currently leaving contacts out
					// try using contact manager only from PP ?, or not running concurently?

					//////////////////////////////////////////////////
					// loop through all the global contacts to check if they match the collision
					// TODO not the most effective way?
//					for (unsigned int l = 0; l < _contacts.size(); l++)
//					{
//						//std::cout << "\t" << _contacts.at(l)->collision1->GetName() << " --> "
//						//		<< _contacts.at(l)->collision2->GetName() << std::endl;
//
//						// check if the current collision equals the contact collision1
//						if (_collisions.at(k)->GetName() ==
//								_contacts.at(l)->collision1->GetName())
//						{
//							// create BSON contact object with opposite coll: collision2
//							BSONObj _contact_bo = LogRaw::CreateBSONContactObject(
//									_contacts.at(l), _contacts.at(l)->collision2);
//
//							// append collision obj to array
//							_contacts_arr_builder.append(_contact_bo);
//
//						}
//						// if the current collision equals the contact collision2
//						else if(_collisions.at(k)->GetName() ==
//								_contacts.at(l)->collision2->GetName())
//						{
//							// create BSON contact object with the opposite coll: collision1
//							BSONObj _contact_bo = LogRaw::CreateBSONContactObject(
//									_contacts.at(l), _contacts.at(l)->collision1);
//
//							// append collision obj to array
//							_contacts_arr_builder.append(_contact_bo);
//						}
//
//					}

					// create the bson contacts array
					BSONArray _contact_arr = _contacts_arr_builder.arr();

					// collision bson obj
					BSONObj _collision_bo = LogRaw::CreateBSONCollisionObject(_collisions.at(k), _contact_arr);

					// append collision obj to array
					_collision_arr_builder.append(_collision_bo);
				}

				// bson array
				BSONArray _collision_arr = _collision_arr_builder.arr();

				// link bson object
				BSONObj _link_bo = LogRaw::CreateBSONLinkObject(_links.at(j), _collision_arr);

				// append link object to array
				_link_arr_builder.append(_link_bo);
			}

			// create the bson link array
			BSONArray _link_arr = _link_arr_builder.arr();

			// model bson object
			BSONObj _model_bo = LogRaw::CreateBSONModelObject(this->models.at(i), _link_arr);

			// append model object to array
			_bson_model_arr_builder.append(_model_bo);
		}
	}


    // create the bson model array
    BSONArray _bson_model_arr = _bson_model_arr_builder.arr();

    // write to db only if at least one model is present
	if (!_bson_model_arr.isEmpty())
	{
	    // create the document object
	    _doc_bo = BSON("models" << _bson_model_arr << "timestamp" << timestamp_ms);

		// Create scoped connection
		ScopedDbConnection scoped_connection(this->connName);

		// insert document object into the database
		scoped_connection->insert(this->dbName + "." + this->collName + "_raw",	_doc_bo);

		// let the pool know the connection is done
		scoped_connection.done();
	}
}

//////////////////////////////////////////////////
bool LogRaw::CheckThreshold(const gazebo::physics::ModelPtr _model)
{
	// ignore thresholds
	if(this->writeAll)
	{
		// add data to db
		return true;
	}

	// if the model is not yet in the memory, add it and save it's initial data to the db
	if(!modelPoseMemoryMap.count(_model))
	{
		// add pose to the memory
		this->modelPoseMemoryMap[_model] = _model->GetWorldPose();

		// add data to db
		return true;
	}


	// get the current and memory pose of the model
	const math::Pose curr_pose = _model->GetWorldPose();
	const math::Pose memory_pose = this->modelPoseMemoryMap[_model];

	// check for dist thresh
	if (memory_pose.pos.Distance(curr_pose.pos) >= this->distTh)
	{
		// add new pose to the memory
		this->modelPoseMemoryMap[_model] = _model->GetWorldPose();

		// add data to db
		return true;
	}
	// check for angular thresh
	else if (memory_pose.rot.GetAsEuler().Distance(curr_pose.rot.GetAsEuler()) >= this->angleTh)
	{
		// add new pose to the memory
		this->modelPoseMemoryMap[_model] = _model->GetWorldPose();

		// add data to db
		return true;
	}

	// don't write to db
	return false;
}


//////////////////////////////////////////////////
BSONObj LogRaw::CreateBSONContactObject(const physics::Contact* _contact, const physics::Collision* _collision)
{
	return BSON ("name" << _collision->GetName()
			<< "coll_model_name" << _collision->GetParent()->GetParent()->GetName()
			<< "coll_link_name" << _collision->GetParent()->GetName()
			// TODO if all contact points are needed loop through all the values
			// only first contact point is used
			<< "pos" << BSON ("x" << _contact->positions[0].x
							<< "y" << _contact->positions[0].y
							<< "z" << _contact->positions[0].z)
			<< "normal" << BSON ("x" << _contact->normals[0].x
							<< "y" << _contact->normals[0].y
							<< "z" << _contact->normals[0].z));
}

//////////////////////////////////////////////////
BSONObj LogRaw::CreateBSONCollisionObject(const physics::CollisionPtr _collision, const BSONArray _contact_arr)
{
	return 	BSON ("name" << _collision->GetName()
			<< "pos" << BSON(  "x" << _collision->GetWorldPose().pos.x
							<< "y" << _collision->GetWorldPose().pos.y
							<< "z" << _collision->GetWorldPose().pos.z)
			<< "rot" << BSON(  "x" << _collision->GetWorldPose().rot.GetAsEuler().x
							<< "y" << _collision->GetWorldPose().rot.GetAsEuler().y
							<< "z" << _collision->GetWorldPose().rot.GetAsEuler().z)
			<< "bbox" << BSON( "min" << BSON(  "x" << _collision->GetBoundingBox().min.x
											<< "y" << _collision->GetBoundingBox().min.y
											<< "z" << _collision->GetBoundingBox().min.z)
							<< "max" << BSON(  "x" << _collision->GetBoundingBox().max.x
											<< "y" << _collision->GetBoundingBox().max.y
											<< "z" << _collision->GetBoundingBox().max.z))
			<< "contacts" << _contact_arr);
}

//////////////////////////////////////////////////
BSONObj LogRaw::CreateBSONLinkObject(const physics::LinkPtr _link, const BSONArray _collision_arr)
{
	return BSON("name" << _link->GetName()
			<< "pos" << BSON(  "x" << _link->GetWorldPose().pos.x
							<< "y" << _link->GetWorldPose().pos.y
							<< "z" << _link->GetWorldPose().pos.z)
			<< "rot" << BSON(  "x" << _link->GetWorldPose().rot.GetAsEuler().x
							<< "y" << _link->GetWorldPose().rot.GetAsEuler().y
							<< "z" << _link->GetWorldPose().rot.GetAsEuler().z)
			<< "bbox" << BSON( "min" << BSON(  "x" << _link->GetBoundingBox().min.x
											<< "y" << _link->GetBoundingBox().min.y
											<< "z" << _link->GetBoundingBox().min.z)
							<< "max" << BSON(  "x" << _link->GetBoundingBox().max.x
											<< "y" << _link->GetBoundingBox().max.y
											<< "z" << _link->GetBoundingBox().max.z))
			<< "collisions" << _collision_arr);
}

//////////////////////////////////////////////////
BSONObj LogRaw::CreateBSONModelObject(const physics::ModelPtr _model, const BSONArray _link_arr)
{
	return BSON("name" << _model->GetName()
		<< "pos" << BSON(  "x"  << _model->GetWorldPose().pos.x
						<< "y"  << _model->GetWorldPose().pos.y
						<< "z"  << _model->GetWorldPose().pos.z)
		<< "rot" << BSON(  "x"  << _model->GetWorldPose().rot.GetAsEuler().x
						<< "y"  << _model->GetWorldPose().rot.GetAsEuler().y
						<< "z"  << _model->GetWorldPose().rot.GetAsEuler().z)
		<< "bbox" << BSON( "min" << BSON(  "x" << _model->GetBoundingBox().min.x
										<< "y" << _model->GetBoundingBox().min.y
										<< "z" << _model->GetBoundingBox().min.z)
						<< "max" << BSON(  "x" << _model->GetBoundingBox().max.x
										<< "y" << _model->GetBoundingBox().max.y
										<< "z" << _model->GetBoundingBox().max.z))
		<< "links" << _link_arr);
}
