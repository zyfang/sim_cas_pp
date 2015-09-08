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

#include "LogTF.hh"

using namespace sg_pp;
using namespace gazebo;
using namespace mongo;

#define TIME_OFFSET 100000

//////////////////////////////////////////////////
LogTF::LogTF(const gazebo::physics::WorldPtr _world,
		const std::string _db_name,
		const std::string _coll_name,
		int _suffix,
		const std::string _connection_name)
	: world(_world)
	, dbName(_db_name)
	, collName(_coll_name)
	, connName(_connection_name)
{
	// get the world models
	this->models = this->world->GetModels();

	// init tf message seq count
	this->tfSeq = 0;

	// get values from the config file
	LogTF::ReadConfigFile();

	// TODO for adding time offset to the simulation times
	this->suffixTime = _suffix;
}

//////////////////////////////////////////////////
LogTF::~LogTF()
{

}

//////////////////////////////////////////////////
void LogTF::ReadConfigFile()
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


	this->publishTF = cfg.lookup("tf.publish");
	std::cout << "LogTF - publish TF: " << this->publishTF << std::endl;

	this->writeAllTFTransf = cfg.lookup("tf.write_all_transf");
	std::cout << "LogTF - write_all_transf: " << this->writeAllTFTransf << std::endl;

	// in case thresholds are set for logging the tf
	if (!this->writeAllTFTransf)
	{
		this->tfVectDistThresh = cfg.lookup("tf.dist_tresh");
		std::cout << "LogTF - tf_dist_tresh: " << this->tfVectDistThresh << std::endl;

		this->tfAngularDistThresh = cfg.lookup("tf.angular_tresh");
		std::cout << "LogTF - angular_tresh: " << this->tfAngularDistThresh << std::endl;

		this->tfDurationThresh = cfg.lookup("tf.duration_tresh");
		std::cout << "LogTF - duration_tresh: " << this->tfDurationThresh << std::endl;
	}
}

//////////////////////////////////////////////////
void LogTF::WriteAndPublishTF()
{
	// vector with all the transforms
	std::vector<tf::StampedTransform> stamped_transforms;

	// set ros time with the simulation time
	ros::Time tf_time = ros::Time(this->world->GetSimTime().sec, this->world->GetSimTime().nsec);

	// create a local transformation
	tf::Transform transform;

	// create a local gazebo pose
	math::Pose pose;


	// loop through all the models
	for(physics::Model_V::const_iterator m_iter = this->models.begin();
			m_iter != this->models.end(); m_iter++)
	{
		// save the pose of the current model
		pose = m_iter->get()->GetWorldPose();

		// set position and orientation to the transform
		transform.setOrigin(tf::Vector3(pose.pos.x, pose.pos.y, pose.pos.z));
		transform.setRotation(tf::Quaternion(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w));

		// add the transform to the vectors
		stamped_transforms.push_back(
				tf::StampedTransform(transform, tf_time, "/map" /*this->world->GetName()*/, m_iter->get()->GetName()));

		// get the links vector from the current model
		physics::Link_V links = m_iter->get()->GetLinks();

		// loop through the links
		for (physics::Link_V::const_iterator l_iter = links.begin();
				l_iter != links.end(); l_iter++)
		{
			// save the pose of the current link
			pose = l_iter->get()->GetRelativePose();

			// set position and orientation to the transform
			transform.setOrigin(tf::Vector3(pose.pos.x, pose.pos.y, pose.pos.z));
			transform.setRotation(tf::Quaternion(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w));

			// add the transform to the vectors
			stamped_transforms.push_back(
					tf::StampedTransform(transform, tf_time, m_iter->get()->GetName(), l_iter->get()->GetName()));

			// get the collisions of the current link
            physics::Collision_V collisions = l_iter->get()->GetCollisions();

			// loop through all the collision
			for (physics::Collision_V::const_iterator c_iter = collisions.begin();
					c_iter != collisions.end(); c_iter++)
            {
				// save the pose of the current link
				pose = c_iter->get()->GetRelativePose();

				// set position and orientation to the transform
				transform.setOrigin(tf::Vector3(pose.pos.x, pose.pos.y, pose.pos.z));
				transform.setRotation(tf::Quaternion(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w));

				// add the transform to the vectors
				stamped_transforms.push_back(
						tf::StampedTransform(transform, tf_time, l_iter->get()->GetName(), c_iter->get()->GetName()));
			}
		}
	}

	// publish if flag is set to true
	if(this->publishTF)
	{
		// TODO, static or class member?
		// broadcaster to send the information
		static tf::TransformBroadcaster transf_br;

		// broadcast the transforms
		transf_br.sendTransform(stamped_transforms);
	}

	// write the transformations to the data base
	LogTF::WriteTFData(stamped_transforms);
}

//////////////////////////////////////////////////
void LogTF::WriteTFData(const std::vector<tf::StampedTransform>& _stamped_transforms)
{
	// create bson transform object
	std::vector<BSONObj> transforms_bo;

	// get the timestamp im ms and date format
	Date_t stamp_ms = this->world->GetSimTime().nsec / 1000000.0 + this->world->GetSimTime().sec * 1000.0  + (TIME_OFFSET * this->suffixTime);

	// iterate through the stamped tranforms
	for (std::vector<tf::StampedTransform>::const_iterator st_iter = _stamped_transforms.begin();
			st_iter != _stamped_transforms.end(); ++st_iter)
	{
		// check if the transform should be written
		if(LogTF::CheckTFThresh(st_iter))
		{
			// the tf transformation
			BSONObjBuilder transform_stamped_bb;

			// the translation and rotation
			BSONObjBuilder transform_bb;

			transform_stamped_bb.append("header", BSON(   "seq" << this->tfSeq
					<< "stamp" << stamp_ms
					<< "frame_id" << st_iter->frame_id_));

			transform_stamped_bb.append("child_frame_id", st_iter->child_frame_id_);

			transform_bb.append("translation", BSON(   "x" << st_iter->getOrigin().x()
					<< "y" << st_iter->getOrigin().y()
					<< "z" << st_iter->getOrigin().z()));

			transform_bb.append("rotation", BSON(   "x" << st_iter->getRotation().x()
					<< "y" << st_iter->getRotation().y()
					<< "z" << st_iter->getRotation().z()
					<< "w" << st_iter->getRotation().w()));

			transform_stamped_bb.append("transform", transform_bb.obj());

			transforms_bo.push_back(transform_stamped_bb.obj());
		}
	}

	// increment the the message seq
	this->tfSeq++;

    // insert document object into the database, use scoped connection
	ScopedDbConnection scoped_connection(this->connName);

	// insert document object into the database
	scoped_connection->insert(this->dbName + "." + this->collName + "_tf", BSON("transforms" << transforms_bo
														<< "__recorded" << stamp_ms
														<< "__topic" << "/tf_sim"));

	// let the pool know the connection is done
	scoped_connection.done();
}

//////////////////////////////////////////////////
bool LogTF::CheckTFThresh(const std::vector<tf::StampedTransform>::const_iterator& _curr_st_iter)
{
	if(this->writeAllTFTransf)
	{
		// write transformation to db
		return true;
	}

	// flag to check if transformation is new
	bool transf_found = false;

	// TODO might be faster with std::map
	// loop through the prev tf transformation if there are any
	if (!this->lastTFTransformsMemory.empty())
	{
		for (std::vector<tf::StampedTransform>::iterator memory_st_iter = this->lastTFTransformsMemory.begin();
				memory_st_iter != this->lastTFTransformsMemory.end(); memory_st_iter++)
		{
			// check if the transform is the same with the current one
			if((memory_st_iter->frame_id_ == _curr_st_iter->frame_id_ && memory_st_iter->child_frame_id_ == _curr_st_iter->child_frame_id_) ||
					(memory_st_iter->frame_id_ == _curr_st_iter->child_frame_id_ && memory_st_iter->child_frame_id_ == _curr_st_iter->frame_id_))
			{
				// the transformation is found
				transf_found = true;

				// compute vectorial distance between the current transformation and the last one
				double vect_dist = _curr_st_iter->getOrigin().distance(memory_st_iter->getOrigin());


				// TODO check the weird rotation values 6.2 etc
				double angular_dist = 2.0 * fabs(
						_curr_st_iter->getRotation().angle(memory_st_iter->getRotation()));

				// TODO check which time to use
				// ms
				double duration = fabs(
						(_curr_st_iter->stamp_.sec * 1000.0 + _curr_st_iter->stamp_.nsec / 1000000.0) -
								(memory_st_iter->stamp_.sec * 1000.0 + memory_st_iter->stamp_.nsec / 1000000.0));
				// ns
//				double duration2 = fabs(
//						(_curr_st_iter->stamp_.sec * 1e9 + _curr_st_iter->stamp_.nsec) -
//								(memory_st_iter->stamp_.sec * 1e9 + memory_st_iter->stamp_.nsec));


				// check if the thresholds are crossed
				if(vect_dist > this->tfVectDistThresh ||
						angular_dist > this->tfAngularDistThresh ||
								duration > this->tfDurationThresh)
				{
					std::cout << memory_st_iter->frame_id_<< "->" <<memory_st_iter->child_frame_id_
							<< " dist: " << vect_dist
							<< " rot: " << angular_dist
							<< " duration: " << duration << std::endl;

					// one of the threshold passed, the current transformation is added to the memory
					*(memory_st_iter) = *(_curr_st_iter);

					// write transformation to db
					return true;
				}
			}
		}
	}

	// check if transformation is new
	if(!transf_found)
	{
		// transform is new, so log it.
		this->lastTFTransformsMemory.push_back((*_curr_st_iter));

		// write transformation to db
		return true;
	}

	return false;
}
