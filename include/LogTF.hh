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

#ifndef LOG_TF_HH
#define LOG_TF_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <libconfig.h++>
#include "mongo/client/dbclient.h"

/// \brief Post Processing classes namespace
namespace sg_pp
{
/// \brief class LogTF
class LogTF
{
	/// \brief Constructor
	public: LogTF(const gazebo::physics::WorldPtr _world,
			const std::string _db_name,
			const std::string _coll_name,
			int _suffix,
			const std::string _connection_name);

	/// \brief Destructor
	public: virtual ~LogTF();

	/// \brief Publish / Write tf data
	public: void WriteAndPublishTF();

	/// \brief Load config file
	private: void ReadConfigFile();

	/// \brief Write tf transforms to the database
	private: void WriteTFData(const std::vector<tf::StampedTransform>& _stamped_transforms);

	/// \brief Check if the transform should be written to the db
	private: bool CheckTFThresh(const std::vector<tf::StampedTransform>::const_iterator& _st_iter);

	/// \brief Gazebo world
	private: const gazebo::physics::WorldPtr world;

	/// \brief Vector of the world models
	private: gazebo::physics::Model_V models;

	/// \brief which connection to log to
	private: const std::string connName;
	
	/// \brief Database name
	private: const std::string dbName;

	/// \brief Db collection name
	private: const std::string collName;

	/// \brief Current tf seq nr
	private: long long int tfSeq;

	/// \brief flag for writing all tf transformations to the db
	private: bool publishTF;

	/// \brief flag for writing all tf transformations to the db
	private: bool writeAllTFTransf;

	/// \brief last timestamps tf transforms
	private: std::vector<tf::StampedTransform> lastTFTransformsMemory;

	/// \brief Vectorial distance threshold between tf transformation in order to be logged or not
	private: double tfVectDistThresh;

	/// \brief Angular distance threshold between tf transformation in order to be logged or not
	private: double tfAngularDistThresh;

	/// \brief Duration threshold between tf transformation in order to be logged or not
	private: int tfDurationThresh;

	// for adding time offset to the simulation times
	private: int suffixTime;
};
}
#endif
