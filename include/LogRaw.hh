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

#ifndef LOG_RAW_HH
#define LOG_RAW_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <libconfig.h++>
#include <mongo/client/dbclient.h>

/// \brief Post Processing classes namespace
namespace sg_pp
{
	/// \brief class LogRaw
	class LogRaw
	{
		/// \brief Constructor
		public: 
			LogRaw(const gazebo::physics::WorldPtr _world,
				const std::string _db_name,
	            const std::string _coll_name,
	            const std::string _connection_name,
	            const int _timeoffset,
	            const std::string _cfg_file);

		/// \brief Destructor
		public: virtual ~LogRaw();

	    /// \brief Write raw data to mongodb
	    public: void WriteRawData();

		/// \brief Load config file
		private: void ReadConfigFile();

		/// \brief Check treshold in order to write data
		public: bool CheckThreshold(const gazebo::physics::ModelPtr _model);
		
	    /// \brief Return a contact bson object
		private: mongo::BSONObj CreateBSONContactObject(
				const gazebo::physics::Contact* _contact,
				const gazebo::physics::Collision* _collision);

		/// \brief Return a collision bson object
		private: mongo::BSONObj CreateBSONCollisionObject(
				const gazebo::physics::CollisionPtr _collision,
				const mongo::BSONArray _contact_arr);

		/// \brief Return a link bson object
		private: mongo::BSONObj CreateBSONLinkObject(
				const gazebo::physics::LinkPtr _link,
				const mongo::BSONArray _collision_arr);

		/// \brief Return a link bson object
		private: mongo::BSONObj CreateBSONModelObject(
				const gazebo::physics::ModelPtr _model,
				const mongo::BSONArray _link_arr);

		/// \brief Gazebo world
		private: const gazebo::physics::WorldPtr world;

		/// \brief Vector of the world models
		private: gazebo::physics::Model_V models;

		/// \brief which connection to log to
		private: const std::string connName;

		/// \brief Config file name
		private: const std::string cfgFilename;

		/// \brief Database name
		private: const std::string dbName;

		/// \brief Db collection name
		private: const std::string collName;

	    /// \brief storing absolute timeoffset
	    private: const int TIME_OFFSET;

		/// \brief pointer of ContactManager, for getting contacts from physics engine
		private: gazebo::physics::ContactManager *contactManagerPtr;

		/// \brief Flag to write all logs
		private: bool writeAll;

		/// \brief Distance tresh between the entities
		private: double distTh;

		/// \brief Angle tresh between the entities
		private: double angleTh;

		/// \brief Prev pose of all the models
		private: std::map<gazebo::physics::ModelPtr, gazebo::math::Pose> modelPoseMemoryMap;

	};
}
#endif
