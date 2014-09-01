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


#ifndef POST_PROCESS_PLUGIN_HH
#define POST_PROCESS_PLUGIN_HH

#include "gazebo/gazebo.hh"
#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/physics/Contact.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/physics/PhysicsTypes.hh"

#include "mongo/client/dbclient.h"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace gazebo
{
	/// \brief class PostProcess, saving world contacts by subscribing to a topic
	class PostProcess : public SystemPlugin
	{
		/// \brief Constructor
		public: PostProcess();

		/// \brief Destructor
		public: virtual ~PostProcess();

		/// \brief Load plugin (Load called first, then Init)
		protected: virtual void Load(int /*_argc*/, char ** /*_argv*/);

		/// \brief Init plugin (Load called first, then Init)
		protected: virtual void Init();

		/// \brief Call after the world connected event
		private: void InitOnWorldConnect();

		/// \brief Check which models have face collision for events
		private: void GetEventCollisions();

        /// \brief Called on every simulation timestamp, saves logs to mongodb
        private: void UpdateDB();

        /// \brief Write event data to mongodb
        private: void WriteEventData();

        /// \brief Write particle events data to mongodb
        private: void WriteParticleEventData();

        /// \brief Write raw data to mongodb
        private: void WriteRawData();

        /// \brief Publish tf
        private: void PublishAndWriteTFData();

        /// \brief write tf transforms to the database
        private: void WriteTFData(const std::vector<tf::StampedTransform>& _stamped_transforms);

        /// \brief check if the transform should be written to the db
        private: bool ShouldWriteTransform(std::vector<tf::StampedTransform>::const_iterator& _st_iter);

        /// \brief create a contact bson object
		private: mongo::BSONObj CreateBSONContactObject(const physics::Contact* _contact,
						const physics::Collision* _collision);

		/// \brief create a collision bson object
		private: mongo::BSONObj CreateBSONCollisionObject(const physics::CollisionPtr _collision,
														const mongo::BSONArray _contact_arr);

		/// \brief create a link bson object
		private: mongo::BSONObj CreateBSONLinkObject(const physics::LinkPtr _link,
														const mongo::BSONArray _collision_arr);

		/// \brief create a link bson object
		private: mongo::BSONObj CreateBSONModelObject(const physics::ModelPtr _model,
														const mongo::BSONArray _link_arr);

		/// \brief Contacts callback function, just to start the contacts in the physics engine
		private: void DummyContactsCallback(ConstContactsPtr& _msg);

		/// \brief World name
		private: std::string worldName;

		/// \brief Database name
		private: std::string dbName;

		/// \brief Db collection name
		private: std::string collName;

		/// \brief database name
		private: std::string dbCollName;

		/// \brief Gazebo node providing subscription and advertising
		private: transport::NodePtr gznode;

		/// \brief Gazebo subscriber
		private: transport::SubscriberPtr contactSub;

		/// \brief Connection to the database
		private: mongo::DBClientConnection mongoDBClientConnection;

		/// \brief World Pointer
		private: physics::WorldPtr world;

		/// \brief Vector of the world models
		private: physics::Model_V models;

		/// \brief Pointer to the world created pointer;
	    private: event::ConnectionPtr worldCreatedConnection;

		/// \brief Pointer to the update event connection
	    private: event::ConnectionPtr rawDBConnection;

		/// \brief Pointer to the update event connection
	    private: event::ConnectionPtr eventConnection;

	    /// \brief pointer of ContactManager, for getting contacts from physics engine
	    private: physics::ContactManager *contactManagerPtr;

	    /// \brief Mug top event collision
	    private: physics::Collision* eventCollisionMug;

	    /// \brief Hit hand thumb and fore finger event collision
	    private: physics::Collision *eventCollisionForeFinger, *eventCollisionThumb;

	    /// \brief Event no contact collision vector
	    private: std::set<physics::Collision*> eventCollisions_S;

        /// \brief map of event collisions to a set of all its contacts model names
        private: std::map< physics::Collision*, std::set<std::string> > eventCollToSetOfModelNames_M;

        /// \brief map of event collisions to a set of all its particle names
        private: std::map< physics::Collision*, std::set<std::string> > eventCollToSetOfParticleNames_M;

	    /// \brief name of the grasped model
	    private: std::string graspedModelName;

	    /// \brief liquid model
	    private: physics::ModelPtr liquidSpheres;

	    /// \brief all particle collisions
	    private: std::set<physics::Collision*> allLiquidCollisions_S;

	    /// \brief poured particle collisions
	    private: std::set<physics::Collision*> pouredLiquidCollisions_S;

        /// \brief particle collisions belonging to the pancake
        private: std::set<physics::Collision*> pancakeCollision_S;

	    /// \brief flag for starting / finishing the pouring
	    private: bool pouringStarted, pouringFinished;

        /// \brief flag for when the pancake is created
        private: bool pancakeCreated;

	    /// \brief timestamp of last particle leaving the mug
	    private: long long int lastParticleLeavingTimestamp;

	    /// \brief flag for writing all tf transformations to the db
	    private: bool writeAllTFTransf;

	    /// \brief last timestamps tf transforms
	    private: std::vector<tf::StampedTransform> lastTFTransformsMemory;

	    /// \brief Vectorial distance threshold between tf transformation in order to be logged or not
	    private: double tfVectDistThresh;

	    /// \brief Angular distance threshold between tf transformation in order to be logged or not
	    private: double tfAngularDistThresh;

	    /// \brief Duration threshold between tf transformation in order to be logged or not
	    private: double tfDurationThresh;

	    /// \brief Current tf seq nr
	    private: long long int tfSeq;

	    // DEBUG
	    private: void DebugOutput(std::string _msg);
	};
}


#endif
