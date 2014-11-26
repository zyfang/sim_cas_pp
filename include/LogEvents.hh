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

#ifndef LOG_EVENTS_HH
#define LOG_EVENTS_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <beliefstate_client/BeliefstateClient.h>
#include <beliefstate_client/Context.h>
#include <libconfig.h++>
#include "mongo/client/dbclient.h"
#include "GzEvent.hh"

/// \brief Post Processing classes namespace
namespace kgpp
{
/// \brief class LogEvents
class LogEvents
{
	/// \brief Constructor
	public: LogEvents(const gazebo::physics::WorldPtr _world,
			const std::string _db_name,
			const std::string _coll_name);

	/// \brief Destructor
	public: virtual ~LogEvents();

	/// \brief Initialise events
	public: void InitEvents();

	/// \brief Write semantic events to OWL files
	public: void CheckEvents();

	/// \brief Finalize events
	public: void FiniEvents();

	/// \brief Load config file
	private: void ReadConfigFile();

	/// \brief Check current Grasp
	private: bool CheckCurrentGrasp(
			const double _timestamp_ms,
			bool _fore_finger_contact,
			bool _thumb_contact,
			gazebo::physics::Collision *_grasp_coll1,
			gazebo::physics::Collision *_grasp_coll2);

	/// \brief Check current event collisions
	private: bool CheckCurrentEventCollisions(
			const double _timestamp_ms,
			std::set<std::pair<std::string, std::string> > &_curr_ev_contact_model_pair_S);

	/// \brief Check liquid transfer event
	private: bool CheckFluidFlowTransEvent(
			const double _timestamp_ms,
			int _prev_poured_particle_nr);

	/// \brief Join short disconnections in the events timeline
	private: void JoinShortDisconnections();

	/// \brief End still active events
	private: void EndActiveEvents();

	/// \brief Write beliefstate contexts
	private: void WriteContexts();

	/// \brief Write timelines to file
	private: void WriteTimelines();

	/// \brief Gazebo world
	private: const gazebo::physics::WorldPtr world;

	/// \brief Vector of the world models
	private: gazebo::physics::Model_V models;

	/// \brief liquid model
	private: gazebo::physics::ModelPtr liquidSpheres;

	/// \brief pointer of ContactManager, for getting contacts from physics engine
	private: gazebo::physics::ContactManager *contactManagerPtr;

	/// \brief Database name
	private: const std::string dbName;

	/// \brief Db collection name
	private: const std::string collName;

	/// \brief Event disconnection threshold limit
	private: double eventDiscTresh;

	/// \brief Set with the event contact model names
	private: std::set<std::pair<std::string, std::string> > prevEvContactModelPair_S;

	// TODO remove maps
	/// \brief map of event collisions to a set of all its contacts model names
	private: std::map<gazebo::physics::Collision*, std::set<std::string> > prevEvCollToModelNames_S_M;

	/// \brief map of event collisions to a set of all its particle names
	private: std::map<gazebo::physics::Collision*, std::set<std::string> > eventCollToSetOfParticleNames_M;

	/// \brief Event no contact collision vector
	private: std::set<gazebo::physics::Collision*> eventCollisions_S;

	/// \brief Hand thumb and fore finger event collision
	private: gazebo::physics::Collision *eventCollisionForeFinger;

	/// \brief Hand thumb and fore finger event collision
	private: gazebo::physics::Collision *eventCollisionThumb;

	/// \brief Mug top event collision
	private: gazebo::physics::Collision* eventCollisionMug;

	/// \brief all particle collisions
	private: std::set<gazebo::physics::Collision*> allLiquidParticles_S;

	/// \brief poured particle collisions
	private: std::set<gazebo::physics::Collision*> totalPouredParticles_S;

	/// \brief particle collisions belonging to the pancake
	private: std::set<gazebo::physics::Collision*> pancakeCollision_S;

	/// \brief name of the grasped model
	private: std::string prevGraspedModel;

	/// \brief Map of event names to a stack of GzEvent
	private: std::map<std::string, std::list<kgpp::GzEvent*> > nameToEvents_M;

	/// \brief Map of all the objects name from the simulation to beliefstate objects
	private: std::map<std::string, beliefstate_client::Object*> nameToBsObject_M;

	// TODO remove this
	/// \brief Model names to GzEventObj map
	private: std::map<std::string, kgpp::GzEventObj*> nameToEventObj_M;

	/// \brief Grasp GzEvent
	private: kgpp::GzEvent* graspGzEvent;

	/// \brief Flag for when the pancake is created
	private: bool pancakeCreated;

	/// \brief Grasp flag
	private: bool graspInit;

	/// \brief Beliefstate client
	private: beliefstate_client::BeliefstateClient* beliefStateClient;

	/// \brief Log location of the events
	private: std::string logLocation;

};
}
#endif
