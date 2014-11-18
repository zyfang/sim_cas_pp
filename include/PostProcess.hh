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

#include <beliefstate_client/BeliefstateClient.h>
#include <beliefstate_client/Context.h>

#include <libconfig.h++>

#include "GzEvent.hh"
#include "LogTF.hh"
#include "LogEvents.hh"

namespace gazebo
{
/// \brief class PostProcess
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

	/// \brief Load config file
	protected: void ReadConfigFile();

	/// \brief Call after the world connected event
	private: void InitOnWorldConnect();

	/// \brief Check which models have face collision for events
	private: void FirstSimulationStepInit();

	/// \brief Function having all the post processing threads
	private: void ProcessCurrentData();

	/// \brief Write semantic events to OWL files
	private: void WriteSemanticData();

	/// \brief Check current Grasp
	private: bool CheckCurrentGrasp(
			const double _timestamp_ms,
			bool _fore_finger_contact,
			bool _thumb_contact,
			physics::Collision *_grasp_coll1,
			physics::Collision *_grasp_coll2);

	/// \brief Check current event collisions
	private: bool CheckCurrentEventCollisions(
			const double _timestamp_ms,
			std::set<std::pair<std::string, std::string> > &_curr_ev_contact_model_pair_S);

	/// \brief Check liquid transfer event
	private: bool CheckFluidFlowTransEvent(
			const double _timestamp_ms,
			int _prev_poured_particle_nr);

	/// \brief Contacts callback function, just to start the contacts in the physics engine
	private: void DummyContactsCallback(ConstContactsPtr& _msg);

	/// \brief Check if the log play has finished
	private: void LogCheckWorker();

	/// \brief Terminate simulation
	private: void TerminateSimulation();

	/// \brief Join short disconnections in the events timeline
	private: void JoinShortDisconnections();

	/// \brief End still active events
	private: void EndActiveEvents();

	/// \brief Write beliefstate contexts
	private: void WriteContexts();

	/// \brief Write timelines to file
	private: void WriteTimelines();

	/// \brief TF logger class
	private: postp::LogTF* tfLogger;

	/// \brief Event logger class
	private: postp::LogEvents* eventsLogger;


	/// \brief World name
	private: std::string worldName;

	/// \brief Database name
	private: std::string dbName;

	/// \brief Db collection name
	private: std::string collName;

	/// \brief Db collection name
	private: std::string collSuffix;

	/// \brief Gazebo communication node
	private: transport::NodePtr gznode;

	/// \brief Gazebo subscriber
	private: transport::SubscriberPtr contactSub;

	/// \brief Connection to the database
	private: mongo::DBClientConnection mongoDBClientConnection;

	/// \brief Thread for checking the end of a log
	private: boost::thread* checkLogEndThread;

	/// \brief Flag used to set that initially pause mode is set, used of detecting the end of a Log
	private: bool pauseMode;

	/// \brief World Pointer
	private: physics::WorldPtr world;

	/// \brief Vector of the world models
	private: physics::Model_V models;

	/// \brief World created connection
	private: event::ConnectionPtr worldCreatedConnection;

	/// \brief Pause event connection
	private: event::ConnectionPtr pauseConnection;

	/// \brief World update connection
	private: event::ConnectionPtr eventConnection;

	/// \brief pointer of ContactManager, for getting contacts from physics engine
	private: physics::ContactManager *contactManagerPtr;

	/// \brief Mug top event collision
	private: physics::Collision* eventCollisionMug;

	/// \brief Hit hand thumb and fore finger event collision
	private: physics::Collision *eventCollisionForeFinger, *eventCollisionThumb;

	/// \brief Event no contact collision vector
	private: std::set<physics::Collision*> eventCollisions_S;

	/// \brief Set with the event contact model names
	private: std::set<std::pair<std::string, std::string> > prevEvContactModelPair_S;

	// TODO remove maps
	/// \brief map of event collisions to a set of all its contacts model names
	private: std::map< physics::Collision*, std::set<std::string> > prevEvCollToModelNames_S_M;

	/// \brief map of event collisions to a set of all its particle names
	private: std::map< physics::Collision*, std::set<std::string> > eventCollToSetOfParticleNames_M;

	/// \brief name of the grasped model
	private: std::string prevGraspedModel;

	/// \brief liquid model
	private: physics::ModelPtr liquidSpheres;

	/// \brief all particle collisions
	private: std::set<physics::Collision*> allLiquidParticles_S;

	/// \brief poured particle collisions
	private: std::set<physics::Collision*> totalPouredParticles_S;

	/// \brief particle collisions belonging to the pancake
	private: std::set<physics::Collision*> pancakeCollision_S;

	/// \brief flag for starting / finishing the pouring
	private: bool pouringStarted, pouringFinished;

	/// \brief flag for when the pancake is created
	private: bool pancakeCreated;

	/// \brief Beliefstate client
	private: beliefstate_client::BeliefstateClient* beliefStateClient;

	/// \brief Map of all the objects name from the simulation to beliefstate objects
	private: std::map<std::string, beliefstate_client::Object*> nameToBsObject_M;

	/// \brief Grasp flag
	private: bool graspInit;

	/// \brief Grasp GzEvent
	private: postp::GzEvent* graspGzEvent;

	/// \brief Map of event names to a stack of GzEvent
	private: std::map<std::string, std::list<postp::GzEvent*> > nameToEvents_M;

	// TODO remove this
	/// \brief Model names to GzEventObj map
	private: std::map<std::string, postp::GzEventObj*> nameToEventObj_M;


};
}


#endif
