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
namespace sg_pp
{
/// \brief class LogEvents
class LogEvents
{
	/// \brief Constructor
	public: LogEvents(const gazebo::physics::WorldPtr _world,
			const std::string _db_name,
            const std::string _coll_name,
            const std::string _connection_name,
            const int _timeoffset,
            const std::string _cfg_file);

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
    private: void CheckGraspEvent(
			const double _timestamp_ms,
            const unsigned int _grasp_contacts_nr,
            const std::set<std::string> &_grasped_models);

	/// \brief Check current event collisions
    private: void CheckSurfaceEvents(
			const double _timestamp_ms,
            const std::set<std::pair<std::string, std::string> > &_curr_surface_models_in_contact);

    /// \brief Check translation event
    private: void CheckTranslEvent(
            const double _timestamp_ms,
            const bool _transf_detect_flag);

    /// \brief Check tool event
    private: void CheckToolEvent(
            const double _timestamp_ms,
            const std::string _contact_with_tool_model,
            const std::string _tool_name);

	/// \brief Join short disconnections in the events timeline
	private: void MergeEventDisconnections();

	/// \brief End still active events
	private: void EndActiveEvents();

	/// \brief Write beliefstate (owl) contexts
	private: void WriteContexts();

	/// \brief Write timelines to file
	private: void WriteTimelines();

	/// \brief Gazebo world
	private: const gazebo::physics::WorldPtr world;

    /// \brief pointer of ContactManager, for getting contacts from physics engine
    private: gazebo::physics::ContactManager *contactManagerPtr;

	/// \brief Beliefstate client
    private: beliefstate_client::BeliefstateClient* beliefStateClient;

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

	/// \brief Log location of the events
    private: std::string logLocation;

    /// \brief Event disconnection threshold limit
    private: double eventDiscThresh;

    /// \brief Transfer event duration thresh
    private: double transfEvDurThresh;

	/// \brief Previous state of models being in contact with surfaces
    /// Set with <surface model name, model in contact with>
    private: std::set<std::pair<std::string, std::string> >
      prevSurfaceModelsInContact;

    // TODO change container, vector? to have access to the next value?
    /// \brief Map of event names to GzEvents
    private: std::map<std::string, std::list<sg_pp::PpEvent*> > nameToEvents_M;

    // TODO change, needed so every object appears once for knowrob hashing
    /// \brief Map of all the objects name from the simulation to beliefstate objects
    private: std::map<std::string, beliefstate_client::Object*> nameToBsObject_M;

	// TODO remove this
	/// \brief Model names to GzEventObj map
    private: std::map<std::string, sg_pp::PpEventObj*> nameToEventObj_M;

    /// \brief Surface collision names to check in the world
    private: std::set<std::string> surfaceCollNames;

    /// \brief Surface collisions to check in the world
    private: std::set<gazebo::physics::Collision*> surfaceColls;

    // TODO add init to avoid checking for NULl
    /// \brief Grasp GzEvent
    private: sg_pp::PpEvent* graspGzEvent;

    /// \brief Grasp flag
    private: bool graspInit;

    /// \brief Grasp collision names to check for grasp contacts
    private: std::set<std::string> graspCollNames;

    /// \brief Grasp collisions to check for grasp contacts
    private: std::set<gazebo::physics::Collision*> graspColls;

    /// \brief Container collision names to particle model names
    private: std::map<std::string, std::set<std::string> > contCollToPNames;

    /// \brief Container collisions to particle models
    private: std::map<gazebo::physics::Collision*,
      std::set< gazebo::physics::ModelPtr> > contCollToPModels;

    /// \brief Transfered particles of the given model
    private: std::map<gazebo::physics::ModelPtr,
      std::map<gazebo::physics::Collision*, bool> > transferedParticles;

    /// \brief Number of transfered particles of the given model
    private: std::map<gazebo::physics::ModelPtr, unsigned int>
      transferedPartCount;

    /// \brief Timestamp of latest particle leaving the container
    private: double transfTs;

    /// \brief Pool of the transfered particles in the current transfer event
    private: std::vector<gazebo::physics::Collision*> transfParticlePool;

    // TODO works for one transfer per time (mixed particles in a bowl would not work)
    /// \brief Transf event
    private: sg_pp::PpEvent* transfEvent;

    /// \brief Tool collision names to particle model names
    private: std::map<std::string, std::set<std::string> > toolCollToPNames;

    /// \brief Tool collisions to particle models
    private: std::map<gazebo::physics::Collision*,
      std::set< gazebo::physics::ModelPtr> > toolCollToPModels;

    /// \brief Name of the prev contact with the tool
    private: std::string prevContactWithToolModel;

    // TODO generalize for multiple events
    /// \brief Tool event
    private: sg_pp::PpEvent* toolGzEvent;

    /// \brief name of the grasped model
    private: std::string prevGraspedModel;

};
}
#endif
