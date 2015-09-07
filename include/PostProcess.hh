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

#include <gazebo/gazebo.hh>
#include <mongo/client/dbclient.h>
#include <libconfig.h++>

#include "GzEvent.hh"
#include "LogTF.hh"
#include "LogEvents.hh"
#include "LogRaw.hh"
#include "LogParticles.hh"

namespace sg_pp
{
/// \brief class PostProcess
class PostProcess : public gazebo::SystemPlugin
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
	private: void ReadConfigFile();

	/// \brief Call after the world connected event
	private: void InitOnWorldConnect();

	/// \brief Check which models have face collision for events
	private: void FirstSimulationStepInit();

	/// \brief Function having all the post processing threads
	private: void ProcessCurrentData();

	/// \brief Check if the log play has finished
	private: void CheckLoggingFinishedWorker();

	/// \brief Terminate simulation
	private: void TerminateSimulation();

	/// \brief Contacts callback function, just to start the contacts in the physics engine
	private: void DummyContactsCallback(ConstContactsPtr& _msg);

	/// \brief Thread for checking the end of a log
	private: boost::thread* checkLogginFinishedThread;

	/// \brief TF logger class
	private: sg_pp::LogTF* tfLogger;

	/// \brief Event logger class
	private: sg_pp::LogEvents* eventsLogger;

	/// \brief Raw logger class
	private: sg_pp::LogRaw* rawLogger;

	/// \brief Particle logger class
	private: sg_pp::LogParticles* particleLogger;

	/// \brief World name to be connected to
	private: std::string worldName;

	/// \brief Database name
	private: std::string dbName;

	/// \brief Db collection name
	private: std::string collName;

	/// \brief Db collection name
	private: std::string collSuffix;

	/// \brief Gazebo communication node
	private: gazebo::transport::NodePtr gznode;

	/// \brief Server control publisher for shutting down the server when pp terminated
	private: gazebo::transport::PublisherPtr serverControlPub;

	/// \brief Gazebo subscriber to contacts
	private: gazebo::transport::SubscriberPtr contactSub;

	/// \brief World created connection
	private: gazebo::event::ConnectionPtr worldCreatedConnection;

	/// \brief World update connection
	private: gazebo::event::ConnectionPtr eventConnection;

	/// \brief pointer of ContactManager, for getting contacts from physics engine
	private: gazebo::physics::ContactManager *contactManagerPtr;

	/// \brief World Pointer
	private: gazebo::physics::WorldPtr world;

	/// \brief Flag used to set that initially pause mode is set, used of detecting the end of a Log
	private: bool pauseMode;

	/// \brief Check what to process
	private: bool processTf;

	/// \brief Check what to process
	private: bool processRaw;

	/// \brief Check what to process
	private: bool processEvents;

	/// \brief Check what to process
	private: bool processParticle;
};
}


#endif
