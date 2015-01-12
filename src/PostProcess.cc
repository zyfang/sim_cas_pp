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

#include "PostProcess.hh"

#include <iostream>
#include <fstream>
#include <boost/thread.hpp>
#include <gazebo/util/LogPlay.hh>

using namespace gazebo;
using namespace mongo;
using namespace sg_pp;

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(PostProcess)

//////////////////////////////////////////////////
PostProcess::PostProcess()
{
	// TODO init ros only if needed
	// intialize ROS
	int argc = 0;
	char** argv = NULL;
	ros::init(argc, argv, "post_process");
}

//////////////////////////////////////////////////
PostProcess::~PostProcess()
{
	delete this->checkLogginFinishedThread;
}

//////////////////////////////////////////////////
void PostProcess::Load(int _argc, char ** _argv)
{
    for (unsigned int i = 0; i < _argc; ++i){
    	// look for '--suffix' characters
    	if(std::string(_argv[i]) == "--suffix"){
            // set the next argument as the name of the db and collection
    		this->collSuffix = _argv[++i];
    	}
    }

    // read config file
    PostProcess::ReadConfigFile();
}

//////////////////////////////////////////////////
void PostProcess::Init()
{
    // set the flag that the simulation starts in pause mode
    this->pauseMode = true;

    // Initialize variables after connected to the world
    this->worldCreatedConnection =  event::Events::ConnectWorldCreated(
            boost::bind(&PostProcess::InitOnWorldConnect, this));

    // get the event collisions, only called once, the connection is then changed
    this->eventConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&PostProcess::FirstSimulationStepInit, this));

    // thread for checking if the log has finished playing
	this->checkLogginFinishedThread =
			new boost::thread(&PostProcess::CheckLoggingFinishedWorker, this);
}

//////////////////////////////////////////////////
void PostProcess::ReadConfigFile()
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
	this->dbName = cfg.lookup("mongo.db_name").c_str();
	std::cout << "*PostProcess* - db_name: " << this->dbName << std::endl;

	// set the collection name
	this->collName = cfg.lookup("mongo.coll_name").c_str();
	// if a suffix has been added append it to the collection name
	if(this->collSuffix != NULL){
		this->collName += this->collSuffix;
	}
	std::cout << "*PostProcess* - coll_name: " << this->collName << std::endl;

	this->worldName = cfg.lookup("sim.world_name").c_str();
	std::cout << "*PostProcess* - world_name: " << this->worldName << std::endl;

	this->processRaw = cfg.lookup("pp.raw");
	std::cout << "*PostProcess* - processing raw data: " << this->processRaw << std::endl;

	this->processTf = cfg.lookup("pp.tf");
	std::cout << "*PostProcess* - processing tf data: " << this->processTf << std::endl;

	this->processEvents = cfg.lookup("pp.events");
	std::cout << "*PostProcess* - processing events data: " << this->processEvents << std::endl;
}

//////////////////////////////////////////////////
void PostProcess::InitOnWorldConnect()
{
	// get the world
	this->world = physics::get_world(this->worldName);

	// get the contact manager to refresh it every timestamp
	this->contactManagerPtr = this->world->GetPhysicsEngine()->GetContactManager();

	// creating gazebo node for subscribing/publishing, just to start the contacts in the physics engine
	this->gznode = transport::NodePtr(new transport::Node());

	// Initialize gazebo node, just to start the contacts in the physics engine
	this->gznode->Init();

	// publisher for shutting down the server
	serverControlPub = this->gznode->Advertise<msgs::ServerControl>("/gazebo/server/control");

	// subscribing to the contacts topic, just to start the contacts in the physics engine
	// if no subscription is done to the contacts topic the contact manager does not run
    this->contactSub = this->gznode->Subscribe(
            "~/physics/contacts", &PostProcess::DummyContactsCallback, this);

    // initialize the tf logging class
    this->tfLogger = new sg_pp::LogTF(this->world, this->dbName, this->collName, std::atoi(this->collSuffix.c_str()));

    // initialize the events logging class
    this->eventsLogger = new sg_pp::LogEvents(this->world, this->dbName, this->collName, std::atoi(this->collSuffix.c_str()));

    // initialize the raw logging class
    this->rawLogger = new sg_pp::LogRaw(this->world, this->dbName, this->collName);
}

//////////////////////////////////////////////////
void PostProcess::FirstSimulationStepInit()
{
    // set the flag to false, so the end of the log simulation can be detected
    this->pauseMode = false;

    // Initialize events
    this->eventsLogger->InitEvents();

	// Run the post processing threads once so the first step is not skipped
	PostProcess::ProcessCurrentData();

    // From now on for every update event call the given function
    this->eventConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&PostProcess::ProcessCurrentData, this));
}

//////////////////////////////////////////////////
void PostProcess::ProcessCurrentData()
{
	// group of threads for processing the data in parallel
	boost::thread_group process_thread_group;

	// tf data
	if (this->processTf)
	{
		process_thread_group.create_thread(
				boost::bind(&sg_pp::LogTF::WriteAndPublishTF, this->tfLogger));
	}

	// events data
	if (this->processEvents)
	{
		process_thread_group.create_thread(
				boost::bind(&sg_pp::LogEvents::CheckEvents, this->eventsLogger));
	}

	// raw data
	if (this->processRaw)
	{
		process_thread_group.create_thread(
				boost::bind(&sg_pp::LogRaw::WriteRawData, this->rawLogger));
	}

	// wait for all the threads to finish work
	process_thread_group.join_all();

	// clear/refresh the contact manager, otherwise data from past contacts are still present
    this->contactManagerPtr->Clear();
}

//////////////////////////////////////////////////
void PostProcess::CheckLoggingFinishedWorker()
{
	// flag to stop the while loop
	bool log_play_finished = false;

	// loop until the log has finished playing
	while(!log_play_finished)
	{
		// loop sleep
		usleep(2000000);

		// if the world is paused
		if(this->world && this->world->IsPaused() && !this->pauseMode)
		{
			// check that no manual pause happened!
			std::string sdfString;
			if(!util::LogPlay::Instance()->Step(sdfString))
			{
				log_play_finished = true;
				std::cout << "*PostProcess* - Last recorded step at " << this->world->GetSimTime().Double()
						<< ", terminating simulation.."<< std::endl;
			}
			else
			{
				std::cout << "*PostProcess* - Manual pause, every time this msg appears one simulation step is lost.. "  << std::endl;
			}
		}
	}

	// terminate simulation
	PostProcess::TerminateSimulation();
}

//////////////////////////////////////////////////
void PostProcess::TerminateSimulation()
{
	// finish the events
	this->eventsLogger->FiniEvents();

	// shutdown ros
	ros::shutdown();

    std::cout << "Shutting down server.." << std::endl;

    // send server control msg to terminate the server (does not apply when client is running)
    msgs::ServerControl server_msg;
    server_msg.set_stop(true);
    serverControlPub->Publish(server_msg);

    // finish the simulation
//	gazebo::shutdown();
}

//////////////////////////////////////////////////
void PostProcess::DummyContactsCallback(ConstContactsPtr& _msg)
{
}
