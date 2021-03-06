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
	// TODO init ros only if needed (only if LogTF active, is called from there now)
	// intialize ROS
	// int argc = 0;
	// char** argv = NULL;
    // ros::init(argc, argv, "post_process");

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
        // look for '--config' characters
        if(std::string(_argv[i]) == "--config"){
            // set the next argument as the name of the db
            this->cfgFilename = _argv[++i];
            }

    	// look for '--db' characters
        if(std::string(_argv[i]) == "--db"){
            // set the next argument as the name of the db
            this->dbName = _argv[++i];
            }
        

        if(std::string(_argv[i]) == "--collection"){
            // set the next argument as the name of the collection
            this->collName = _argv[++i];
            }

        // look for '--suffix' characters
    	if(std::string(_argv[i]) == "--suffix"){
            // append to collection name and use as multiplier for timeoffset
    		this->offsetMultiplier = _argv[++i];
            this->timeOffset = std::atoi(string(this->offsetMultiplier).c_str())*sg_pp::TIME_OFFSET;
           }

        // look for '--delay' characters, if option given, delay postprocessing after gazebo has started until SimTime equals the given number in seconds
    	if(std::string(_argv[i]) == "--delay"){
            // set the next argument as the name of the db and collection
            std::string temp = _argv[++i];
            this->process_delay = std::atoi(temp.c_str());
            
            if(this->process_delay<0)
            {
            	this->process_delay=0;
            }
        }

        //whether or not the controller should take care of shutting down gazebo (or whether it's ran simultaneously with other processes that will shutdown gazebo once they're finished)
         if(std::string(_argv[i]) == "-replaying"){
             replaying = true;
            }
         else {
             replaying = false;
         }

    }

    // read config file
    PostProcess::ReadConfigFile();

    //set the flag that postprocessing didn't start yet
    startedpostprocess = false;

    //delay post-processing
    delay_postprocess_start_thread_ =
        new boost::thread(&PostProcess::DelayPostprocessStart, this);
}

//////////////////////////////////////////////////

void PostProcess::DelayPostprocessStart()
{
	std::cout << "Starting postprocessing when simtime exceeds " << this->process_delay << " seconds" << std::endl;
	while(!this->world || this->world->GetSimTime()<this->process_delay)
	{
		usleep(1000);
	}
    std::cout << "Starting Postprocessing now" << std::endl;
	if(!startedpostprocess)
	{
        // get the event collisions, only called once, the connection is then changed
	    this->eventConnection = event::Events::ConnectWorldUpdateBegin(
	        boost::bind(&PostProcess::FirstSimulationStepInit, this));

	    // thread for checking if the log has finished playing
		this->checkLogginFinishedThread =
				new boost::thread(&PostProcess::CheckLoggingFinishedWorker, this);
		startedpostprocess=true;
	}
}

//////////////////////////////////////////////////
void PostProcess::Init()
{
    // set the flag that the simulation starts in pause mode
    this->pauseMode = true;

    // Initialize variables after connected to the world
    this->worldCreatedConnection =  event::Events::ConnectWorldCreated(
            boost::bind(&PostProcess::InitOnWorldConnect, this));

//     // get the event collisions, only called once, the connection is then changed
//     this->eventConnection = event::Events::ConnectWorldUpdateBegin(
//         boost::bind(&PostProcess::FirstSimulationStepInit, this));

//     // thread for checking if the log has finished playing
//     this->checkLogginFinishedThread =
//            new boost::thread(&PostProcess::CheckLoggingFinishedWorker, this);

	//SELF NOTE: I'd move these threads out of init and into updateOnCallBack and then only call it once if the flag is true (set it to false after starting the processes, and then it should never be set to true again). Cannot test this without also changing the header file and everything. Test after dinner.
}

//////////////////////////////////////////////////
void PostProcess::ReadConfigFile()
{
	// create the config
	libconfig::Config cfg;

	// read config file
	try
	{
        if (this->cfgFilename.empty())
        {
            cfg.readFile("config.cfg");
        }
        else
        {
            std::cout << "Reading " << ("config/"+this->cfgFilename) << std::endl;
            cfg.readFile(("config/"+this->cfgFilename).c_str());
        }
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

    // if no db name has been added, use default
    if (this->dbName.empty()){
        this->dbName = cfg.lookup("mongo.db_name").c_str();
    }	
	std::cout << "*PostProcess* - db_name: " << this->dbName << std::endl;

	// if no collection name has been added, use default
    if (this->collName.empty()){
        this->collName = cfg.lookup("mongo.coll_name").c_str();
    }   

    //get port which data should be written out to
    this->dbPort = cfg.lookup("mongo.port").c_str();//to string because connectionname is string

	// suffix append to the collection name
	this->collName += "_" + this->offsetMultiplier;

	std::cout << "*PostProcess* - coll_name: " << this->collName << std::endl;

	this->worldName = cfg.lookup("sim.world_name").c_str();
	std::cout << "*PostProcess* - world_name: " << this->worldName << std::endl;

	this->processMotionExpressions = cfg.lookup("pp.motion_expressions");
	std::cout << "*PostProcess* - processing motion expressions: " << this->processMotionExpressions << std::endl;

	this->processRaw = cfg.lookup("pp.raw");
	std::cout << "*PostProcess* - processing raw data: " << this->processRaw << std::endl;

	this->processTf = cfg.lookup("pp.tf");
	std::cout << "*PostProcess* - processing tf data: " << this->processTf << std::endl;

	this->processEvents = cfg.lookup("pp.events");
	std::cout << "*PostProcess* - processing events data: " << this->processEvents << std::endl;

    this->processParticle = cfg.lookup("pp.particles");
    std::cout << "*PostProcess* - processing particle data: " << this->processParticle << std::endl;
}

//////////////////////////////////////////////////
bool PostProcess::CheckShouldLog(bool &logflag, ScopedDbConnection &scoped_connection, std::string loggingname, std::string postid)
{
	if(logflag)
	{
        // If collection already exist don't log the data
        if (scoped_connection->exists(this->dbName + "." + this->collName + "_" + postid))
		{
			// set flag to false
			logflag= false;
            std::cout << "*PostProcess* !!! Collection: " << this->dbName << "." << this->collName << "_" << postid
					<< " already exists skipping " << loggingname << std::endl;
			return false;
		}
		else
		{
			return true;
		}
	}
	else
	{
		return false;
	}
}

//////////////////////////////////////////////////
void PostProcess::InitOnWorldConnect()
{
	//specify to which port the database should be written. default ("localhost") is 27017
	const std::string connection_name = "localhost:"+ dbPort;

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

    this->logdone_sub_ = this->gznode->Subscribe("/gazebo/log/control", &PostProcess::LogDone, this);


    // Create scoped connection to check whether the collection already exists
	ScopedDbConnection scoped_connection(connection_name);

	// initialize the tf logging class
    if (PostProcess::CheckShouldLog(this->processTf, scoped_connection, "LogTF","tf"))
    {
        // initialize the tf logging class
        this->tfLogger = new sg_pp::LogTF(this->world, this->dbName, this->collName, connection_name, this->timeOffset, this->cfgFilename);
    }

    // initialize the event logging class
    if (PostProcess::CheckShouldLog(this->processEvents, scoped_connection, "LogEvents","ev"))
    {
        // initialize the events logging class
        this->eventsLogger = new sg_pp::LogEvents(this->world, this->dbName, this->collName, connection_name, this->timeOffset, this->cfgFilename);
    }

	// initialize the motion_expressions logging class
    if (PostProcess::CheckShouldLog(this->processMotionExpressions, scoped_connection, "LogMotionExpressions","motion_expressions"))
    {
		    // initialize the motion expressions logging class
            this->motionExpressionsLogger = new sg_pp::LogMotionExpressions(this->world, this->dbName, this->collName, connection_name, this->timeOffset, this->cfgFilename);
    }

    // initialize the raw with thresholding  logging class
    if (PostProcess::CheckShouldLog(this->processRaw, scoped_connection, "LogRaw","raw"))
    {
		    // initialize the raw logging class
            this->rawLogger = new sg_pp::LogRaw(this->world, this->dbName, this->collName, connection_name, this->timeOffset, this->cfgFilename);
	}

	// initialize the raw with thresholding logging class
    if (PostProcess::CheckShouldLog(this->processParticle, scoped_connection, "LogParticle","particles"))
    {
		    // initialize the particle logging class
            this->particleLogger = new sg_pp::LogParticles(this->world, this->dbName, this->collName, connection_name, this->timeOffset, this->cfgFilename);
	}

	// if no PP is happening, shut down server
	if(!this->processEvents && !this->processRaw && !this->processTf && !this->processParticle && !this->processMotionExpressions)
	{
	    std::cout << "*PostProcess* !!! No PP selected, Shutting down gzserver.. !!! " << std::endl;

	    // send server control msg to terminate the server (does not apply when client is running)
	    msgs::ServerControl server_msg;
	    server_msg.set_stop(true);
	    serverControlPub->Publish(server_msg);
	}
    
}

//////////////////////////////////////////////////
void PostProcess::FirstSimulationStepInit()
{
    // set the flag to false, so the end of the log simulation can be detected
    this->pauseMode = false;

    // Initialize events
    if (this->processEvents)
	{
    	this->eventsLogger->InitEvents();
	}
    // Initialize particles
    if(this->processParticle)
    {
    	this->particleLogger->InitParticles();
	}

	if(this->processMotionExpressions)
	{
    	this->motionExpressionsLogger->Init();
    }

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

	// motion expressions
	if (this->processMotionExpressions)
	{
		process_thread_group.create_thread(
				boost::bind(&sg_pp::LogMotionExpressions::WriteRawData, this->motionExpressionsLogger));
	}

    // particles data
    if (this->processParticle)
    {
        process_thread_group.create_thread(
                boost::bind(&sg_pp::LogParticles::WriteParticleData, this->particleLogger));
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
    log_play_finished = false;

    if(replaying) { //if there is no other process giving a shutdown signal, shutdown when log is finished replaying
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
        std::cout << "Terminating simulation from PostProcess 1"  << std::endl;
        PostProcess::TerminateSimulation();
    }
    else { //if we're going to get a message when the log has finished
        while(!log_play_finished)
        {
            // loop sleep
            usleep(2000000);

        }
        // terminate simulation
        std::cout << "Terminating simulation from PostProcess 2"  << std::endl;
        PostProcess::TerminateSimulation();
    }
}

//////////////////////////////////////////////////
void PostProcess::LogDone(ConstLogControlPtr& _msg)
{
    log_play_finished = _msg->stop();
}


void PostProcess::TerminateSimulation()
{

	// if events are processed, finish them
	if (this->processEvents)
	{
		// finish the events
		this->eventsLogger->FiniEvents();
	}

	// shutdown ros
	// ros::shutdown();

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
