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
using namespace hand_sim;

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(PostProcess)

//////////////////////////////////////////////////
PostProcess::PostProcess()
{
	// intialize ROS
	int argc = 0;
	char** argv = NULL;
	ros::init(argc, argv, "post_process");
}

//////////////////////////////////////////////////
PostProcess::~PostProcess()
{
	delete this->contactManagerPtr;

	delete this->beliefStateClient;

	delete this->checkLogEndThread;
}

//////////////////////////////////////////////////
void PostProcess::Load(int _argc, char ** _argv)
{
    // TODO add default values and separate db from coll
	// check if database and collection name have been given as arguments from the command line
    for (unsigned int i = 0; i < _argc; ++i){
    	// look for '-db' characters
    	if(std::string(_argv[i]) == "-db"){
            // set the next argument as the name of the db and collection
    		this->dbCollName = _argv[++i];

    		// if the name doesn't have a correct style, add default value
    		if(this->dbCollName.find(".") == std::string::npos){
                this->dbCollName = "sim_db.default";
    		}
    	}

    	// look for '-w' characters, for world name
    	if(std::string(_argv[i]) == "-w"){
    		// set the next argument as the name of world
    		this->worldName = _argv[++i];
    	}

    }

    // if no db.coll name has been added, use default
    if (this->dbCollName.empty()){
        this->dbCollName = "sim_db.default";
    }

    // if no world name has been added, use default
    if (this->worldName.empty()){
    	this->worldName = "kitchen_world";
    }


	// set the saving of all tf transformations flag
	this->writeAllTFTransf = true;

	// init tf message seq count
	this->tfSeq = 0;

	// set the thresholds for logging tf transformations
	this->tfVectDistThresh = 0.001;
	this->tfAngularDistThresh = 0.1;
	this->tfDurationThresh = 100; // ms


	std::cout << "******** MONGO LOG PLUGIN LOADED *********" << std::endl;
    std::cout << "\nWorld: " << this->worldName << std::endl;
    std::cout << "Database.Collection: " << this->dbCollName << std::endl;
    std::cout << "\nOptions: " << std::endl;
    std::cout << "   -w  \t\t World name to be loaded." << std::endl;
    std::cout << "   -db \t\t Database.Collection name for the data to be written to." << std::endl;

    // TODO config file
    PostProcess::ReadConfigFile();
}

//////////////////////////////////////////////////
void PostProcess::Init()
{
	// set the grasp init flag to false
	this->graspInit = false;

    // set the pouring started flag to false
    this->pancakeCreated = false;

    // set the pouring started flag to false
    this->pouringStarted = false;

    // set the pouring finished flag to false
    this->pouringFinished = false;

    // Initialize variables after connected to the world
    this->worldCreatedConnection =  event::Events::ConnectWorldCreated(
            boost::bind(&PostProcess::InitOnWorldConnect, this));

    // get the event collisions, only called once, the connection is then changed
    this->eventConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&PostProcess::FirstSimulationStepInit, this));

    // set the flag that the simulation starts in pause mode
    this->pauseMode = true;

    // thread for checking if the log has finished playing
	this->checkLogEndThread = new boost::thread(&PostProcess::LogCheckWorker, this);
}

//////////////////////////////////////////////////
void PostProcess::ReadConfigFile()
{

	std::cout << "reading config file /******************" << std::endl;
	libconfig::Config cfg;

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

	try
	{
		std::string name = cfg.lookup("name");
		std::cout << "Store name: " << name << std::endl;
	}
	catch(const libconfig::SettingNotFoundException &nfex)
	{
		std::cerr << "No 'name' setting in configuration file." << std::endl;
	}
}

//////////////////////////////////////////////////
void PostProcess::InitOnWorldConnect()
{
	// connect to the database
	this->mongoDBClientConnection.connect("localhost");

	// get the world
	this->world = physics::get_world(this->worldName);

	// get all the models
	this->models = this->world->GetModels();

    // get the liquid model
    this->liquidSpheres = this->world->GetModel("LiquidTangibleThing");

	// get the contact manager
	this->contactManagerPtr = this->world->GetPhysicsEngine()->GetContactManager();

	// creating gazebo node for subscribing/publishing, just to start the contacts in the physics engine
	this->gznode = transport::NodePtr(new transport::Node());

	// Initialize gazebo node, just to start the contacts in the physics engine
	this->gznode->Init();

	// subscribing to the contacts topic, just to start the contacts in the physics engine
	// if no subscription is done to the contacts topic the contact manager does not run
    this->contactSub = this->gznode->Subscribe(
            "~/physics/contacts", &PostProcess::DummyContactsCallback, this);

}

//////////////////////////////////////////////////
void PostProcess::FirstSimulationStepInit()
{
    // set the flag to false, so the end of the log simulation can be detected
    this->pauseMode = false;

    std::cout << "!!! First recorded step: " << this->world->GetSimTime().Double() << std::endl;

	// open the main GzEvent
	this->nameToEvents_M["Main"].push_back(
			new GzEvent("Main","&knowrob_sim;", "PancakeEpisode", this->world->GetSimTime().Double()));

	// loop through all the models to see which have event collisions
	for(physics::Model_V::const_iterator m_iter = this->models.begin();
			m_iter != this->models.end(); m_iter++)
	{
		// map model name to the GzEventObj object
		this->nameToEventObj_M[m_iter->get()->GetName()] = new GzEventObj(m_iter->get()->GetName());

		// map model name to the beliefstate object
		this->nameToBsObject_M[m_iter->get()->GetName()] =
				new beliefstate_client::Object("&knowrob_sim;", m_iter->get()->GetName());


		// get the links vector from the current model
		const physics::Link_V links = m_iter->get()->GetLinks();

		// loop through the links
		for (physics::Link_V::const_iterator l_iter = links.begin();
				l_iter != links.end(); l_iter++)
		{
			// get the collisions of the current link
            const physics::Collision_V collisions = l_iter->get()->GetCollisions();

			// loop through all the collision
			for (physics::Collision_V::const_iterator c_iter = collisions.begin();
					c_iter != collisions.end(); c_iter++)
            {
				// check if collision belongs to the liquid model
				if ((*m_iter) == this->liquidSpheres)
				{
					this->allLiquidParticles_S.insert(c_iter->get());
				}

				// if the collision is without physical contact then add it to the map
				if (c_iter->get()->GetSurface()->collideWithoutContact)
				{
					// specific no-contact collision
					if (c_iter->get()->GetName() == "mug_event_collision")
					{
						this->eventCollisionMug = c_iter->get();
					}
					else if (c_iter->get()->GetName() == "fore_finger_event_collision")
					{
						this->eventCollisionForeFinger = c_iter->get();
					}
					else if (c_iter->get()->GetName() == "thumb_event_collision")
					{
						this->eventCollisionThumb = c_iter->get();
					}
					else
                    {
						// insert collision into set
                        this->eventCollisions_S.insert(c_iter->get());

                        // init the event collision with an empty set (models that are in collision with)
//                        this->prevEvCollToModelNames_S_M[c_iter->get()] = std::set<std::string>();

                        // init the event coll to particle names map with empty sets of strings
                        this->eventCollToSetOfParticleNames_M[c_iter->get()] = std::set<std::string>();
					}
				}
			}
		}
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

//	process_thread_group.create_thread(boost::bind(&PostProcess::PublishAndWriteTFData, this));
	process_thread_group.create_thread(boost::bind(&PostProcess::WriteSemanticData, this));

	// wait for all the threads to finish work
	process_thread_group.join_all();

	// clear/refresh the contact manager, otherwise data from past contacts are still present
    this->contactManagerPtr->Clear();
}

//////////////////////////////////////////////////
void PostProcess::PublishAndWriteTFData()
{
	// TODO, static or class member?
	// broadcaster to send the information
	static tf::TransformBroadcaster transf_br;

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

	// broadcast the transforms
	transf_br.sendTransform(stamped_transforms);

	// write the transformations to the data base
	PostProcess::WriteTFData(stamped_transforms);

}

//////////////////////////////////////////////////
void PostProcess::WriteTFData(const std::vector<tf::StampedTransform>& _stamped_transforms)
{
	// create bson transform object
	std::vector<BSONObj> transforms_bo;

	// get the timestamp im ms and date format
	Date_t stamp_ms = this->world->GetSimTime().nsec / 1000000.0 + this->world->GetSimTime().sec * 1000.0;

	// iterate through the stamped tranforms
	for (std::vector<tf::StampedTransform>::const_iterator st_iter = _stamped_transforms.begin();
			st_iter != _stamped_transforms.end(); ++st_iter)
	{
		// check if the transform should be written
		if(PostProcess::ShouldWriteTransform(st_iter))
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
	ScopedDbConnection scoped_connection("localhost");

	// insert document object into the database
	scoped_connection->insert(this->dbCollName + ".tf", BSON("transforms" << transforms_bo
														<< "__recorded" << stamp_ms
														<< "__topic" << "/tf_sim"));

	// let the pool know the connection is done
	scoped_connection.done();
}

//////////////////////////////////////////////////
bool PostProcess::ShouldWriteTransform(std::vector<tf::StampedTransform>::const_iterator& _curr_st_iter)
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

//////////////////////////////////////////////////
void PostProcess::WriteSemanticData()
{
    // compute simulation time in milliseconds
    const double timestamp_ms = this->world->GetSimTime().Double();

    // get all the contacts from the physics engine
    const std::vector<physics::Contact*> all_contacts = this->contactManagerPtr->GetContacts();

	// marks if there is a difference between the previous and current step
	bool diff_detected = false;

    // set finger contacts flags to false
    bool fore_finger_contact = false;
    bool thumb_contact = false;

    // grasp collisions
    physics::Collision *grasp_coll1, *grasp_coll2;

    // current grasped model name
    std::string curr_grasped_model;

    // TODO check until the pouring is finished
    // curr poured particles
    int prev_poured_particles_nr = this->totalPouredParticles_S.size();

    // TODO eventually change pouring so we compare the two sets?
    std::set<physics::Collision*> curr_poured_particles_S;


    //TODO test to remove map
    std::set<std::pair<std::string, std::string> > curr_ev_contact_model_pair_S;

    // current map of event collisions to set of models names in contact with
    std::map< physics::Collision*, std::set<std::string> > curr_ev_coll_to_model_names_S_M;

    // current map of event collisions to set of models names
    std::map< physics::Collision*, std::set<std::string> > curr_ev_coll_to_particle_names_S_M;

    // init current maps with the empty set
    for(std::set<physics::Collision*>::const_iterator s_iter = this->eventCollisions_S.begin();
            s_iter != this->eventCollisions_S.end(); s_iter++)
    {
//        curr_ev_coll_to_model_names_S_M[*s_iter] = std::set<std::string>();
        curr_ev_coll_to_particle_names_S_M[*s_iter] = std::set<std::string>();
    }

    ////////////////////////////////////////////////////////////////
    ////////////// Loop through all the contacts to set current state
    for (unsigned int i = 0; i < all_contacts.size(); i++)
    {
        // collision 1 and 2 of the contact
        physics::Collision* coll1 = all_contacts.at(i)->collision1;
        physics::Collision* coll2 = all_contacts.at(i)->collision2;


        ////////////// Supporting Collisions
        // check if collision 1 belongs to the event collision set
        if (this->eventCollisions_S.find(coll1) != this->eventCollisions_S.end())
        {
            // insert contact model name into the set
            curr_ev_coll_to_model_names_S_M[coll1].insert(coll2->GetParentModel()->GetName());

            curr_ev_contact_model_pair_S.insert(std::pair<std::string, std::string>(
            		coll1->GetParentModel()->GetName(), coll2->GetParentModel()->GetName()));
        }
        // check if collision 2 belongs to the event collision set
        else if(this->eventCollisions_S.find(coll2) != this->eventCollisions_S.end())
        {
            // insert contact model name into the set
            curr_ev_coll_to_model_names_S_M[coll2].insert(coll1->GetParentModel()->GetName());

            curr_ev_contact_model_pair_S.insert(std::pair<std::string, std::string>(
            		coll2->GetParentModel()->GetName(), coll1->GetParentModel()->GetName()));
        }

        // TODO use else if ?
        /////////////// Grasping
        // check grasping with both fingers, this might fail if the sensor is in contact with multiple models
        if (coll1 == this->eventCollisionForeFinger || coll2 == this->eventCollisionForeFinger)
        {
            // if one of the collisions is the fore finger, set contact flag to true, and save both collisions
            fore_finger_contact = true;
            grasp_coll1 = coll1;
            grasp_coll2 = coll2;
        }
        else if (coll1 == this->eventCollisionThumb || coll2 == this->eventCollisionThumb)
        {
            // if one of the collisions is the thumb, set contact flag to true, and save both collisions
            thumb_contact = true;
            grasp_coll1 = coll1;
            grasp_coll2 = coll2;
        }

        // TODO Pour Pancake events
        ////////////// Pouring Action
        // look into pouring until the pancake is created
        if(!this->pancakeCreated)
        {
            // check for the currently poured particles
            if (coll1 == this->eventCollisionMug || coll2 == this->eventCollisionMug)
            {
                // check if coll1 or 2 belongs to the liquid
                if (coll1->GetModel() == this->liquidSpheres)
                {
                    // add to poured set, which also checks for duplicates
                    this->totalPouredParticles_S.insert(coll1);
                }
                else if (coll2->GetModel() == this->liquidSpheres)
                {
                    // add to poured set, which also checks for duplicates
                    this->totalPouredParticles_S.insert(coll2);
                }
            }

            ////////////// Poured Particles Collisions
            // check if one collision is a poured particle and the other belongs to the eventCollisions
            if(this->totalPouredParticles_S.find(coll1) != this->totalPouredParticles_S.end() &&
                    this->eventCollisions_S.find(coll2) != this->eventCollisions_S.end())
            {
                // add the model name to the set coll2's set
                curr_ev_coll_to_model_names_S_M[coll2].insert(coll1->GetModel()->GetName());

                // add the particle collision name to the coll2's set
                curr_ev_coll_to_particle_names_S_M[coll2].insert(coll1->GetName());
            }
            else if(this->totalPouredParticles_S.find(coll2) != this->totalPouredParticles_S.end() &&
                    this->eventCollisions_S.find(coll1) != this->eventCollisions_S.end())
            {
                // add the model name to the set coll1's set
                curr_ev_coll_to_model_names_S_M[coll1].insert(coll2->GetModel()->GetName());

                // add the particle collision name to the coll1's set
                curr_ev_coll_to_particle_names_S_M[coll1].insert(coll2->GetName());
            }
        }

        ////////////// Flipping Action
        // create pancake when the spatula is grasped, save all particles belonging to the pancake
        else if(this->pancakeCreated)
        {
            ////////////// Pancake Particles Collisions
            // check if one collision is a poured particle and the other belongs to the eventCollisions
            if(this->pancakeCollision_S.find(coll1) != this->pancakeCollision_S.end() &&
                    this->eventCollisions_S.find(coll2) != this->eventCollisions_S.end())
            {
                // add the model name to the set coll2's set
                curr_ev_coll_to_model_names_S_M[coll2].insert(coll1->GetModel()->GetName());

                // add the particle collision name to the coll2's set
                curr_ev_coll_to_particle_names_S_M[coll2].insert(coll1->GetName());
            }
            else if(this->pancakeCollision_S.find(coll2) != this->pancakeCollision_S.end() &&
                    this->eventCollisions_S.find(coll1) != this->eventCollisions_S.end())
            {
                // add the model name to the set coll1's set
                curr_ev_coll_to_model_names_S_M[coll1].insert(coll2->GetModel()->GetName());

                // add the particle collision name to the coll1's set
                curr_ev_coll_to_particle_names_S_M[coll1].insert(coll2->GetName());
            }
        }
    }



    ////////////////////////////////////////////////////////////////
    ////////////// Compare current state with previous one

    // Check if the current grasp has changed
    diff_detected = PostProcess::CheckCurrentGrasp(
    		timestamp_ms, fore_finger_contact, thumb_contact, grasp_coll1, grasp_coll2);

    // Check if difference between the event collisions have appeared
    diff_detected = PostProcess::CheckCurrentEventCollisions(
    		timestamp_ms, curr_ev_contact_model_pair_S);

	// Check if diff between particles leaving the mug appeared
	diff_detected = PostProcess::CheckLiquidTransferEvent(
			timestamp_ms, prev_poured_particles_nr);


//    // TODO Pour Pancake events
//    // check if new particle has been poured
//    if (this->totalPouredParticles_S.size() > prev_poured_particles_nr )
//    {
//        diff_detected = true;
//    }
//
//    // check if poured particle collision appeared/changed
//    if (curr_ev_coll_to_particle_names_S_M != this->eventCollToSetOfParticleNames_M)
//    {
//        diff_detected = true;;
//        this->eventCollToSetOfParticleNames_M = curr_ev_coll_to_particle_names_S_M;
//    }
//
//    // save the particles belonging to the pancake
//    if (!this->pancakeCreated && curr_grasped_model == "Spatula")
//    {
//        ////////////// Loop through all the contacts
//        for (unsigned int i = 0; i < all_contacts.size(); i++)
//        {
//            // collision 1 and 2 of the contact
//            physics::Collision* coll1 = all_contacts.at(i)->collision1;
//            physics::Collision* coll2 = all_contacts.at(i)->collision2;
//
//            // save the particles belonging to the pancake
//            if ((coll1->GetName() == "pancake_maker_event_collision")
//                    && (coll2->GetModel() == this->liquidSpheres))
//            {
//                this->pancakeCollision_S.insert(coll2);
//            }
//            else if((coll2->GetName() == "pancake_maker_event_collision")
//                    && (coll1->GetModel() == this->liquidSpheres))
//            {
//                this->pancakeCollision_S.insert(coll1);
//            }
//        }
//
//        diff_detected = true;
//        this->pancakeCreated = true;
//    }


    ////////////////////////////
    // Output at detected difference
//    if(diff_detected)
//    {
//        diff_detected = false;
//
//        ////////////////////////////
//        // Event info
//        for(std::map<physics::Collision*, std::set<std::string> >::const_iterator m_iter = curr_ev_coll_to_model_names_S_M.begin();
//            m_iter != curr_ev_coll_to_model_names_S_M.end(); m_iter++)
//        {
//            std::cout << m_iter->first->GetParentModel()->GetName() << " --> ";
//
//            for(std::set<std::string>::const_iterator s_iter = m_iter->second.begin();
//                s_iter != m_iter->second.end(); s_iter++)
//            {
//                std::cout << *s_iter << "; ";
//            }
//            std::cout << std::endl;
//        }
//
//        ////////////////////////////
//        // Grasped model
//        std::cout << "grasp --> " << curr_grasped_model<< ";" << std::endl;
//
//
//        ////////////////////////////
//        // Poured particles
//        std::cout << "poured --> " << this->pouredLiquidCollisions_S.size() <<
//                     "/" << this->allLiquidCollisions_S.size() << ";" << std::endl;
//
//        ////////////////////////////
//        // Pancake size
//        std::cout << "pancake size --> " << this->pancakeCollision_S.size() << " particles" << std::endl;
//
//        for (std::set<physics::Collision*>::const_iterator c_iter = this->pancakeCollision_S.begin();
//             c_iter != this->pancakeCollision_S.end(); c_iter++)
//        {
//            //TODO why is the copy required? try (*c_iter)
//            physics::Collision* c = *c_iter;
//        }
//
//
//        ////////////////////////////
//        // Pouring Info before pancake created
//        if(!this->pancakeCreated)
//        {
//            // Poured particles supported by
//            for(std::map<physics::Collision*, std::set<std::string> >::const_iterator m_iter = curr_ev_coll_to_particle_names_S_M.begin();
//                m_iter != curr_ev_coll_to_particle_names_S_M.end(); m_iter++)
//            {
//                std::cout << "\t" << m_iter->first->GetParentModel()->GetName() << " --> ";
//
//                // Write only nr of particles at the moment
//                std::cout << m_iter->second.size() << " particles;" <<std::endl;
//                for(std::set<std::string>::const_iterator s_iter = m_iter->second.begin();
//                    s_iter != m_iter->second.end(); s_iter++)
//                {
//                    //std::cout << *s_iter << "; ";
//                }
//                //std::cout << std::endl;
//            }
//        }
//
//        ////////////////////////////
//        // Pancake Info
//        else if(this->pancakeCreated)
//        {
//            // Pancake particles supported by
//            for(std::map<physics::Collision*, std::set<std::string> >::const_iterator m_iter = curr_ev_coll_to_particle_names_S_M.begin();
//                m_iter != curr_ev_coll_to_particle_names_S_M.end(); m_iter++)
//            {
//                std::cout << "\t" << m_iter->first->GetParentModel()->GetName() << " --> ";
//
//                // Write only nr of particles at the moment
//                std::cout << m_iter->second.size() << " pancake particles;" <<std::endl;
//                for(std::set<std::string>::const_iterator s_iter = m_iter->second.begin();
//                    s_iter != m_iter->second.end(); s_iter++)
//                {
//                    //std::cout << *s_iter << "; ";
//                }
//                //std::cout << std::endl;
//            }
//        }
//        std::cout <<"-------------------------------------------------------------------ts: "<< timestamp_ms << " ms"<< std::endl;
//    }
}

//////////////////////////////////////////////////
bool PostProcess::CheckCurrentGrasp(
		const double _timestamp_ms,
		bool _fore_finger_contact,
		bool _thumb_contact,
		physics::Collision *_grasp_coll1,
		physics::Collision *_grasp_coll2)
{
	// marks if there is a difference between the previous and current step
	bool diff_detected = false;

    // current grasped model name
    std::string curr_grasped_model;

    // check for grasp / both fingers are in contact
    if (_fore_finger_contact && _thumb_contact)
    {
        // if coll1 belongs to the hand model, then coll2 is the grasped model
        if (_grasp_coll1->GetParentModel()->GetName() == "Hand")
        {
            curr_grasped_model = _grasp_coll2->GetParentModel()->GetName();
        }
        else
        {
            curr_grasped_model = _grasp_coll1->GetParentModel()->GetName();
        }
    }

    // check for difference between current and past grasp
    if (curr_grasped_model != this->prevGraspedModel)
    {
       // set diff flag to true and the prev grasp value takes the current one
       diff_detected = true;
       this->prevGraspedModel = curr_grasped_model;

       // the name of the grasping event
       std::string grasp_ev_name = "Grasp" + curr_grasped_model;

       // Check if grasp event has been initialized
       // using flag because curr grasped model could be empty and detected as an event
       if(!this->graspInit)
       {
    	   this->graspInit = true;

    	   // create the contact GzEvent
    	   this->graspGzEvent = new GzEvent(
    			   grasp_ev_name, "GraspingSomething", "objectActedOn", _timestamp_ms);

    	   // add grasped object
    	   this->graspGzEvent->AddObject(this->nameToEventObj_M[curr_grasped_model]);
       }
       else
       {
    	   if(this->graspGzEvent->IsOpen())
    	   {
    		   // ending context
    		   this->graspGzEvent->End(_timestamp_ms);

    		   // add grasp event to the map of all events
    		   this->nameToEvents_M[grasp_ev_name].push_back(this->graspGzEvent);
    	   }
    	   else
    	   {
    		   // create the contact GzEvent
    		   this->graspGzEvent = new GzEvent(
    				   grasp_ev_name, "&knowrob_sim;", "GraspingSomething",
    				   "knowrob:", "objectActedOn", _timestamp_ms);

    		   // add grasped object
    		   this->graspGzEvent->AddObject(this->nameToEventObj_M[curr_grasped_model]);
    	   }
       }
    }

    return diff_detected;
}

//////////////////////////////////////////////////
bool PostProcess::CheckCurrentEventCollisions(
		const double _timestamp_ms,
		std::set<std::pair<std::string, std::string> > &_curr_ev_contact_model_pair_S)
{
	// marks if there is a difference between the previous and current step
	bool diff_detected = false;

	// the symmetric difference will be added to this set
	std::set<std::pair<std::string, std::string> > symmetric_diff_S;

	// computing the symmetric diff between the curr and prev set
	std::set_symmetric_difference(
			_curr_ev_contact_model_pair_S.begin(),
			_curr_ev_contact_model_pair_S.end(),
			this->prevEvContactModelPair_S.begin(),
			this->prevEvContactModelPair_S.end(),
			std::inserter(symmetric_diff_S,symmetric_diff_S.end()));

	// check if there are any differences
	if (symmetric_diff_S.size() > 0)
	{
		// set diff flag to true and set the prev values to the current one
		bool diff_detected = true;
		this->prevEvContactModelPair_S = _curr_ev_contact_model_pair_S;

		// iterate through all the collising event models
		for(std::set<std::pair<std::string, std::string> >::const_iterator m_iter = symmetric_diff_S.begin();
				m_iter != symmetric_diff_S.end(); m_iter++)
		{

			// TODO hardcoded check for duplicates of spatula<->pancake maker,
			// aka ignore spatula->pancakemaker, use pancakemaker->spatula
			if(m_iter->first == "Spatula" && m_iter->second == "PancakeMaker")
			{
				continue;
			}

			// set name of the context first models + second model in contact
			std::string contact_ev_name = "Contact" + m_iter->first + m_iter->second;


			// if event does not exist
			// TODO add two way detection of contacts, store somewhere the pairs in contact
			if(!this->nameToEvents_M.count(contact_ev_name))
			{
				// create local contact GzEvent
				hand_sim::GzEvent* contact_event = new hand_sim::GzEvent(
						contact_ev_name, "&knowrob_sim;", "TouchingSituation",
						"knowrob_sim:", "inContact", _timestamp_ms);

				// add the two objects in contact
				contact_event->AddObject(this->nameToEventObj_M[m_iter->first]);

				contact_event->AddObject(this->nameToEventObj_M[m_iter->second]);

				// add local event to the map
				this->nameToEvents_M[contact_ev_name].push_back(contact_event);
			}
			else
			{
				// if the contact exists and it is open, end it
				if(this->nameToEvents_M[contact_ev_name].back()->IsOpen())
				{
					// end contact event
					this->nameToEvents_M[contact_ev_name].back()->End(_timestamp_ms);
				}
				else
				{
					// create local contact GzEvent
					hand_sim::GzEvent* contact_event = new hand_sim::GzEvent(
							contact_ev_name, "&knowrob_sim;", "TouchingSituation",
							"knowrob_sim:", "inContact", _timestamp_ms);

					// add the two objects in contact
					contact_event->AddObject(this->nameToEventObj_M[m_iter->first]);

					contact_event->AddObject(this->nameToEventObj_M[m_iter->second]);

					// add local event to the map
					this->nameToEvents_M[contact_ev_name].push_back(contact_event);				}
			}
		}
	}

	return diff_detected;
}

//////////////////////////////////////////////////
bool PostProcess::CheckLiquidTransferEvent(
		const double _timestamp_ms,
		int _prev_poured_particle_nr)
{
	// marks if there is a difference between the previous and current step
	bool diff_detected = false;


    // Check if new particle has been poured
    if (this->totalPouredParticles_S.size() > _prev_poured_particle_nr )
    {
        diff_detected = true;

        // check if the event doesn't exist (first particles leaving)
		if(!this->nameToEvents_M.count("LiquidTransfer"))
		{
		    std::cout << "*Creating* LiquidTransfer event at " << _timestamp_ms << std::endl;

			// add local event to the map
			this->nameToEvents_M["LiquidTransfer"].push_back(new hand_sim::GzEvent(
					"LiquidTransfer", "&knowrob_sim;", "LiquidTransfer", _timestamp_ms));
		}
		else // check if last particle left
		{
			if(this->totalPouredParticles_S.size() == this->allLiquidParticles_S.size())
			{
			    std::cout << "*End* LiquidTransfer event at " << _timestamp_ms << std::endl;
				// end liquid transfer event
				this->nameToEvents_M["LiquidTransfer"].back()->End(_timestamp_ms);
			}
		}


    }

	return diff_detected;
}

//////////////////////////////////////////////////
void PostProcess::LogCheckWorker()
{
	// flag to stop the while loop
	bool log_play_finished = false;

	// loop until the log has finished playing
	while(!log_play_finished)
	{
		// if the world is paused
		if(this->world && this->world->IsPaused() && !this->pauseMode)
		{
			// check that no manual pause happened!
			std::string sdfString;
			if(!util::LogPlay::Instance()->Step(sdfString))
			{
				log_play_finished = true;
				std::cout << "!!! Last recorded step at " << this->world->GetSimTime().Double() << ", terminating simulation.."<< std::endl;
			}
			else
			{
				std::cout << "!!! Manual pause, every time this msg appears one step of the simulation is lost.. "  << std::endl;
			}
		}
		// loop sleep
		usleep(2000000);
	}

	// terminate simulation
	PostProcess::TerminateSimulation();
}

//////////////////////////////////////////////////
void PostProcess::TerminateSimulation()
{
	// close main GzEvent
	this->nameToEvents_M["Main"].back()->End(this->world->GetSimTime().Double());

	// close all open events
	PostProcess::EndActiveEvents();

	// Concatenate timelines with short disconnections
	PostProcess::JoinShortDisconnections();

	// write events as belief state contexts
	PostProcess::WriteContexts();

	// write events to timeline file
	PostProcess::WriteTimelines();

	// shutting down ros
	ros::shutdown();

	// finish the simulation
	gazebo::shutdown();
}

//////////////////////////////////////////////////
void PostProcess::EndActiveEvents()
{
	// iterate through the map
	for(std::map<std::string, std::list<hand_sim::GzEvent*> >::const_iterator m_it = this->nameToEvents_M.begin();
			m_it != this->nameToEvents_M.end(); m_it++)
	{
		// iterate through the events with the same name
		for(std::list<GzEvent*>::const_iterator ev_it = m_it->second.begin();
				ev_it != m_it->second.end(); ev_it++)
		{
			// if event still open end it at the end time
			if((*ev_it)->IsOpen())
			{
				(*ev_it)->End(this->world->GetSimTime().Double());
			}
		}
	}
}

//////////////////////////////////////////////////
void PostProcess::JoinShortDisconnections()
{
	// iterate through the map
	for(std::map<std::string, std::list<hand_sim::GzEvent*> >::iterator m_it = this->nameToEvents_M.begin();
			m_it != this->nameToEvents_M.end(); m_it++)
	{
		// iterate through the events with the same name
		for(std::list<GzEvent*>::iterator ev_it = m_it->second.begin();
				ev_it != m_it->second.end(); /*increment in the loop*/)
		{
			// save current event for comparison
			std::list<GzEvent*>::iterator curr_ev = ev_it;

			// increment current iteration (next event)
			ev_it++;

			// check that the next value is not the last
			if(ev_it != m_it->second.end())
			{
				if((*ev_it)->GetStartTime() - (*curr_ev)->GetEndTime() < 0.2)
				{
					// set the next values start time
					(*ev_it)->SetStartTime((*curr_ev)->GetStartTime());

					// remove current value from list
					m_it->second.erase(curr_ev);
				}
			}
		}
	}
}

//////////////////////////////////////////////////
void PostProcess::WriteContexts()
{
	// initialize the beliefstate
	this->beliefStateClient = new beliefstate_client::BeliefstateClient("bs_client");

	// register the OWL namespace
	this->beliefStateClient->registerOWLNamespace("knowrob_sim", "http://knowrob.org/kb/knowrob_sim.owl#");

	// iterate through the map
	for(std::map<std::string, std::list<hand_sim::GzEvent*> >::const_iterator m_it = this->nameToEvents_M.begin();
			m_it != this->nameToEvents_M.end(); m_it++)
	{
		// iterate through the events with the same name
		for(std::list<hand_sim::GzEvent*>::const_iterator ev_it = m_it->second.begin();
				ev_it != m_it->second.end(); ev_it++)
		{
			// create local belief state context
			beliefstate_client::Context* curr_ctx;

			// open belief state context
			curr_ctx = new beliefstate_client::Context(this->beliefStateClient,
					(*ev_it)->GetName(), (*ev_it)->GetClassNamespace(), (*ev_it)->GetClass(), (*ev_it)->GetStartTime());

			// get the objects of the event
			std::vector<GzEventObj*> curr_objects = (*ev_it)->GetObjects();

			// iterate through the objects
			for(std::vector<GzEventObj*>::const_iterator ob_it = curr_objects.begin();
					ob_it != curr_objects.end(); ob_it++)
			{
				// add object to the context
				curr_ctx->addObject(this->nameToBsObject_M[(*ob_it)->GetName()],
						(*ev_it)->GetPropertyNamespace() + (*ev_it)->GetProperty());
			}

			// end belief state context
			curr_ctx->end(true, (*ev_it)->GetEndTime());
		}
	}

	// export belief state client
	this->beliefStateClient->exportFiles("sim_data");

}

//////////////////////////////////////////////////
void PostProcess::WriteTimelines()
{

	std::ofstream timeline_file;
	timeline_file.open ("timeline.html");

	timeline_file <<
			"<html>\n"
			"<script type=\"text/javascript\" src=\"https://www.google.com/jsapi?autoload={'modules':[{'name':'visualization',\n"
			"       'version':'1','packages':['timeline']}]}\"></script>\n"
			"<script type=\"text/javascript\">\n"
			"google.setOnLoadCallback(drawChart);\n"
			"\n"
			"function drawChart() {\n"
			"  var container = document.getElementById('sim_timeline_ex');\n"
			"\n"
			"  var chart = new google.visualization.Timeline(container);\n"
			"\n"
			"  var dataTable = new google.visualization.DataTable();\n"
			"\n"
			"  dataTable.addColumn({ type: 'string', id: 'Event' });\n"
			"  dataTable.addColumn({ type: 'number', id: 'Start' });\n"
			"  dataTable.addColumn({ type: 'number', id: 'End' });\n"
			"\n"
			"  dataTable.addRows([\n";

	// iterate through the map
	for(std::map<std::string, std::list<hand_sim::GzEvent*> >::const_iterator m_it = this->nameToEvents_M.begin();
			m_it != this->nameToEvents_M.end(); m_it++)
	{
		// iterate through the events with the same name
		for(std::list<GzEvent*>::const_iterator ev_it = m_it->second.begin();
				ev_it != m_it->second.end(); ev_it++)
		{
			// add all events to the timeline file
			timeline_file << "    [ '" << (*ev_it)->GetName() << "', "
					<< (*ev_it)->GetStartTime() * 1000 << ", "
					<< (*ev_it)->GetEndTime() * 1000 << "],\n";
		}
	}

	timeline_file <<
			"  ]);\n"
			"\n"
			"  chart.draw(dataTable);\n"
			"}\n"
			"</script>\n"
			"<div id=\"sim_timeline_ex\" style=\"width: 1300px; height: 900px;\"></div>\n"
			"\n"
			"</html>";

	timeline_file.close();
}

//////////////////////////////////////////////////
void PostProcess::DummyContactsCallback(ConstContactsPtr& _msg)
{
}
