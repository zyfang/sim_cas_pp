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
//#include <boost/chrono.hpp>
#include <boost/thread.hpp>

using namespace gazebo;
using namespace mongo;
using namespace beliefstate_client;

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(PostProcess)

//////////////////////////////////////////////////
PostProcess::PostProcess()
{
	// intialize ROS
	int argc = 0;
	char** argv = NULL;
	ros::init(argc, argv, "post_process");

	// initialize the beliefstate
	this->beliefStateClient = new BeliefstateClient("bs_client");

	// register the OWL namespace
	this->beliefStateClient->registerOWLNamespace("sim","http://some-namespace.org/#");

}

//////////////////////////////////////////////////
PostProcess::~PostProcess()
{
	delete this->contactManagerPtr;
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

	// set the thresholds for logging tf transormations
	this->tfVectDistThresh = 0.001;
	this->tfAngularDistThresh = 0.1;
	this->tfDurationThresh = 100; // ms


	std::cout << "******** MONGO LOG PLUGIN LOADED *********" << std::endl;
    std::cout << "\nWorld: " << this->worldName << std::endl;
    std::cout << "Database.Collection: " << this->dbCollName << std::endl;
    std::cout << "\nOptions: " << std::endl;
    std::cout << "   -w  \t\t World name to be loaded." << std::endl;
    std::cout << "   -db \t\t Database.Collection name for the data to be written to." << std::endl;
}

//////////////////////////////////////////////////
void PostProcess::Init()
{
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
        boost::bind(&PostProcess::GetEventCollisions, this));
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
    this->liquidSpheres = this->world->GetModel("liquid_spheres");

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

	// start the main simulation context
	this->mainContext = new Context(
			this->beliefStateClient, "PourFlipEpisode", "&sim;", "MainTimelineClass", 0);
}

//////////////////////////////////////////////////
void PostProcess::GetEventCollisions()
{
	// loop through all the models to see which have event collisions
	for(physics::Model_V::const_iterator m_iter = this->models.begin();
			m_iter != this->models.end(); m_iter++)
	{

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
				if (m_iter->get()->GetName() == "liquid_spheres")
				{
					this->allLiquidCollisions_S.insert(c_iter->get());
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

                        // init the event coll to model names map with empty sets of strings
                        this->eventCollToSetOfModelNames_M[c_iter->get()] = std::set<std::string>();

                        // init the event coll to particle names map with empty sets of strings
                        this->eventCollToSetOfParticleNames_M[c_iter->get()] = std::set<std::string>();
					}
				}
			}
		}
	}

    // Listen to the update event. This event is broadcast every simulation iteration.
    this->eventConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&PostProcess::UpdateDB, this));
}

//////////////////////////////////////////////////
void PostProcess::UpdateDB()
{
//    boost::chrono::system_clock::time_point parallel_start = boost::chrono::system_clock::now();

	// group of threads for processing the data in parallel
	boost::thread_group process_thread_group;

//	process_thread_group.create_thread(boost::bind(&PostProcess::WriteEventData, this));
//	process_thread_group.create_thread(boost::bind(&PostProcess::WriteRawData, this));
//	process_thread_group.create_thread(boost::bind(&PostProcess::WriteParticleEventData, this));
	// TODO separate this two? add flag in the method to publish or not
	process_thread_group.create_thread(boost::bind(&PostProcess::PublishAndWriteTFData, this));
//	process_thread_group.create_thread(boost::bind(&PostProcess::WriteSemanticData, this));

	process_thread_group.create_thread(boost::bind(&PostProcess::DummyUpdate, this));

	// wait for all the threads to finish work
	process_thread_group.join_all();


	// clear the contact manager, otherwise data from past contacts are still present
    this->contactManagerPtr->Clear();

//    boost::chrono::duration<double> parallel_dur = boost::chrono::system_clock::now() - parallel_start;
//    std::cout << "Parallel TOTAL: " << parallel_dur.count() << " seconds\n";
}

//////////////////////////////////////////////////
void PostProcess::DummyUpdate()
{
	std::cout << "raw" << this->world->GetSimTime() << std::endl;

	std::cout << "double" << this->world->GetSimTime().Double() << std::endl;

	std::cout << "ms" << this->world->GetSimTime().nsec / 1000000.0 + this->world->GetSimTime().sec * 1000.0 << std::endl;

	std::cout << "**********************************" << std::endl;
}

//////////////////////////////////////////////////
void PostProcess::WriteEventData()
{
//    boost::chrono::system_clock::time_point ev_start = boost::chrono::system_clock::now();

	// set diff detected flag to false
	bool diff_detected = false;

    // compute simulation time in nanoseconds
    const long long int timestamp = this->world->GetSimTime().nsec + this->world->GetSimTime().sec * 1e9;

    // get all the contacts from the physics engine
    const std::vector<physics::Contact*> contacts = this->contactManagerPtr->GetContacts();

    // current map of event collisions to set of models names
    std::map< physics::Collision*, std::set<std::string> > event_coll_to_set_of_model_names_M;

    // init current map with the supporting event collisions, and an empty set
    for(std::set<physics::Collision*>::const_iterator s_iter = this->eventCollisions_S.begin();
    		s_iter != this->eventCollisions_S.end(); s_iter++)
    {
        event_coll_to_set_of_model_names_M[*s_iter] = std::set<std::string>();
    }

    // current map of event collisions to set of models names
    std::map< physics::Collision*, std::set<std::string> > event_coll_to_set_of_particle_names_M;

    // init current map with the supporting event collisions, and an empty set
    for(std::set<physics::Collision*>::const_iterator s_iter = this->eventCollisions_S.begin();
            s_iter != this->eventCollisions_S.end(); s_iter++)
    {
        event_coll_to_set_of_particle_names_M[*s_iter] = std::set<std::string>();
    }

    // TODO check until the pouring is finished
    // curr poured particles
    int prev_poured_particles_nr = this->pouredLiquidCollisions_S.size();

    // current grasped model name
    std::string grasped_model_name;

    // set finger contacts flags to false
    bool fore_finger_contact = false;
    bool thumb_contact = false;

    // grasp collisions
    physics::Collision *grasp_coll1, *grasp_coll2;

    ////////////// Loop through all the contacts
    // set current states
    for (unsigned int i = 0; i < contacts.size(); i++)
    {
        // collision 1 and 2 of the contact
        physics::Collision* coll1 = contacts.at(i)->collision1;
        physics::Collision* coll2 = contacts.at(i)->collision2;


        ////////////// Supporting Collisions
        // check if collision 1 belongs to the event collision set
        if (this->eventCollisions_S.find(coll1) != this->eventCollisions_S.end())
        {
            // insert contact model name into the set
            event_coll_to_set_of_model_names_M[coll1].insert(coll2->GetParentModel()->GetName());
        }
        // check if collision 2 belongs to the event collision set
        else if(this->eventCollisions_S.find(coll2) != this->eventCollisions_S.end())
        {
            // insert contact model name into the set
            event_coll_to_set_of_model_names_M[coll2].insert(coll1->GetParentModel()->GetName());
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
//        ////////////// Pouring Action
//        // look into pouring until the pancake is created
//        if(!this->pancakeCreated)
//        {
//            // check for the currently poured particles
//            if (coll1 == this->eventCollisionMug || coll2 == this->eventCollisionMug)
//            {
//                // check if coll1 or 2 belongs to the liquid
//                if (coll1->GetModel()->GetName() == "liquid_spheres")
//                {
//                    // add to poured set, which also checks for duplicates
//                    this->pouredLiquidCollisions_S.insert(coll1);
//                }
//                else if (coll2->GetModel()->GetName() == "liquid_spheres")
//                {
//                    // add to poured set, which also checks for duplicates
//                    this->pouredLiquidCollisions_S.insert(coll2);
//                }
//            }

//            ////////////// Poured Particles Collisions
//            // check if one collision is a poured particle and the other belongs to the eventCollisions
//            if(this->pouredLiquidCollisions_S.find(coll1) != this->pouredLiquidCollisions_S.end() &&
//                    this->eventCollisions_S.find(coll2) != this->eventCollisions_S.end())
//            {
//                // add the model name to the set coll2's set
//                event_coll_to_set_of_model_names_M[coll2].insert(coll1->GetModel()->GetName());

//                // add the particle collision name to the coll2's set
//                event_coll_to_set_of_particle_names_M[coll2].insert(coll1->GetName());
//            }
//            else if(this->pouredLiquidCollisions_S.find(coll2) != this->pouredLiquidCollisions_S.end() &&
//                    this->eventCollisions_S.find(coll1) != this->eventCollisions_S.end())
//            {
//                // add the model name to the set coll1's set
//                event_coll_to_set_of_model_names_M[coll1].insert(coll2->GetModel()->GetName());

//                // add the particle collision name to the coll1's set
//                event_coll_to_set_of_particle_names_M[coll1].insert(coll2->GetName());
//            }
//        }

//        ////////////// Flipping Action
//        // create pancake when the spatula is grasped, save all particles belonging to the pancake
//        else if(this->pancakeCreated)
//        {
//            ////////////// Pancake Particles Collisions
//            // check if one collision is a poured particle and the other belongs to the eventCollisions
//            if(this->pancakeCollision_S.find(coll1) != this->pancakeCollision_S.end() &&
//                    this->eventCollisions_S.find(coll2) != this->eventCollisions_S.end())
//            {
//                // add the model name to the set coll2's set
//                event_coll_to_set_of_model_names_M[coll2].insert(coll1->GetModel()->GetName());

//                // add the particle collision name to the coll2's set
//                event_coll_to_set_of_particle_names_M[coll2].insert(coll1->GetName());
//            }
//            else if(this->pancakeCollision_S.find(coll2) != this->pancakeCollision_S.end() &&
//                    this->eventCollisions_S.find(coll1) != this->eventCollisions_S.end())
//            {
//                // add the model name to the set coll1's set
//                event_coll_to_set_of_model_names_M[coll1].insert(coll2->GetModel()->GetName());

//                // add the particle collision name to the coll1's set
//                event_coll_to_set_of_particle_names_M[coll1].insert(coll2->GetName());
//            }
//        }
    }


    ////////////// Compare states
    // check for grasp
    if (fore_finger_contact && thumb_contact)
    {
        // if coll1 belongs to the hand model, then coll2 is the grasped model
        if (grasp_coll1->GetParentModel()->GetName() == "hit_hand")
        {
            grasped_model_name = grasp_coll2->GetParentModel()->GetName();
        }
        else
        {
            grasped_model_name = grasp_coll1->GetParentModel()->GetName();
        }
    }

    // check for difference between current and past grasp
    if (grasped_model_name != this->graspedModelName)
    {
        diff_detected = true;
        this->graspedModelName = grasped_model_name;
    }

    // check current and past collision states difference
    if (event_coll_to_set_of_model_names_M != this->eventCollToSetOfModelNames_M)
    {
        diff_detected = true;;
        this->eventCollToSetOfModelNames_M = event_coll_to_set_of_model_names_M;
    }

    // TODO Pour Pancake events
//    // check if new particle has been poured
//    if (this->pouredLiquidCollisions_S.size() > prev_poured_particles_nr )
//    {
//        diff_detected = true;
//    }

//    // check if poured particle collision appeared/changed
//    if (event_coll_to_set_of_particle_names_M != this->eventCollToSetOfParticleNames_M)
//    {
//        diff_detected = true;;
//        this->eventCollToSetOfParticleNames_M = event_coll_to_set_of_particle_names_M;
//    }

//    // save the particles belonging to the pancake
//    if (!this->pancakeCreated && grasped_model_name == "spatula")
//    {
//        ////////////// Loop through all the contacts
//        for (unsigned int i = 0; i < _contacts.size(); i++)
//        {
//            // collision 1 and 2 of the contact
//            physics::Collision* coll1 = _contacts.at(i)->collision1;
//            physics::Collision* coll2 = _contacts.at(i)->collision2;

//            // save the particles belonging to the pancake
//            if ((coll1->GetName() == "pancake_maker_event_collision")
//                    && (coll2->GetModel()->GetName() == "liquid_spheres"))
//            {
//                this->pancakeCollision_S.insert(coll2);
//            }
//            else if((coll2->GetName() == "pancake_maker_event_collision")
//                    && (coll1->GetModel()->GetName() == "liquid_spheres"))
//            {
//                this->pancakeCollision_S.insert(coll1);
//            }
//        }

//        diff_detected = true;
//        this->pancakeCreated = true;
//    }


    ////////////////////////////
    // Output at detected difference
    if (diff_detected)
    {
        diff_detected = false;
//        this->world->SetPaused(true);

        // document bson object builder
        BSONObjBuilder doc_bo_builder;

        ////////////////////////////
        // Event info
        for(std::map<physics::Collision*, std::set<std::string> >::const_iterator m_iter = event_coll_to_set_of_model_names_M.begin();
            m_iter != event_coll_to_set_of_model_names_M.end(); m_iter++)
        {
            BSONArrayBuilder support_arr_builder;

            std::cout << m_iter->first->GetParentModel()->GetName() << " --> ";
            for(std::set<std::string>::const_iterator s_iter = m_iter->second.begin();
                s_iter != m_iter->second.end(); s_iter++)
            {
                std::cout << *s_iter << "; ";

                support_arr_builder.append(*s_iter);
            }
            std::cout << std::endl;

            doc_bo_builder.append(m_iter->first->GetParentModel()->GetName(), support_arr_builder.arr());
        }

        ////////////////////////////
        // Grasped model
        std::cout << "grasp --> " << grasped_model_name<< ";" << std::endl;

        doc_bo_builder.append("grasp", grasped_model_name);

        ////////////////////////////
        // Poured particles
        std::cout << "poured --> " << this->pouredLiquidCollisions_S.size() <<
                     "/" << this->allLiquidCollisions_S.size() << ";" << std::endl;

        BSONObjBuilder pour_builder;

        BSONObjBuilder pancake_builder;

        pour_builder.append("total particles", (int) this->allLiquidCollisions_S.size());

        pour_builder.append("poured particles", (int) this->pouredLiquidCollisions_S.size());

        ////////////////////////////
        // Pancake size
        std::cout << "pancake size --> " << this->pancakeCollision_S.size() << " particles" << std::endl;

        pancake_builder.append("nr pancake particles", (int) this->pancakeCollision_S.size());

        BSONArrayBuilder pancake_arr_builder;

        // TODO change all interators to const iterator?
        for (std::set<physics::Collision*>::const_iterator c_iter = this->pancakeCollision_S.begin();
             c_iter != this->pancakeCollision_S.end(); c_iter++)
        {
            //TODO why is the copy required?
            physics::Collision* c = *c_iter;

            pancake_arr_builder.append(c->GetName());
        }

        pancake_builder.append("particle names",pancake_arr_builder.arr());

        ////////////////////////////
        // Pouring Info before pancake created
        if(!this->pancakeCreated)
        {
            BSONObjBuilder pour_support_builder;

            // Poured particles supported by
            for(std::map<physics::Collision*, std::set<std::string> >::const_iterator m_iter = event_coll_to_set_of_particle_names_M.begin();
                m_iter != event_coll_to_set_of_particle_names_M.end(); m_iter++)
            {
                BSONArrayBuilder support_arr_builder;

                std::cout << "\t" << m_iter->first->GetParentModel()->GetName() << " --> ";

                // Write only nr of particles at the moment
                std::cout << m_iter->second.size() << " particles;" <<std::endl;
                for(std::set<std::string>::const_iterator s_iter = m_iter->second.begin();
                    s_iter != m_iter->second.end(); s_iter++)
                {
                    //std::cout << *s_iter << "; ";
                    support_arr_builder.append(*s_iter);
                }
                //std::cout << std::endl;

                pour_support_builder.append(m_iter->first->GetParentModel()->GetName(), support_arr_builder.arr());
            }

            pour_builder.append("pour supports", pour_support_builder.obj());
        }

        ////////////////////////////
        // Pancake Info
        else if(this->pancakeCreated)
        {
            BSONObjBuilder pancake_support_builder;

            // Pancake particles supported by
            for(std::map<physics::Collision*, std::set<std::string> >::const_iterator m_iter = event_coll_to_set_of_particle_names_M.begin();
                m_iter != event_coll_to_set_of_particle_names_M.end(); m_iter++)
            {
                BSONArrayBuilder pancake_arr_builder;

                std::cout << "\t" << m_iter->first->GetParentModel()->GetName() << " --> ";

                // Write only nr of particles at the moment
                std::cout << m_iter->second.size() << " pancake particles;" <<std::endl;
                for(std::set<std::string>::const_iterator s_iter = m_iter->second.begin();
                    s_iter != m_iter->second.end(); s_iter++)
                {
                    pancake_arr_builder.append(*s_iter);
                    //std::cout << *s_iter << "; ";
                }
                //std::cout << std::endl;

                pancake_support_builder.append(m_iter->first->GetParentModel()->GetName(), pancake_arr_builder.arr());
            }

            pancake_builder.append("pancake supports", pancake_support_builder.obj());
        }
        std::cout <<"-------------------------------------------------------------------ts: "<< timestamp << std::endl;

    // TODO Pour Pancake events
        //doc_bo_builder.append("pour", pour_builder.obj());
        //doc_bo_builder.append("pancake", pancake_builder.obj());

        // create the document object
        doc_bo_builder.append("timestamp", timestamp);

        // insert document object into the database
//        this->mongoDBClientConnection.insert(this->dbCollName + ".events", doc_bo_builder.obj());

    	// use scoped connection
    	ScopedDbConnection scoped_connection("localhost");

    	// insert document object into the database
    	scoped_connection->insert(this->dbCollName + ".events", doc_bo_builder.obj());

    	// let the pool know the connection is done
    	scoped_connection.done();
    }

//    boost::chrono::duration<double> ev_sec = boost::chrono::system_clock::now() - ev_start;
//    std::cout << "EVENT: " << ev_sec.count() << " seconds\n";
}

//////////////////////////////////////////////////
void PostProcess::WriteParticleEventData()
{
    //    boost::chrono::system_clock::time_point ev_start = boost::chrono::system_clock::now();

		// set diff detected flag to false
		bool diff_detected = false;

        // compute simulation time in nanoseconds
        const long long int _timestamp = this->world->GetSimTime().nsec + this->world->GetSimTime().sec * 1e9;

        // get all the contacts from the physics engine
        const std::vector<physics::Contact*> _contacts = this->contactManagerPtr->GetContacts();

        // current map of event collisions to set of models names
        std::map< physics::Collision*, std::set<std::string> > event_coll_to_set_of_model_names_M;

        // init current map with the supporting event collisions, and an empty set
        for(std::set<physics::Collision*>::const_iterator s_iter = this->eventCollisions_S.begin();
                s_iter != this->eventCollisions_S.end(); s_iter++)
        {
            event_coll_to_set_of_model_names_M[*s_iter] = std::set<std::string>();
        }

        // current map of event collisions to set of models names
        std::map< physics::Collision*, std::set<std::string> > event_coll_to_set_of_particle_names_M;

        // init current map with the supporting event collisions, and an empty set
        for(std::set<physics::Collision*>::const_iterator s_iter = this->eventCollisions_S.begin();
                s_iter != this->eventCollisions_S.end(); s_iter++)
        {
            event_coll_to_set_of_particle_names_M[*s_iter] = std::set<std::string>();
        }

        // TODO check until the pouring is finished
        // curr poured particles
        int prev_poured_particles_nr = this->pouredLiquidCollisions_S.size();

        // current grasped model name
        std::string grasped_model_name;

        // set finger contacts flags to false
        bool fore_finger_contact = false;
        bool thumb_contact = false;

        // grasp collisions
        physics::Collision *grasp_coll1, *grasp_coll2;

        ////////////// Loop through all the contacts
        // set current states
        for (unsigned int i = 0; i < _contacts.size(); i++)
        {
            // collision 1 and 2 of the contact
            physics::Collision* coll1 = _contacts.at(i)->collision1;
            physics::Collision* coll2 = _contacts.at(i)->collision2;


            ////////////// Supporting Collisions
            // check if collision 1 belongs to the event collision set
            if (this->eventCollisions_S.find(coll1) != this->eventCollisions_S.end())
            {
                // insert contact model name into the set
                event_coll_to_set_of_model_names_M[coll1].insert(coll2->GetParentModel()->GetName());
            }
            // check if collision 2 belongs to the event collision set
            else if(this->eventCollisions_S.find(coll2) != this->eventCollisions_S.end())
            {
                // insert contact model name into the set
                event_coll_to_set_of_model_names_M[coll2].insert(coll1->GetParentModel()->GetName());
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
                    if (coll1->GetModel()->GetName() == "liquid_spheres")
                    {
                        // add to poured set, which also checks for duplicates
                        this->pouredLiquidCollisions_S.insert(coll1);
                    }
                    else if (coll2->GetModel()->GetName() == "liquid_spheres")
                    {
                        // add to poured set, which also checks for duplicates
                        this->pouredLiquidCollisions_S.insert(coll2);
                    }
                }

                ////////////// Poured Particles Collisions
                // check if one collision is a poured particle and the other belongs to the eventCollisions
                if(this->pouredLiquidCollisions_S.find(coll1) != this->pouredLiquidCollisions_S.end() &&
                        this->eventCollisions_S.find(coll2) != this->eventCollisions_S.end())
                {
                    // add the model name to the set coll2's set
                    event_coll_to_set_of_model_names_M[coll2].insert(coll1->GetModel()->GetName());

                    // add the particle collision name to the coll2's set
                    event_coll_to_set_of_particle_names_M[coll2].insert(coll1->GetName());
                }
                else if(this->pouredLiquidCollisions_S.find(coll2) != this->pouredLiquidCollisions_S.end() &&
                        this->eventCollisions_S.find(coll1) != this->eventCollisions_S.end())
                {
                    // add the model name to the set coll1's set
                    event_coll_to_set_of_model_names_M[coll1].insert(coll2->GetModel()->GetName());

                    // add the particle collision name to the coll1's set
                    event_coll_to_set_of_particle_names_M[coll1].insert(coll2->GetName());
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
                    event_coll_to_set_of_model_names_M[coll2].insert(coll1->GetModel()->GetName());

                    // add the particle collision name to the coll2's set
                    event_coll_to_set_of_particle_names_M[coll2].insert(coll1->GetName());
                }
                else if(this->pancakeCollision_S.find(coll2) != this->pancakeCollision_S.end() &&
                        this->eventCollisions_S.find(coll1) != this->eventCollisions_S.end())
                {
                    // add the model name to the set coll1's set
                    event_coll_to_set_of_model_names_M[coll1].insert(coll2->GetModel()->GetName());

                    // add the particle collision name to the coll1's set
                    event_coll_to_set_of_particle_names_M[coll1].insert(coll2->GetName());
                }
            }
        }


        ////////////// Compare states
        // check for grasp
        if (fore_finger_contact && thumb_contact)
        {
            // if coll1 belongs to the hand model, then coll2 is the grasped model
            if (grasp_coll1->GetParentModel()->GetName() == "hit_hand")
            {
                grasped_model_name = grasp_coll2->GetParentModel()->GetName();
            }
            else
            {
                grasped_model_name = grasp_coll1->GetParentModel()->GetName();
            }
        }

        // check for difference between current and past grasp
        if (grasped_model_name != this->graspedModelName)
        {
           diff_detected = true;
            this->graspedModelName = grasped_model_name;
        }

        // check current and past collision states difference
        if (event_coll_to_set_of_model_names_M != this->eventCollToSetOfModelNames_M)
        {
            diff_detected = true;;
            this->eventCollToSetOfModelNames_M = event_coll_to_set_of_model_names_M;
        }

        // TODO Pour Pancake events
        // check if new particle has been poured
        if (this->pouredLiquidCollisions_S.size() > prev_poured_particles_nr )
        {
            diff_detected = true;
        }

        // check if poured particle collision appeared/changed
        if (event_coll_to_set_of_particle_names_M != this->eventCollToSetOfParticleNames_M)
        {
            diff_detected = true;;
            this->eventCollToSetOfParticleNames_M = event_coll_to_set_of_particle_names_M;
        }

        // save the particles belonging to the pancake
        if (!this->pancakeCreated && grasped_model_name == "spatula")
        {
            ////////////// Loop through all the contacts
            for (unsigned int i = 0; i < _contacts.size(); i++)
            {
                // collision 1 and 2 of the contact
                physics::Collision* coll1 = _contacts.at(i)->collision1;
                physics::Collision* coll2 = _contacts.at(i)->collision2;

                // save the particles belonging to the pancake
                if ((coll1->GetName() == "pancake_maker_event_collision")
                        && (coll2->GetModel()->GetName() == "liquid_spheres"))
                {
                    this->pancakeCollision_S.insert(coll2);
                }
                else if((coll2->GetName() == "pancake_maker_event_collision")
                        && (coll1->GetModel()->GetName() == "liquid_spheres"))
                {
                    this->pancakeCollision_S.insert(coll1);
                }
            }

            diff_detected = true;
            this->pancakeCreated = true;
        }


        ////////////////////////////
        // Output at detected difference
        if (diff_detected)
        {
            diff_detected = false;
    //        this->world->SetPaused(true);

            // document bson object builder
            BSONObjBuilder doc_bo_builder;

            ////////////////////////////
            // Event info
            for(std::map<physics::Collision*, std::set<std::string> >::const_iterator m_iter = event_coll_to_set_of_model_names_M.begin();
                m_iter != event_coll_to_set_of_model_names_M.end(); m_iter++)
            {
                BSONArrayBuilder support_arr_builder;

                std::cout << m_iter->first->GetParentModel()->GetName() << " --> ";
                for(std::set<std::string>::const_iterator s_iter = m_iter->second.begin();
                    s_iter != m_iter->second.end(); s_iter++)
                {
                    std::cout << *s_iter << "; ";

                    support_arr_builder.append(*s_iter);
                }
                std::cout << std::endl;

                doc_bo_builder.append(m_iter->first->GetParentModel()->GetName(), support_arr_builder.arr());
            }

            ////////////////////////////
            // Grasped model
            std::cout << "grasp --> " << grasped_model_name<< ";" << std::endl;

            doc_bo_builder.append("grasp", grasped_model_name);

            ////////////////////////////
            // Poured particles
            std::cout << "poured --> " << this->pouredLiquidCollisions_S.size() <<
                         "/" << this->allLiquidCollisions_S.size() << ";" << std::endl;

            BSONObjBuilder pour_builder;

            BSONObjBuilder pancake_builder;

            pour_builder.append("total particles", (int) this->allLiquidCollisions_S.size());

            pour_builder.append("poured particles", (int) this->pouredLiquidCollisions_S.size());

            ////////////////////////////
            // Pancake size
            std::cout << "pancake size --> " << this->pancakeCollision_S.size() << " particles" << std::endl;

            pancake_builder.append("nr pancake particles", (int) this->pancakeCollision_S.size());

            BSONArrayBuilder pancake_arr_builder;

            // TODO change all interators to const iterator?
            for (std::set<physics::Collision*>::const_iterator c_iter = this->pancakeCollision_S.begin();
                 c_iter != this->pancakeCollision_S.end(); c_iter++)
            {
                //TODO why is the copy required?
                physics::Collision* c = *c_iter;

                pancake_arr_builder.append(c->GetName());
            }

            pancake_builder.append("particle names",pancake_arr_builder.arr());

            ////////////////////////////
            // Pouring Info before pancake created
            if(!this->pancakeCreated)
            {
                BSONObjBuilder pour_support_builder;

                // Poured particles supported by
                for(std::map<physics::Collision*, std::set<std::string> >::const_iterator m_iter = event_coll_to_set_of_particle_names_M.begin();
                    m_iter != event_coll_to_set_of_particle_names_M.end(); m_iter++)
                {
                    BSONArrayBuilder support_arr_builder;

                    std::cout << "\t" << m_iter->first->GetParentModel()->GetName() << " --> ";

                    // Write only nr of particles at the moment
                    std::cout << m_iter->second.size() << " particles;" <<std::endl;
                    for(std::set<std::string>::const_iterator s_iter = m_iter->second.begin();
                        s_iter != m_iter->second.end(); s_iter++)
                    {
                        //std::cout << *s_iter << "; ";
                        support_arr_builder.append(*s_iter);
                    }
                    //std::cout << std::endl;

                    pour_support_builder.append(m_iter->first->GetParentModel()->GetName(), support_arr_builder.arr());
                }

                pour_builder.append("pour supports", pour_support_builder.obj());
            }

            ////////////////////////////
            // Pancake Info
            else if(this->pancakeCreated)
            {
                BSONObjBuilder pancake_support_builder;

                // Pancake particles supported by
                for(std::map<physics::Collision*, std::set<std::string> >::const_iterator m_iter = event_coll_to_set_of_particle_names_M.begin();
                    m_iter != event_coll_to_set_of_particle_names_M.end(); m_iter++)
                {
                    BSONArrayBuilder pancake_arr_builder;

                    std::cout << "\t" << m_iter->first->GetParentModel()->GetName() << " --> ";

                    // Write only nr of particles at the moment
                    std::cout << m_iter->second.size() << " pancake particles;" <<std::endl;
                    for(std::set<std::string>::const_iterator s_iter = m_iter->second.begin();
                        s_iter != m_iter->second.end(); s_iter++)
                    {
                        pancake_arr_builder.append(*s_iter);
                        //std::cout << *s_iter << "; ";
                    }
                    //std::cout << std::endl;

                    pancake_support_builder.append(m_iter->first->GetParentModel()->GetName(), pancake_arr_builder.arr());
                }

                pancake_builder.append("pancake supports", pancake_support_builder.obj());
            }
            std::cout <<"-------------------------------------------------------------------ts: "<< _timestamp << std::endl;

        // TODO Pour Pancake events
            doc_bo_builder.append("pour", pour_builder.obj());
            doc_bo_builder.append("pancake", pancake_builder.obj());

            // create the document object
            doc_bo_builder.append("timestamp", _timestamp);

            // insert document object into the database
//            this->mongoDBClientConnection.insert(this->dbCollName + ".particles", doc_bo_builder.obj());

        	// use scoped connection
        	ScopedDbConnection scoped_connection("localhost");

        	// insert document object into the database
        	scoped_connection->insert(this->dbCollName + ".particles", doc_bo_builder.obj());

        	// let the pool know the connection is done
        	scoped_connection.done();
        }

    //    boost::chrono::duration<double> ev_sec = boost::chrono::system_clock::now() - ev_start;
    //    std::cout << "EVENT: " << ev_sec.count() << " seconds\n";
}

//////////////////////////////////////////////////
void PostProcess::WriteRawData()
{
//    boost::chrono::system_clock::time_point raw_start = boost::chrono::system_clock::now();

    // compute simulation time in nanoseconds
    const long long int _timestamp = this->world->GetSimTime().nsec + this->world->GetSimTime().sec * 1e9;

    // get all the contacts from the physics engine
    const std::vector<physics::Contact*> _contacts = this->contactManagerPtr->GetContacts();

    // document bson object
    BSONObj _doc_bo;

    // bson array model builder
    BSONArrayBuilder _bson_model_arr_builder;

    //////////////////////////////////////////////////
    // loop trough all the models
    for (unsigned int i = 0; i < this->models.size(); i++ )
    {
        // get the links vector from the current model
        const physics::Link_V _links = this->models.at(i)->GetLinks();

        // bson array builder
        BSONArrayBuilder _link_arr_builder;

        //////////////////////////////////////////////////
        // loop through the links
        for (unsigned int j = 0; j < _links.size(); j++)
        {
            // get the collisions of the current link
            const physics::Collision_V _collisions = _links.at(j)->GetCollisions();

            // bson array builder
            BSONArrayBuilder _collision_arr_builder;


            //////////////////////////////////////////////////
            // loop through the collisions
            for (unsigned int k = 0; k < _collisions.size(); k++)
            {
                // bson array builder
                BSONArrayBuilder _contacts_arr_builder;

                //////////////////////////////////////////////////
                // loop through all the global contacts to check if they match the collision
                // TODO not the most effective way
                for (unsigned int l = 0; l < _contacts.size(); l++)
                {
                    //std::cout << "\t" << _contacts.at(l)->collision1->GetName() << " --> "
                    //		<< _contacts.at(l)->collision2->GetName() << std::endl;

                    // check if the current collision equals the contact collision1
                    if (_collisions.at(k)->GetName() ==
                            _contacts.at(l)->collision1->GetName())
                    {
                        // create BSON contact object with opposite coll: collision2
                        BSONObj _contact_bo = PostProcess::CreateBSONContactObject(
                                _contacts.at(l), _contacts.at(l)->collision2);

                        // append collision obj to array
                        _contacts_arr_builder.append(_contact_bo);

                    }
                    // if the current collision equals the contact collision2
                    else if(_collisions.at(k)->GetName() ==
                            _contacts.at(l)->collision2->GetName())
                    {
                        // create BSON contact object with the opposite coll: collision1
                        BSONObj _contact_bo = PostProcess::CreateBSONContactObject(
                                _contacts.at(l), _contacts.at(l)->collision1);

                        // append collision obj to array
                        _contacts_arr_builder.append(_contact_bo);
                    }

                }

                // create the bson contacts array
                BSONArray _contact_arr = _contacts_arr_builder.arr();

                // collision bson obj
                BSONObj _collision_bo = PostProcess::CreateBSONCollisionObject(_collisions.at(k), _contact_arr);

                // append collision obj to array
                _collision_arr_builder.append(_collision_bo);
            }

            // bson array
            BSONArray _collision_arr = _collision_arr_builder.arr();

            // link bson object
            BSONObj _link_bo = PostProcess::CreateBSONLinkObject(_links.at(j), _collision_arr);

            // append link object to array
            _link_arr_builder.append(_link_bo);
        }

        // create the bson link array
        BSONArray _link_arr = _link_arr_builder.arr();

        // model bson object
        BSONObj _model_bo = PostProcess::CreateBSONModelObject(this->models.at(i), _link_arr);

        // append model object to array
        _bson_model_arr_builder.append(_model_bo);
    }


    // create the bson model array
    BSONArray _bson_model_arr = _bson_model_arr_builder.arr();

    // create the document object
    _doc_bo = BSON("models" << _bson_model_arr << "timestamp" << _timestamp);

    // insert document object into the database
//    this->mongoDBClientConnection.insert(this->dbCollName + ".raw", _doc_bo);

	// use scoped connection
	ScopedDbConnection scoped_connection("localhost");

	// insert document object into the database
	scoped_connection->insert(this->dbCollName + ".raw", _doc_bo);

	// let the pool know the connection is done
	scoped_connection.done();

//    boost::chrono::duration<double> raw_sec = boost::chrono::system_clock::now() - raw_start;
//    std::cout << "RAW: " << raw_sec.count() << " seconds\n";

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
				tf::StampedTransform(transform, tf_time, this->world->GetName(), m_iter->get()->GetName()));

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

			// get the timestamp im ms and date format
			Date_t stamp_ms = this->world->GetSimTime().nsec / 1000000.0 + this->world->GetSimTime().sec * 1000.0;

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

    // insert document object into the database
//    this->mongoDBClientConnection.insert(this->dbCollName + ".tf", BSON("transforms" << transforms_bo));

	// use scoped connection
	ScopedDbConnection scoped_connection("localhost");

	// insert document object into the database
	scoped_connection->insert(this->dbCollName + ".tf", BSON("transforms" << transforms_bo
														<< "__recorded" << Date_t(time(NULL) * 1000)
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
				double duration2 = fabs(
						(_curr_st_iter->stamp_.sec * 1e9 + _curr_st_iter->stamp_.nsec) -
								(memory_st_iter->stamp_.sec * 1e9 + memory_st_iter->stamp_.nsec));


				// check if the thresholds are crossed
				if(vect_dist > this->tfVectDistThresh ||
						angular_dist > this->tfAngularDistThresh ||
								duration > this->tfDurationThresh)
				{
					std::cout << memory_st_iter->frame_id_<< "->" <<memory_st_iter->child_frame_id_
							<< " dist: " << vect_dist
							<< " rot: " << angular_dist
							<< " duration: " << duration
							<< " duration2: " << duration2 << std::endl;

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
    // compute simulation time in miliseconds
    const int timestamp_ms = this->world->GetSimTime().nsec / 1000000.0 + this->world->GetSimTime().sec * 1000.0;


//	this->contextIDs.push_back(this ->beliefStateClient->startContext("exp1", "&sim", "Contact", timestamp_ms));

//	this->beliefStateClient->endContext(this->contextIDs.back(), true, timestamp_ms);
//
//	this->contextIDs.pop_back();

//	this->eventCollisionThumb->co

	// set diff detected flag to false
	bool diff_detected = false;

    // compute simulation time in nanoseconds
    const long long int _timestamp = this->world->GetSimTime().nsec + this->world->GetSimTime().sec * 1e9;

    // get all the contacts from the physics engine
    const std::vector<physics::Contact*> _contacts = this->contactManagerPtr->GetContacts();

    // current map of event collisions to set of models names
    std::map< physics::Collision*, std::set<std::string> > event_coll_to_set_of_model_names_M;

    // init current map with the supporting event collisions, and an empty set
    for(std::set<physics::Collision*>::const_iterator s_iter = this->eventCollisions_S.begin();
            s_iter != this->eventCollisions_S.end(); s_iter++)
    {
        event_coll_to_set_of_model_names_M[*s_iter] = std::set<std::string>();
    }

    // current map of event collisions to set of models names
    std::map< physics::Collision*, std::set<std::string> > event_coll_to_set_of_particle_names_M;

    // init current map with the supporting event collisions, and an empty set
    for(std::set<physics::Collision*>::const_iterator s_iter = this->eventCollisions_S.begin();
            s_iter != this->eventCollisions_S.end(); s_iter++)
    {
        event_coll_to_set_of_particle_names_M[*s_iter] = std::set<std::string>();
    }

    // TODO check until the pouring is finished
    // curr poured particles
    int prev_poured_particles_nr = this->pouredLiquidCollisions_S.size();

    // current grasped model name
    std::string grasped_model_name;

    // set finger contacts flags to false
    bool fore_finger_contact = false;
    bool thumb_contact = false;

    // grasp collisions
    physics::Collision *grasp_coll1, *grasp_coll2;

    ////////////// Loop through all the contacts
    // set current states
    for (unsigned int i = 0; i < _contacts.size(); i++)
    {
        // collision 1 and 2 of the contact
        physics::Collision* coll1 = _contacts.at(i)->collision1;
        physics::Collision* coll2 = _contacts.at(i)->collision2;


        ////////////// Supporting Collisions
        // check if collision 1 belongs to the event collision set
        if (this->eventCollisions_S.find(coll1) != this->eventCollisions_S.end())
        {
            // insert contact model name into the set
            event_coll_to_set_of_model_names_M[coll1].insert(coll2->GetParentModel()->GetName());
        }
        // check if collision 2 belongs to the event collision set
        else if(this->eventCollisions_S.find(coll2) != this->eventCollisions_S.end())
        {
            // insert contact model name into the set
            event_coll_to_set_of_model_names_M[coll2].insert(coll1->GetParentModel()->GetName());
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
                if (coll1->GetModel()->GetName() == "liquid_spheres")
                {
                    // add to poured set, which also checks for duplicates
                    this->pouredLiquidCollisions_S.insert(coll1);
                }
                else if (coll2->GetModel()->GetName() == "liquid_spheres")
                {
                    // add to poured set, which also checks for duplicates
                    this->pouredLiquidCollisions_S.insert(coll2);
                }
            }

            ////////////// Poured Particles Collisions
            // check if one collision is a poured particle and the other belongs to the eventCollisions
            if(this->pouredLiquidCollisions_S.find(coll1) != this->pouredLiquidCollisions_S.end() &&
                    this->eventCollisions_S.find(coll2) != this->eventCollisions_S.end())
            {
                // add the model name to the set coll2's set
                event_coll_to_set_of_model_names_M[coll2].insert(coll1->GetModel()->GetName());

                // add the particle collision name to the coll2's set
                event_coll_to_set_of_particle_names_M[coll2].insert(coll1->GetName());
            }
            else if(this->pouredLiquidCollisions_S.find(coll2) != this->pouredLiquidCollisions_S.end() &&
                    this->eventCollisions_S.find(coll1) != this->eventCollisions_S.end())
            {
                // add the model name to the set coll1's set
                event_coll_to_set_of_model_names_M[coll1].insert(coll2->GetModel()->GetName());

                // add the particle collision name to the coll1's set
                event_coll_to_set_of_particle_names_M[coll1].insert(coll2->GetName());
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
                event_coll_to_set_of_model_names_M[coll2].insert(coll1->GetModel()->GetName());

                // add the particle collision name to the coll2's set
                event_coll_to_set_of_particle_names_M[coll2].insert(coll1->GetName());
            }
            else if(this->pancakeCollision_S.find(coll2) != this->pancakeCollision_S.end() &&
                    this->eventCollisions_S.find(coll1) != this->eventCollisions_S.end())
            {
                // add the model name to the set coll1's set
                event_coll_to_set_of_model_names_M[coll1].insert(coll2->GetModel()->GetName());

                // add the particle collision name to the coll1's set
                event_coll_to_set_of_particle_names_M[coll1].insert(coll2->GetName());
            }
        }
    }


    ////////////// Compare states
    // check for grasp
    if (fore_finger_contact && thumb_contact)
    {
        // if coll1 belongs to the hand model, then coll2 is the grasped model
        if (grasp_coll1->GetParentModel()->GetName() == "hit_hand")
        {
            grasped_model_name = grasp_coll2->GetParentModel()->GetName();
        }
        else
        {
            grasped_model_name = grasp_coll1->GetParentModel()->GetName();
        }
    }

    // check for difference between current and past grasp
    if (grasped_model_name != this->graspedModelName)
    {
       diff_detected = true;
        this->graspedModelName = grasped_model_name;
    }

    // check current and past collision states difference
    if (event_coll_to_set_of_model_names_M != this->eventCollToSetOfModelNames_M)
    {
        diff_detected = true;;
        this->eventCollToSetOfModelNames_M = event_coll_to_set_of_model_names_M;
    }

    // TODO Pour Pancake events
    // check if new particle has been poured
    if (this->pouredLiquidCollisions_S.size() > prev_poured_particles_nr )
    {
        diff_detected = true;
    }

    // check if poured particle collision appeared/changed
    if (event_coll_to_set_of_particle_names_M != this->eventCollToSetOfParticleNames_M)
    {
        diff_detected = true;;
        this->eventCollToSetOfParticleNames_M = event_coll_to_set_of_particle_names_M;
    }

    // save the particles belonging to the pancake
    if (!this->pancakeCreated && grasped_model_name == "spatula")
    {
        ////////////// Loop through all the contacts
        for (unsigned int i = 0; i < _contacts.size(); i++)
        {
            // collision 1 and 2 of the contact
            physics::Collision* coll1 = _contacts.at(i)->collision1;
            physics::Collision* coll2 = _contacts.at(i)->collision2;

            // save the particles belonging to the pancake
            if ((coll1->GetName() == "pancake_maker_event_collision")
                    && (coll2->GetModel()->GetName() == "liquid_spheres"))
            {
                this->pancakeCollision_S.insert(coll2);
            }
            else if((coll2->GetName() == "pancake_maker_event_collision")
                    && (coll1->GetModel()->GetName() == "liquid_spheres"))
            {
                this->pancakeCollision_S.insert(coll1);
            }
        }

        diff_detected = true;
        this->pancakeCreated = true;
    }


    ////////////////////////////
    // Output at detected difference
    if (diff_detected)
    {
        diff_detected = false;
//        this->world->SetPaused(true);

        // document bson object builder
        BSONObjBuilder doc_bo_builder;

        ////////////////////////////
        // Event info
        for(std::map<physics::Collision*, std::set<std::string> >::const_iterator m_iter = event_coll_to_set_of_model_names_M.begin();
            m_iter != event_coll_to_set_of_model_names_M.end(); m_iter++)
        {
            BSONArrayBuilder support_arr_builder;

            std::cout << m_iter->first->GetParentModel()->GetName() << " --> ";
            for(std::set<std::string>::const_iterator s_iter = m_iter->second.begin();
                s_iter != m_iter->second.end(); s_iter++)
            {
                std::cout << *s_iter << "; ";

                support_arr_builder.append(*s_iter);
            }
            std::cout << std::endl;

            doc_bo_builder.append(m_iter->first->GetParentModel()->GetName(), support_arr_builder.arr());
        }

        ////////////////////////////
        // Grasped model
        std::cout << "grasp --> " << grasped_model_name<< ";" << std::endl;

        doc_bo_builder.append("grasp", grasped_model_name);

        ////////////////////////////
        // Poured particles
        std::cout << "poured --> " << this->pouredLiquidCollisions_S.size() <<
                     "/" << this->allLiquidCollisions_S.size() << ";" << std::endl;

        BSONObjBuilder pour_builder;

        BSONObjBuilder pancake_builder;

        pour_builder.append("total particles", (int) this->allLiquidCollisions_S.size());

        pour_builder.append("poured particles", (int) this->pouredLiquidCollisions_S.size());

        ////////////////////////////
        // Pancake size
        std::cout << "pancake size --> " << this->pancakeCollision_S.size() << " particles" << std::endl;

        pancake_builder.append("nr pancake particles", (int) this->pancakeCollision_S.size());

        BSONArrayBuilder pancake_arr_builder;

        // TODO change all interators to const iterator?
        for (std::set<physics::Collision*>::const_iterator c_iter = this->pancakeCollision_S.begin();
             c_iter != this->pancakeCollision_S.end(); c_iter++)
        {
            //TODO why is the copy required?
            physics::Collision* c = *c_iter;

            pancake_arr_builder.append(c->GetName());
        }

        pancake_builder.append("particle names",pancake_arr_builder.arr());

        ////////////////////////////
        // Pouring Info before pancake created
        if(!this->pancakeCreated)
        {
            BSONObjBuilder pour_support_builder;

            // Poured particles supported by
            for(std::map<physics::Collision*, std::set<std::string> >::const_iterator m_iter = event_coll_to_set_of_particle_names_M.begin();
                m_iter != event_coll_to_set_of_particle_names_M.end(); m_iter++)
            {
                BSONArrayBuilder support_arr_builder;

                std::cout << "\t" << m_iter->first->GetParentModel()->GetName() << " --> ";

                // Write only nr of particles at the moment
                std::cout << m_iter->second.size() << " particles;" <<std::endl;
                for(std::set<std::string>::const_iterator s_iter = m_iter->second.begin();
                    s_iter != m_iter->second.end(); s_iter++)
                {
                    //std::cout << *s_iter << "; ";
                    support_arr_builder.append(*s_iter);
                }
                //std::cout << std::endl;

                pour_support_builder.append(m_iter->first->GetParentModel()->GetName(), support_arr_builder.arr());
            }

            pour_builder.append("pour supports", pour_support_builder.obj());
        }

        ////////////////////////////
        // Pancake Info
        else if(this->pancakeCreated)
        {
            BSONObjBuilder pancake_support_builder;

            // Pancake particles supported by
            for(std::map<physics::Collision*, std::set<std::string> >::const_iterator m_iter = event_coll_to_set_of_particle_names_M.begin();
                m_iter != event_coll_to_set_of_particle_names_M.end(); m_iter++)
            {
                BSONArrayBuilder pancake_arr_builder;

                std::cout << "\t" << m_iter->first->GetParentModel()->GetName() << " --> ";

                // Write only nr of particles at the moment
                std::cout << m_iter->second.size() << " pancake particles;" <<std::endl;
                for(std::set<std::string>::const_iterator s_iter = m_iter->second.begin();
                    s_iter != m_iter->second.end(); s_iter++)
                {
                    pancake_arr_builder.append(*s_iter);
                    //std::cout << *s_iter << "; ";
                }
                //std::cout << std::endl;

                pancake_support_builder.append(m_iter->first->GetParentModel()->GetName(), pancake_arr_builder.arr());
            }

            pancake_builder.append("pancake supports", pancake_support_builder.obj());
        }
        std::cout <<"-------------------------------------------------------------------ts: "<< _timestamp << std::endl;

    // TODO Pour Pancake events
        doc_bo_builder.append("pour", pour_builder.obj());
        doc_bo_builder.append("pancake", pancake_builder.obj());

        // create the document object
        doc_bo_builder.append("timestamp", _timestamp);

        // insert document object into the database
//            this->mongoDBClientConnection.insert(this->dbCollName + ".particles", doc_bo_builder.obj());

    	// use scoped connection
    	ScopedDbConnection scoped_connection("localhost");

    	// insert document object into the database
    	scoped_connection->insert(this->dbCollName + ".particles", doc_bo_builder.obj());

    	// let the pool know the connection is done
    	scoped_connection.done();
    }

}

//////////////////////////////////////////////////
BSONObj PostProcess::CreateBSONContactObject(const physics::Contact* _contact, const physics::Collision* _collision)
{
	return BSON ("name" << _collision->GetName()
			<< "coll_model_name" << _collision->GetParent()->GetParent()->GetName()
			<< "coll_link_name" << _collision->GetParent()->GetName()
			// TODO if all contact points are needed loop through all the values
			// only first contact point is used
			<< "pos" << BSON ("x" << _contact->positions[0].x
							<< "y" << _contact->positions[0].y
							<< "z" << _contact->positions[0].z)
			<< "normal" << BSON ("x" << _contact->normals[0].x
							<< "y" << _contact->normals[0].y
							<< "z" << _contact->normals[0].z));
}

//////////////////////////////////////////////////
BSONObj PostProcess::CreateBSONCollisionObject(const physics::CollisionPtr _collision, const BSONArray _contact_arr)
{
	return 	BSON ("name" << _collision->GetName()
			<< "pos" << BSON(  "x" << _collision->GetWorldPose().pos.x
							<< "y" << _collision->GetWorldPose().pos.y
							<< "z" << _collision->GetWorldPose().pos.z)
			<< "rot" << BSON(  "x" << _collision->GetWorldPose().rot.GetAsEuler().x
							<< "y" << _collision->GetWorldPose().rot.GetAsEuler().y
							<< "z" << _collision->GetWorldPose().rot.GetAsEuler().z)
			<< "bbox" << BSON( "min" << BSON(  "x" << _collision->GetBoundingBox().min.x
											<< "y" << _collision->GetBoundingBox().min.y
											<< "z" << _collision->GetBoundingBox().min.z)
							<< "max" << BSON(  "x" << _collision->GetBoundingBox().max.x
											<< "y" << _collision->GetBoundingBox().max.y
											<< "z" << _collision->GetBoundingBox().max.z))
			<< "contacts" << _contact_arr);
}

//////////////////////////////////////////////////
BSONObj PostProcess::CreateBSONLinkObject(const physics::LinkPtr _link, const BSONArray _collision_arr)
{
	return BSON("name" << _link->GetName()
			<< "pos" << BSON(  "x" << _link->GetWorldPose().pos.x
							<< "y" << _link->GetWorldPose().pos.y
							<< "z" << _link->GetWorldPose().pos.z)
			<< "rot" << BSON(  "x" << _link->GetWorldPose().rot.GetAsEuler().x
							<< "y" << _link->GetWorldPose().rot.GetAsEuler().y
							<< "z" << _link->GetWorldPose().rot.GetAsEuler().z)
			<< "bbox" << BSON( "min" << BSON(  "x" << _link->GetBoundingBox().min.x
											<< "y" << _link->GetBoundingBox().min.y
											<< "z" << _link->GetBoundingBox().min.z)
							<< "max" << BSON(  "x" << _link->GetBoundingBox().max.x
											<< "y" << _link->GetBoundingBox().max.y
											<< "z" << _link->GetBoundingBox().max.z))
			<< "collisions" << _collision_arr);
}

//////////////////////////////////////////////////
BSONObj PostProcess::CreateBSONModelObject(const physics::ModelPtr _model, const BSONArray _link_arr)
{
	return BSON("name" << _model->GetName()
		<< "pos" << BSON(  "x"  << _model->GetWorldPose().pos.x
						<< "y"  << _model->GetWorldPose().pos.y
						<< "z"  << _model->GetWorldPose().pos.z)
		<< "rot" << BSON(  "x"  << _model->GetWorldPose().rot.GetAsEuler().x
						<< "y"  << _model->GetWorldPose().rot.GetAsEuler().y
						<< "z"  << _model->GetWorldPose().rot.GetAsEuler().z)
		<< "bbox" << BSON( "min" << BSON(  "x" << _model->GetBoundingBox().min.x
										<< "y" << _model->GetBoundingBox().min.y
										<< "z" << _model->GetBoundingBox().min.z)
						<< "max" << BSON(  "x" << _model->GetBoundingBox().max.x
										<< "y" << _model->GetBoundingBox().max.y
										<< "z" << _model->GetBoundingBox().max.z))
		<< "links" << _link_arr);
}

// needed to start the contact publishing in the physics engine
//////////////////////////////////////////////////
void PostProcess::DummyContactsCallback(ConstContactsPtr& _msg)
{
}

//////////////////////////////////////////////////
void PostProcess::DebugOutput(std::string _msg)
{

	std::cout << _msg << std::endl;

	std::vector<physics::Contact*> contacts = this->contactManagerPtr->GetContacts();

	for (unsigned int i = 0; i < contacts.size(); i++)
	{
		physics::Collision* coll1 = contacts.at(i)->collision1;
		physics::Collision* coll2 = contacts.at(i)->collision2;

		std::cout << coll1->GetName() << " --> "<< coll2->GetName() << std::endl;

	}

}

