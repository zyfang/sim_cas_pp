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

#include "LogEvents.hh"

using namespace sg_pp;
using namespace gazebo;
using namespace mongo;

#define TIME_OFFSET 100

//////////////////////////////////////////////////
LogEvents::LogEvents(const gazebo::physics::WorldPtr _world,
		const std::string _db_name,
		const std::string _coll_name,
		int _suffix)
	: world(_world)
	, dbName(_db_name)
	, collName(_coll_name)
{
	// get the world models
	this->models = this->world->GetModels();

    // get the liquid model
    this->liquidSpheres = this->world->GetModel("LiquidTangibleThing");

	// get the world models
	this->contactManagerPtr = this->world->GetPhysicsEngine()->GetContactManager();

	// get values from the config file
	LogEvents::ReadConfigFile();

	// set the grasp init flag to false
	this->graspInit = false;

    // set the pouring started flag to false
    this->pancakeCreated = false;

	// TODO for adding time offset to the simulation times
	this->suffixTime = _suffix;
}

//////////////////////////////////////////////////
LogEvents::~LogEvents()
{

}

//////////////////////////////////////////////////
void LogEvents::ReadConfigFile()
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
	this->logLocation = cfg.lookup("events.log_location").c_str();
	std::cout << "*LogEvents* - log_location: " << this->logLocation << std::endl;

	this->eventDiscTresh = cfg.lookup("events.ev_disc_thresh");
	std::cout << "*LogEvents* - ev_disc_thresh: " << this->eventDiscTresh << std::endl;

}

//////////////////////////////////////////////////
void LogEvents::InitEvents()
{
	std::cout << "*LogEvents* - Sim start: " << this->world->GetSimTime().Double() << std::endl;

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
				new beliefstate_client::Object("&knowrob;", m_iter->get()->GetName());


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

                        // init the event coll to particle names map with empty sets of strings
                        this->eventCollToSetOfParticleNames_M[c_iter->get()] = std::set<std::string>();
					}
				}
			}
		}
	}
}

//////////////////////////////////////////////////
void LogEvents::CheckEvents()
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

    // grasped model
    physics::ModelPtr ff_grasped_model;
    physics::ModelPtr thumb_grasped_model;

    // current grasped model
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
        physics::Collision* curr_coll1 = all_contacts.at(i)->collision1;
        physics::Collision* curr_coll2 = all_contacts.at(i)->collision2;


        ////////////// Supporting Collisions
        // check if collision 1 belongs to the event collision set
        if (this->eventCollisions_S.find(curr_coll1) != this->eventCollisions_S.end())
        {
            // insert contact model name into the set
            curr_ev_coll_to_model_names_S_M[curr_coll1].insert(curr_coll2->GetParentModel()->GetName());

            curr_ev_contact_model_pair_S.insert(std::pair<std::string, std::string>(
            		curr_coll1->GetParentModel()->GetName(), curr_coll2->GetParentModel()->GetName()));
        }
        // check if collision 2 belongs to the event collision set
        else if(this->eventCollisions_S.find(curr_coll2) != this->eventCollisions_S.end())
        {
            // insert contact model name into the set
            curr_ev_coll_to_model_names_S_M[curr_coll2].insert(curr_coll1->GetParentModel()->GetName());

            curr_ev_contact_model_pair_S.insert(std::pair<std::string, std::string>(
            		curr_coll2->GetParentModel()->GetName(), curr_coll1->GetParentModel()->GetName()));
        }


        /////////////// Grasping
        // if one of the collisions is the finger sensors, check for the grasped model
        if (curr_coll1 == this->eventCollisionForeFinger || curr_coll2 == this->eventCollisionForeFinger)
        {
            // if one of the collisions is the fore finger, set contact flag to true, and save both collisions
            fore_finger_contact = true;

            // check which is the actual grasped model from the collisions
            // (these vary randomly, coll1 might be self or the one in contact with)
            if (curr_coll1->GetParentModel() == this->eventCollisionForeFinger->GetModel())
            {
            	// coll1 is self, meaning the grasped model is coll2
            	ff_grasped_model = curr_coll2->GetParentModel();
            }
            else
            {
            	// coll1 is not self, implies that it's the grasped model
            	ff_grasped_model = curr_coll1->GetParentModel();
            }

        }
        else if (curr_coll1 == this->eventCollisionThumb || curr_coll2 == this->eventCollisionThumb)
        {
            // if one of the collisions is the thumb, set contact flag to true, and save both collisions
            thumb_contact = true;

            // check which is the actual grasped model from the collisions
            // (these vary randomly, coll1 might be self or the one in contact with)
            if (curr_coll1->GetParentModel() == this->eventCollisionForeFinger->GetModel())
            {
            	// coll1 is self, meaning the grasped model is coll2
            	thumb_grasped_model = curr_coll2->GetParentModel();
            }
            else
            {
            	// coll1 is not self, implies that it's the grasped model
            	thumb_grasped_model = curr_coll1->GetParentModel();
            }
        }

        // TODO Pour Pancake events
        ////////////// Pouring Action
        // look into pouring until the pancake is created
        if(!this->pancakeCreated)
        {
            // check for the currently poured particles
            if (curr_coll1 == this->eventCollisionMug || curr_coll2 == this->eventCollisionMug)
            {
                // check if coll1 or 2 belongs to the liquid
                if (curr_coll1->GetModel() == this->liquidSpheres)
                {
                    // add to poured set, which also checks for duplicates
                    this->totalPouredParticles_S.insert(curr_coll1);
                }
                else if (curr_coll2->GetModel() == this->liquidSpheres)
                {
                    // add to poured set, which also checks for duplicates
                    this->totalPouredParticles_S.insert(curr_coll2);
                }
            }

            ////////////// Poured Particles Collisions
            // check if one collision is a poured particle and the other belongs to the eventCollisions
            if(this->totalPouredParticles_S.find(curr_coll1) != this->totalPouredParticles_S.end() &&
                    this->eventCollisions_S.find(curr_coll2) != this->eventCollisions_S.end())
            {
                // add the model name to the set coll2's set
                curr_ev_coll_to_model_names_S_M[curr_coll2].insert(curr_coll1->GetModel()->GetName());

                // add the particle collision name to the coll2's set
                curr_ev_coll_to_particle_names_S_M[curr_coll2].insert(curr_coll1->GetName());
            }
            else if(this->totalPouredParticles_S.find(curr_coll2) != this->totalPouredParticles_S.end() &&
                    this->eventCollisions_S.find(curr_coll1) != this->eventCollisions_S.end())
            {
                // add the model name to the set coll1's set
                curr_ev_coll_to_model_names_S_M[curr_coll1].insert(curr_coll2->GetModel()->GetName());

                // add the particle collision name to the coll1's set
                curr_ev_coll_to_particle_names_S_M[curr_coll1].insert(curr_coll2->GetName());
            }
        }

        ////////////// Flipping Action
        // create pancake when the spatula is grasped, save all particles belonging to the pancake
        else if(this->pancakeCreated)
        {
            ////////////// Pancake Particles Collisions
            // check if one collision is a poured particle and the other belongs to the eventCollisions
            if(this->pancakeCollision_S.find(curr_coll1) != this->pancakeCollision_S.end() &&
                    this->eventCollisions_S.find(curr_coll2) != this->eventCollisions_S.end())
            {
                // add the model name to the set coll2's set
                curr_ev_coll_to_model_names_S_M[curr_coll2].insert(curr_coll1->GetModel()->GetName());

                // add the particle collision name to the coll2's set
                curr_ev_coll_to_particle_names_S_M[curr_coll2].insert(curr_coll1->GetName());
            }
            else if(this->pancakeCollision_S.find(curr_coll2) != this->pancakeCollision_S.end() &&
                    this->eventCollisions_S.find(curr_coll1) != this->eventCollisions_S.end())
            {
                // add the model name to the set coll1's set
                curr_ev_coll_to_model_names_S_M[curr_coll1].insert(curr_coll2->GetModel()->GetName());

                // add the particle collision name to the coll1's set
                curr_ev_coll_to_particle_names_S_M[curr_coll1].insert(curr_coll2->GetName());
            }
        }
    }

    ////////////////////////////////////////////////////////////////
    ////////////// Compare current state with previous one

    // Check if the current grasp has changed
    diff_detected = LogEvents::CheckCurrentGrasp(
    		timestamp_ms, ff_grasped_model, thumb_grasped_model);

    // Check if difference between the event collisions have appeared
    diff_detected = LogEvents::CheckCurrentEventCollisions(
    		timestamp_ms, curr_ev_contact_model_pair_S);

	// Check if diff between particles leaving the mug appeared
	diff_detected = LogEvents::CheckFluidFlowTransEvent(
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
void LogEvents::FiniEvents()
{
	// close main GzEvent
	this->nameToEvents_M["Main"].back()->End(this->world->GetSimTime().Double());

	// close all open events
	LogEvents::EndActiveEvents();

	// Concatenate timelines with short disconnections
	LogEvents::MergeEventDisconnections();

	// write events as belief state contexts
	LogEvents::WriteContexts();

	// write events to timeline file
	LogEvents::WriteTimelines();
}

//////////////////////////////////////////////////
bool LogEvents::CheckCurrentGrasp(
		const double _timestamp_ms,
		gazebo::physics::ModelPtr _ff_grasped_model,
		gazebo::physics::ModelPtr _thumb_grasped_model)
{
	// marks if there is a difference between the previous and current step
	bool diff_detected = false;

    // current grasped model
	physics::ModelPtr curr_grasped_model;

    // check for grasp / both fingers are in contact with the same model
    if(_ff_grasped_model == _thumb_grasped_model && _ff_grasped_model != NULL)
    {
    	curr_grasped_model = _ff_grasped_model;
    }

    // check for difference between current and prev grasp
    if (curr_grasped_model != this->prevGraspedModel)
    {
    	// set diff flat to true
    	diff_detected = true;

    	if (curr_grasped_model != NULL)
    	{
    		this->prevGraspedModel = curr_grasped_model;

    		// the name of the grasping event
			std::string grasp_ev_name = "Grasp" + curr_grasped_model->GetName();

    		// check if the grasp event has been initialized
    		if(this->graspGzEvent != NULL)
    		{
    			// create the contact GzEvent
    			this->graspGzEvent = new GzEvent(
    					grasp_ev_name, "&knowrob;", "GraspingSomething",
    					"knowrob:", "objectActedOn", _timestamp_ms);

    			// add grasped object
    			this->graspGzEvent->AddObject(this->nameToEventObj_M[curr_grasped_model->GetName()]);

    			std::cout << "*LogEvents* - Start - \t" << grasp_ev_name << "\t\t at " << _timestamp_ms  << std::endl;
    		}
    		// init first grasp
    		else
    		{
        		// create first grasp GzEvent
        		this->graspGzEvent = new GzEvent(
        				grasp_ev_name, "&knowrob;", "GraspingSomething",
        				"knowrob:", "objectActedOn", _timestamp_ms);

        		// add grasped object
        		this->graspGzEvent->AddObject(this->nameToEventObj_M[curr_grasped_model->GetName()]);

        		std::cout << "*LogEvents* - Init - \t" << grasp_ev_name << "\t\t at " << _timestamp_ms  << std::endl;
    		}
    	}
    	else
    	{
    		this->prevGraspedModel.reset();

    		// end grasping event
			this->graspGzEvent->End(_timestamp_ms);

			std::cout << "*LogEvents* - End - \t" << this->graspGzEvent->GetName() << "\t\t at " << _timestamp_ms  << std::endl;

			// add grasp event to the map of all events
			this->nameToEvents_M[this->graspGzEvent->GetName()].push_back(this->graspGzEvent);
    	}

    }

    return diff_detected;
}

//////////////////////////////////////////////////
bool LogEvents::CheckCurrentEventCollisions(
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
				sg_pp::GzEvent* contact_event = new sg_pp::GzEvent(
						contact_ev_name, "&knowrob_sim;", "TouchingSituation",
						"knowrob_sim:", "inContact", _timestamp_ms);

				// add the two objects in contact
				contact_event->AddObject(this->nameToEventObj_M[m_iter->first]);

				contact_event->AddObject(this->nameToEventObj_M[m_iter->second]);

				// add local event to the map
				this->nameToEvents_M[contact_ev_name].push_back(contact_event);

			    std::cout << "*LogEvents* - Start - \t" << contact_ev_name << "\t\t at " << _timestamp_ms << std::endl;
			}
			else
			{
				// if the contact exists and it is open, end it
				if(this->nameToEvents_M[contact_ev_name].back()->IsOpen())
				{
					// end contact event
					this->nameToEvents_M[contact_ev_name].back()->End(_timestamp_ms);

				    std::cout << "*LogEvents* - End - \t" << contact_ev_name << "\t\t at " << _timestamp_ms << std::endl;
				}
				else
				{
					// create local contact GzEvent
					sg_pp::GzEvent* contact_event = new sg_pp::GzEvent(
							contact_ev_name, "&knowrob_sim;", "TouchingSituation",
							"knowrob_sim:", "inContact", _timestamp_ms);

					// add the two objects in contact
					contact_event->AddObject(this->nameToEventObj_M[m_iter->first]);

					contact_event->AddObject(this->nameToEventObj_M[m_iter->second]);

					// add local event to the map
					this->nameToEvents_M[contact_ev_name].push_back(contact_event);				}

			    std::cout << "*LogEvents* - Start - \t" << contact_ev_name << "\t\t at " << _timestamp_ms << std::endl;
			}
		}
	}

	return diff_detected;
}

//////////////////////////////////////////////////
bool LogEvents::CheckFluidFlowTransEvent(
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
		if(!this->nameToEvents_M.count("FluidFlow-Translation"))
		{
		    std::cout << "*LogEvents* - Start - \t FluidFlow-Translation \t\t at " << _timestamp_ms << std::endl;

			// add local event to the map
			this->nameToEvents_M["FluidFlow-Translation"].push_back(new sg_pp::GzEvent(
					"FluidFlow-Translation", "&knowrob;", "FluidFlow-Translation", _timestamp_ms));
		}
		else // check if last particle left
		{
			if(this->totalPouredParticles_S.size() == this->allLiquidParticles_S.size())
			{
			    std::cout << "*LogEvents* - End - \t FluidFlow-Translation \t\t at " << _timestamp_ms << std::endl;

				// end liquid transfer event
				this->nameToEvents_M["FluidFlow-Translation"].back()->End(_timestamp_ms);
			}
		}
    }

	return diff_detected;
}

//////////////////////////////////////////////////
void LogEvents::EndActiveEvents()
{
	// iterate through the map
	for(std::map<std::string, std::list<sg_pp::GzEvent*> >::const_iterator m_it = this->nameToEvents_M.begin();
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
void LogEvents::MergeEventDisconnections()
{
    std::cout << "*LogEvents* - Merging event disconnections:" << std::endl;

	// iterate through the map
	for(std::map<std::string, std::list<sg_pp::GzEvent*> >::iterator m_it = this->nameToEvents_M.begin();
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
				if((*ev_it)->GetStartTime() - (*curr_ev)->GetEndTime() < this->eventDiscTresh)
				{
					std::cout << "\t merging " <<  (*ev_it)->GetName()
							<< "\t" << (*curr_ev)->GetEndTime() << " --><-- " << (*ev_it)->GetStartTime() << std::endl;

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
void LogEvents::WriteContexts()
{
	// Write to owl file
	if(this->logLocation == "owl" || this->logLocation == "all")
	{
		// initialize the beliefstate
		this->beliefStateClient = new beliefstate_client::BeliefstateClient("bs_client");

		// starting new experiment
		this->beliefStateClient->startNewExperiment();

		// adding experiment name to the meta data
		this->beliefStateClient->setMetaDataField("experiment", this->collName);

		// register the OWL namespace
		this->beliefStateClient->registerOWLNamespace("knowrob_sim", "http://knowrob.org/kb/knowrob_sim.owl#");

		// iterate through the map
		for(std::map<std::string, std::list<sg_pp::GzEvent*> >::const_iterator m_it = this->nameToEvents_M.begin();
				m_it != this->nameToEvents_M.end(); m_it++)
		{
			// iterate through the events with the same name
			for(std::list<sg_pp::GzEvent*>::const_iterator ev_it = m_it->second.begin();
					ev_it != m_it->second.end(); ev_it++)
			{
				// create local belief state context
				beliefstate_client::Context* curr_ctx;

				// open belief state context
				curr_ctx = new beliefstate_client::Context(this->beliefStateClient,
						(*ev_it)->GetName(), (*ev_it)->GetClassNamespace(), (*ev_it)->GetClass(), (*ev_it)->GetStartTime() + (TIME_OFFSET * this->suffixTime));

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
				curr_ctx->end(true, (*ev_it)->GetEndTime() + (TIME_OFFSET * this->suffixTime));
			}
		}

		// export belief state client
		this->beliefStateClient->exportFiles(this->collName);
	}

	// Write to mongodb
	if(this->logLocation == "mongo" || this->logLocation == "all")
	{
		// all events
		std::vector<BSONObj> events_objs;

	    // insert document object into the database, use scoped connection
		ScopedDbConnection scoped_connection("localhost");

		// iterate through the map
		for(std::map<std::string, std::list<sg_pp::GzEvent*> >::const_iterator m_it = this->nameToEvents_M.begin();
				m_it != this->nameToEvents_M.end(); m_it++)
		{
			// TODO fix the grasp issue, (grasp event map includes the actual events with the right names)
			// iterate through the events with the same name
			for(std::list<GzEvent*>::const_iterator ev_it = m_it->second.begin();
					ev_it != m_it->second.end(); ev_it++)
			{
				// add to the time array
				events_objs.push_back(BSON("name" << (*ev_it)->GetName()
						<<  "start" << (*ev_it)->GetStartTime()
						<< "end" << (*ev_it)->GetEndTime()));
			}
		}

		// insert document object into the database
		scoped_connection->insert(this->dbName + "." + this->collName + "_ev",
				BSON("events" << events_objs));

		// let the pool know the connection is done
		scoped_connection.done();
	}
}

//////////////////////////////////////////////////
void LogEvents::WriteTimelines()
{
	std::ofstream timeline_file;

	std::stringstream ss;

	ss << "timeline" << this->suffixTime << ".html";

	timeline_file.open(ss.str().c_str());

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
	for(std::map<std::string, std::list<sg_pp::GzEvent*> >::const_iterator m_it = this->nameToEvents_M.begin();
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

