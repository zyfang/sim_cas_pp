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

#include "LogParticles.hh"

using namespace sg_pp;
using namespace gazebo;
using namespace mongo;


//////////////////////////////////////////////////
LogParticles::LogParticles(const gazebo::physics::WorldPtr _world,
		const std::string _db_name,
		const std::string _coll_name)
	: world(_world)
	, dbName(_db_name)
	, collName(_coll_name)
{
	// get the world models
	this->models = this->world->GetModels();

	// get the world models
	this->contactManagerPtr = this->world->GetPhysicsEngine()->GetContactManager();

	// get values from the config file
	LogParticles::ReadConfigFile();
}

//////////////////////////////////////////////////
LogParticles::~LogParticles()
{

}

//////////////////////////////////////////////////
void LogParticles::ReadConfigFile()
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


//	this->publishTF = cfg.lookup("tf.publish");
//	std::cout << "LogParticles - publish TF: " << this->publishTF << std::endl;


}

//////////////////////////////////////////////////
void LogParticles::InitParticles()
{
    std::cout << "*LogParticles* - Sim start: " << this->world->GetSimTime().Double() << std::endl;

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
                if (m_iter->get()->GetName() == "LiquidTangibleThing")
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

                        // init the event collision with an empty set (models that are in collision with)
                        this->eventCollToSetOfModelNames_M[c_iter->get()] = std::set<std::string>();

                        // init the event coll to particle names map with empty sets of strings
                        this->eventCollToSetOfParticleNames_M[c_iter->get()] = std::set<std::string>();
                    }
                }
            }
        }
    }
}

//////////////////////////////////////////////////
void LogParticles::WriteParticleData()
{
	// set diff detected flag to false
    bool diff_detected = false;

    // compute simulation time in milliseconds
    const double timestamp_ms = this->world->GetSimTime().Double();

    // get all the contacts from the physics engine
    const std::vector<physics::Contact*> _contacts = this->contactManagerPtr->GetContacts();

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
        if (coll1 == this->eventCollisionForeFinger && coll2->GetParentModel()->GetName()!="Hand" || coll2 == this->eventCollisionForeFinger && coll1->GetParentModel()->GetName()!="Hand")
        {
            // if one of the collisions is the fore finger, set contact flag to true, and save both collisions
            fore_finger_contact = true;
            grasp_coll1 = coll1;
            grasp_coll2 = coll2;
        }
        else if (coll1 == this->eventCollisionThumb && coll2->GetParentModel()->GetName()!="Hand" || coll2 == this->eventCollisionThumb && coll1->GetParentModel()->GetName()!="Hand")
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
                if (coll1->GetModel()->GetName() == "LiquidTangibleThing")
                {
                    // add to poured set, which also checks for duplicates
                    this->pouredLiquidCollisions_S.insert(coll1);
                }
                else if (coll2->GetModel()->GetName() == "LiquidTangibleThing")
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
        // create pancake when the Spatula is grasped, save all particles belonging to the pancake
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
        if (grasp_coll1->GetParentModel()->GetName() == "Hand")
        {
            grasped_model_name = grasp_coll2->GetParentModel()->GetName();
            // std::cout << "************************** collision 2" << grasp_coll2->GetName() << " and " << grasp_coll1->GetName() << std::endl;
        }
        else
        {
            grasped_model_name = grasp_coll1->GetParentModel()->GetName();
            // std::cout << "************************** collision 1" << grasp_coll1->GetName() << std::endl;
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
    if (!this->pancakeCreated && grasped_model_name == "Spatula")
    {
        ////////////// Loop through all the contacts
        for (unsigned int i = 0; i < _contacts.size(); i++)
        {
            // collision 1 and 2 of the contact
            physics::Collision* coll1 = _contacts.at(i)->collision1;
            physics::Collision* coll2 = _contacts.at(i)->collision2;

            // save the particles belonging to the pancake
            if ((coll1->GetName() == "pancake_maker_event_collision")
                    && (coll2->GetModel()->GetName() == "LiquidTangibleThing"))
            {
                this->pancakeCollision_S.insert(coll2);
            }
            else if((coll2->GetName() == "pancake_maker_event_collision")
                    && (coll1->GetModel()->GetName() == "LiquidTangibleThing"))
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
        // ScopedDbConnection scoped_connection("localhost:27018");
        ScopedDbConnection scoped_connection("localhost");

        // insert document object into the database
        scoped_connection->insert(this->dbCollName + "." + this->collName + "_particles", doc_bo_builder.obj());

        // let the pool know the connection is done
        scoped_connection.done();
    }
}