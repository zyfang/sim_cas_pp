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

//////////////////////////////////////////////////
LogEvents::LogEvents(const gazebo::physics::WorldPtr _world,
        const std::string _db_name,
        const std::string _coll_name,
        const std::string _connection_name,
        const int _timeoffset)
    : world(_world)
    , dbName(_db_name)
    , collName(_coll_name)
    , connName(_connection_name)
    , TIME_OFFSET(_timeoffset)
{
    // get the world models
    this->contactManagerPtr = this->world->GetPhysicsEngine()->GetContactManager();

    // get values from the config file
    LogEvents::ReadConfigFile();

    // set the grasp init flag to false
    this->graspInit = false;

    // if events are exported as owl  init ros
    if(this->logLocation == "owl" || this->logLocation == "all")
    {
        // intialize ROS
        int argc = 0;
        char** argv = NULL;
        ros::init(argc, argv, "owl_events");
    }
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

    this->transfEvDurThresh = cfg.lookup("events.transf_ev_dur_thresh");
    std::cout << "*LogEvents* - transf_ev_dur_thresh: " << this->transfEvDurThresh << std::endl;

    this->eventDiscThresh = cfg.lookup("events.ev_disc_thresh");
    std::cout << "*LogEvents* - ev_disc_thresh: " << this->eventDiscThresh << std::endl;

    // get the grasp collisions from config file
    libconfig::Setting& grasp_coll_cf = cfg.lookup("events.grasp_colls");
    std::cout << "*LogEvents* - grasp_colls: " << std::endl;

    // insert surface coll names to set
    for(int i = 0; i < grasp_coll_cf.getLength(); ++i)
    {
        this->graspCollNames.insert(grasp_coll_cf[i]);
        std::cout << "\t" << grasp_coll_cf[i].c_str() << std::endl;
    }

    // get the surface collisions from config file
    libconfig::Setting& surface_coll_cf = cfg.lookup("events.surface_colls");
    std::cout << "*LogEvents* - surface_colls: " << std::endl;

    // insert surface coll names to set
    for(int i = 0; i < surface_coll_cf.getLength(); ++i)
    {
        this->surfaceCollNames.insert(surface_coll_cf[i]);
        std::cout << "\t" << surface_coll_cf[i].c_str() << std::endl;
    }

    // get the translation pairs <container top sensor, [particle types] >
    libconfig::Setting& transl_colls_cf = cfg.lookup("events.transl_colls");
    std::cout << "*LogEvents* - transl_colls: " << std::endl;

    // map container (transl) coll to particle names
    for(int i = 0; i < transl_colls_cf.getLength(); ++i)
    {
        // key - name of the container coll
        const std::string coll_key = transl_colls_cf[i][0].c_str();
        std::cout << "\t" << coll_key << " -> ";

        // values - names of the particle types to check for collision
        std::set<std::string> particle_names;

        // loop through the rest, and add particle names
        // notice that we skip the first value (j = 0) which is the coll name (key)
        for(int j = 1; j < transl_colls_cf[i].getLength(); ++j)
        {
            particle_names.insert(transl_colls_cf[i][j].c_str());

            std::cout << transl_colls_cf[i][j].c_str() << "; ";
        }
        std::cout << std::endl;

        // add <key - value> to the map
        this->contCollToPNames.insert({coll_key, particle_names});
    }

    // get the tool collision pairs <tool sensor, [particle types] >
    libconfig::Setting& tool_colls_cf = cfg.lookup("events.tool_colls");
    std::cout << "*LogEvents* - tool_colls: " << std::endl;

    // map tool coll to particle names
    for(int i = 0; i < tool_colls_cf.getLength(); ++i)
    {
        // key - name of the container coll
        const std::string coll_key = tool_colls_cf[i][0].c_str();
        std::cout << "\t" << coll_key << " -> ";

        // values - names of the particle types to check for collision
        std::set<std::string> particle_names;

        // loop through the rest, and add particle names
        // notice that we skip the first value (j = 0) which is the coll name (key)
        for(int j = 1; j < tool_colls_cf[i].getLength(); ++j)
        {
            particle_names.insert(tool_colls_cf[i][j].c_str());

            std::cout << tool_colls_cf[i][j].c_str() << "; ";
        }
        std::cout << std::endl;

        // add <key - value> to the map
        this->toolCollToPNames.insert({coll_key, particle_names});
    }
}

//////////////////////////////////////////////////
void LogEvents::InitEvents()
{
    std::cout << "*LogEvents* - Sim start: " << this->world->GetSimTime().Double() << " + timeoffset " << TIME_OFFSET << std::endl;

    // init ts for transfer event
    this->transfTs = this->world->GetSimTime().Double()+TIME_OFFSET;

    // TODO add events init
    this->transfEvent = NULL;

    this->graspGzEvent = NULL;

    this->toolGzEvent = NULL;

    // open the main GzEvent
    this->nameToEvents_M["Main"].push_back( //TODO generalise episode name
            new PpEvent("Main","&knowrob_sim;", "KitchenEpisode", this->world->GetSimTime().Double()+TIME_OFFSET));

    // Look up the no contact collisions (sensors)
    // interate through all the models to see which have event collisions
    for(const auto m_iter : this->world->GetModels())
    {
        // TODO look into this, why not just create the object on spot?
        // THIS needed to ensore unique hashing of the object names when creating contexts
        // map model name to the GzEventObj object
        this->nameToEventObj_M[m_iter->GetName()] = new PpEventObj(m_iter->GetName());

        // TODO make ignore list?
        // NEEDED as well to esablish uniqueness of the context object, see if the other is still needed
        // map model name to the beliefstate object
        this->nameToBsObject_M[m_iter->GetName()] =
                new beliefstate_client::Object("&knowrob;", m_iter->GetName());

        // loop through the links
        for (const auto l_iter : m_iter->GetLinks())
        {
            // loop through all the collision
            for (const auto c_iter : l_iter->GetCollisions())
            {
                // if the collision is without physical contact then add it to the map
                if (c_iter->GetSurface()->collideWithoutContact)
                {

                    // check if grasp coll type
                    if (this->graspCollNames.find(c_iter->GetName()) != this->graspCollNames.end())
                    {
                        this->graspColls.insert(c_iter.get());
                    }

                    // check if surface collision type
                    if (this->surfaceCollNames.find(c_iter->GetName()) != this->surfaceCollNames.end())
                    {
                        this->surfaceColls.insert(c_iter.get());
                    }

                    // check if the container collision (transl) type
                    // initialize the particles as 'untransfered'
                    const auto cont_res = this->contCollToPNames.find(c_iter->GetName());

                    if (cont_res != this->contCollToPNames.end())
                    {
                        // particle models
                        std::set< gazebo::physics::ModelPtr> particle_models;

                        // loop through particle names to find the models and add them to the vector
                        for (const auto p_it : cont_res->second)
                        {
                            // get the particle model
                            physics::ModelPtr particle_model = this->world->GetModel(p_it);

                            // insert model to the set
                            particle_models.insert(particle_model);

                            // init the models particles (collisions) as untransfered
                            // map of link to the transfered flag
                            std::map<physics::Collision*, bool> particles_coll_map;

                            // loop through all links and get the first collision
                            for (const auto l_it : particle_model->GetLinks())
                            {
                                physics::Collision* first_coll = l_it->GetCollisions()[0].get();
                                particles_coll_map[first_coll] = false;
                            }

                            // map the particles model to the untransfered links
                            // set map< model - map< link - bool > > to FALSE
                            this->transferedParticles[this->world->GetModel(p_it)] = particles_coll_map;

                        }

                        // map container collision to all particle model vector
                        this->contCollToPModels[c_iter.get()] = particle_models;
                    }

                    // check if tool collision type
                    const auto tool_res = this->toolCollToPNames.find(c_iter->GetName());

                    if (tool_res != this->toolCollToPNames.end())
                    {
                        // particle models
                        std::set< gazebo::physics::ModelPtr> particle_models;

                        // loop through particle names to find the models and add them to the vector
                        for (const auto p_it : tool_res->second)
                        {
                            // TODO don't segault if not in world
                            // get the particle model
                            physics::ModelPtr particle_model = this->world->GetModel(p_it);

                            // insert model to the set
                            particle_models.insert(particle_model);
                        }

                        // map tool collision to all particle model vector
                        this->toolCollToPModels[c_iter.get()] = particle_models;
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
    const double timestamp_ms = this->world->GetSimTime().Double()+TIME_OFFSET;
    // double timestamp_ms = this->world->GetSimTime().nsec / 1000000.0 + this->world->GetSimTime().sec * 1000.0;

    // state of models being in contact with surfaces
    // <surface model name, model in contact with>
    std::set<std::pair<std::string, std::string> > surface_models_in_contacts;

    // nr of contacts with the grasping 'sensors'
    // grasping happens if all 'sensors' have a contact ( nr_grasp_contacts = graspColl.size )
    unsigned int grasp_contacts_nr = 0;

    // models in contact with grasping 'sensors',
    // grasping happens if all models are the same (e.g. set size = 1)
    std::set<std::string> grasped_models;

    // TODO not generalized
    // model in contact with the tool
    std::string contact_with_tool_model;
    std::string tool_name;


    // flag for detecting particle correct transfer
    bool transf_detect_flag = false;

    ////////////// Iterate through all current contacts
    // By iterating through all the contacts we check their types
    // and compare them to the prev state to see if changes occured
    for(const auto c_it : this->contactManagerPtr->GetContacts())
    {
        // get the collisions in contact
        physics::Collision* c_coll1 = c_it->collision1;
        physics::Collision* c_coll2 = c_it->collision2;


        ////////////// Supporting Collisions
        /// add model names contact with a surface collision ('sensor')
        /// Set(<model1_name, model2_name>, [..])

        // check if the contact collision pair (first, second) are in contact with a surface 'sensor'
        if (this->surfaceColls.find(c_coll1) != this->surfaceColls.end())
        {
            // add the model names pair to the set
            surface_models_in_contacts.insert(std::pair<std::string, std::string>(
                    c_coll1->GetParentModel()->GetName(),
                    c_coll2->GetParentModel()->GetName()));
        }
        else if(this->surfaceColls.find(c_coll2) != this->surfaceColls.end())
        {
            // add the model names pair to the set
            surface_models_in_contacts.insert(std::pair<std::string, std::string>(
                    c_coll2->GetParentModel()->GetName(),
                    c_coll1->GetParentModel()->GetName()));
        }


        /////////////// Grasping
        /// check if any of the collision pair is in contact with a grasping 'sensor',
        /// if in contact, increment the grasping contact number
        /// and add the model to the grasped models set,
        /// if the contact nr = nr sensors, and all models in contact with = 1, then we have a grasp

        // check any of the collisions pair is in contact with a grasping 'sensor'
        //if one of them is the grasping sensor, the other one would be the grasp model
        //to prevent self-collision, check that the other model is not the same as the collision model checked
        if (this->graspColls.find(c_coll1) != this->graspColls.end())
        {
            if(c_coll2->GetParentModel()->GetName()!=c_coll1->GetParentModel()->GetName())
            {
                // increase the current number of grasp contacts
                grasp_contacts_nr++;

                // add model to the grasped objects
                // notice the model is the opposite from the checked collision
                grasped_models.insert(c_coll2->GetParentModel()->GetName());
            }

        }
        else if(this->graspColls.find(c_coll2) != this->graspColls.end())
        {
            if(c_coll2->GetParentModel()->GetName()!=c_coll1->GetParentModel()->GetName())
            {
                // increase the current number of grasp contacts
                grasp_contacts_nr++;

                // add model to the grasped objects
                // notice the model is the opposite from the checked collision
                grasped_models.insert(c_coll1->GetParentModel()->GetName());
            }
        }


        /////////////// Translation (Particles leaving container)
        /// Check if one of the collision is the container top 'sensor',
        /// if yes, check if the sensor is in contact with one of the related particles
        /// e.g. `mug_top_ev` coll is in collision with `Cheese` or `Sauce` etc. Model

        // check any of the collisions pair is in contact with a container top 'sensor'
        // get the results for finding the collisions in the map
        const auto coll1_to_model_res = this->contCollToPModels.find(c_coll1);
        const auto coll2_to_model_res = this->contCollToPModels.find(c_coll2);

        // c_coll2 is the particle in contact with the container sensor
        // if results were found check if they are in contact with the related particle model
        if (coll1_to_model_res != this->contCollToPModels.end())
        {
            // check if the object in collision with the required particle type
            // ->second is the set of the particle models
            const auto coll1_model =
                    coll1_to_model_res->second.find(c_coll2->GetParentModel());

            // if the model belongs to the related set
            if (coll1_model != coll1_to_model_res->second.end())
            {
                // if particle has not been transfered yet, increment counter and set flag to true
                // save the timestamp of the particle leaving
                if(!this->transferedParticles[(*coll1_model)][c_coll2])
                {
                    // increment counter
                    this->transferedPartCount[(*coll1_model)]++;

                    // set flag to transfered (true)
                    this->transferedParticles[(*coll1_model)][c_coll2] = true;

                    // save particle transfer timestamp
                    this->transfTs = timestamp_ms;

                    // set flag to true
                    transf_detect_flag = true;

                    // add the particle collision to the transf event pool
                    this->transfParticlePool.push_back(c_coll2);
                }
            }
        }
        // c_coll1 - particle in collision with the container sensor
        else if (coll2_to_model_res != this->contCollToPModels.end())
        {
            // check if the object in collision with the required particle type
            // ->second is the set of the particle models
            const auto coll2_model =
                    coll2_to_model_res->second.find(c_coll1->GetParentModel());

            // if the model belongs to the related set
            if (coll2_model != coll2_to_model_res->second.end())
            {
                // if particle has not been transfered yet, increment counter and set flag to true
                // save the timestamp of the particle leaving
                if(!this->transferedParticles[(*coll2_model)][c_coll1])
                {
                    // increment counter
                    this->transferedPartCount[(*coll2_model)]++;

                    // set flag to transfered (true)
                    this->transferedParticles[(*coll2_model)][c_coll1] = true;

                    // save particle transfer timestamp
                    this->transfTs = timestamp_ms;

                    // set flag to true
                    transf_detect_flag = true;

                    // add the particle collision to the transf event pool
                    this->transfParticlePool.push_back(c_coll1);
                }
            }
        }

        /////////////// Tool (Particles in contact with tool)
        /// Check if one of the collision is the tool coll 'sensor',
        /// if yes, check if the sensor is in contact with one of the related particles
        /// e.g. `spoon_head_coll` coll is in collision with `Cheese` or `Sauce` etc. Model

        // check any of the collisions pair is in contact with a container top 'sensor'
        // get the results for finding the collisions in the map
        const auto tool_coll1_to_model_res = this->toolCollToPModels.find(c_coll1);
        const auto tool_coll2_to_model_res = this->toolCollToPModels.find(c_coll2);

        // c_coll2 is the particle in contact with the container sensor
        // if results were found check if they are in contact with the related particle model
        if (tool_coll1_to_model_res != this->toolCollToPModels.end())
        {
            // check if the object in collision with the required particle type
            // ->second is the set of the particle models
            const auto coll1_model =
                    tool_coll1_to_model_res->second.find(c_coll2->GetParentModel());

            // TODO this just takes the last coll
            // if the model belongs to the related set
            if (coll1_model != tool_coll1_to_model_res->second.end())
            {
                contact_with_tool_model = c_coll2->GetParentModel()->GetName();
                tool_name = c_coll1->GetParentModel()->GetName();
            }
        }
        // c_coll1 - particle in collision with the container sensor
        else if (tool_coll2_to_model_res != this->toolCollToPModels.end())
        {
            // check if the object in collision with the required particle type
            // ->second is the set of the particle models
            const auto coll2_model =
                    tool_coll2_to_model_res->second.find(c_coll1->GetParentModel());

            // TODO this just takes the last coll
            // if the model belongs to the related set
            if (coll2_model != tool_coll2_to_model_res->second.end())
            {
                contact_with_tool_model = c_coll1->GetParentModel()->GetName();
                tool_name = c_coll2->GetParentModel()->GetName();
            }
        }
    }

    ////////////////////////////////////////////////////////////////
    ////////////// Compare current state with previous one

    // Check if difference between the supporting events have appeared
    LogEvents::CheckSurfaceEvents(
                timestamp_ms, surface_models_in_contacts);

    // Check if the grasp has changed
    LogEvents::CheckGraspEvent(
                timestamp_ms, grasp_contacts_nr, grasped_models);

    // Check transfer event
    LogEvents::CheckTranslEvent(
                timestamp_ms, transf_detect_flag);

    // Check tool event
    LogEvents::CheckToolEvent(
                timestamp_ms, contact_with_tool_model, tool_name);

}

//////////////////////////////////////////////////
void LogEvents::FiniEvents()
{
    // close main GzEvent
    this->nameToEvents_M["Main"].back()->End(this->world->GetSimTime().Double()+TIME_OFFSET);

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
void LogEvents::CheckGraspEvent(
        const double _timestamp_ms,
        const unsigned int _grasp_contacts_nr,
        const std::set<std::string> &_grasped_models)
{

    // current grasped model
    std::string curr_grasped_model;

    // check for grasp nr contacts eq the total grasp 'sensors'
    // and all of them are in contact with the same model
    if(_grasp_contacts_nr == this->graspColls.size() && _grasped_models.size() == 1)
    {
        // TODO only checks for the first model?
        curr_grasped_model = (*_grasped_models.begin());
    }

    // check for difference between current and prev grasp
    if (curr_grasped_model != this->prevGraspedModel)
    {
        if (!curr_grasped_model.empty())
        {
            this->prevGraspedModel = curr_grasped_model;

            // the name of the grasping event
            std::string grasp_ev_name = "Grasp-" + curr_grasped_model;


            // create the contact GzEvent
            this->graspGzEvent = new PpEvent(
                    grasp_ev_name, "&knowrob;", "GraspingSomething",
                    "knowrob:", "objectActedOn", _timestamp_ms);

            // add grasped object
            this->graspGzEvent->AddObject(
                        this->nameToEventObj_M[curr_grasped_model]);

            std::cout << "*LogEvents* - Start - \t" << grasp_ev_name << "\t\t\t at " << _timestamp_ms  << std::endl;

        }
        else
        {
            // delete name
            this->prevGraspedModel.clear();

            // end grasping event
            this->graspGzEvent->End(_timestamp_ms);

            //TODO in order to remove the this->graspGzEvent != NULL check, we can delete this->graspevent and set it to NULL
            // similarly to transl events (or the other way around)

            std::cout << "*LogEvents* - End - \t" << this->graspGzEvent->GetName() << "\t\t\t at " << _timestamp_ms  << std::endl;

            // add grasp event to the map of all events
            this->nameToEvents_M[this->graspGzEvent->GetName()].push_back(this->graspGzEvent);
        }

    }
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
void LogEvents::CheckSurfaceEvents(
        const double _timestamp_ms,
        const std::set<std::pair<std::string, std::string> > &_curr_surface_models_in_contact)
{
    // the symmetric difference will be added to this set
    std::set<std::pair<std::string, std::string> > symmetric_diff_S;

    // computing the symmetric diff between the curr and prev set
    std::set_symmetric_difference(
            _curr_surface_models_in_contact.begin(),
            _curr_surface_models_in_contact.end(),
            this->prevSurfaceModelsInContact.begin(),
            this->prevSurfaceModelsInContact.end(),
            std::inserter(symmetric_diff_S,symmetric_diff_S.end()));

    // check if there are any differences
    if (symmetric_diff_S.size() > 0)
    {
        // set the prev values to the current one
        this->prevSurfaceModelsInContact = _curr_surface_models_in_contact;

        // iterate through all the collising event models
        for(std::set<std::pair<std::string, std::string> >::const_iterator m_iter = symmetric_diff_S.begin();
                m_iter != symmetric_diff_S.end(); m_iter++)
        {
            // set name of the context first models + second model in contact
            std::string contact_ev_name = "Contact-" + m_iter->first + "-" + m_iter->second;


            // if event does not exist
            // TODO add two way detection of contacts, store somewhere the pairs in contact
            if(!this->nameToEvents_M.count(contact_ev_name))
            {
                // create local contact GzEvent
                sg_pp::PpEvent* contact_event = new sg_pp::PpEvent(
                        contact_ev_name, "&knowrob_sim;", "TouchingSituation",
                        "knowrob_sim:", "inContact", _timestamp_ms);

                // add the two objects in contact
                contact_event->AddObject(this->nameToEventObj_M[m_iter->first]);

                contact_event->AddObject(this->nameToEventObj_M[m_iter->second]);

                // add local event to the map
                this->nameToEvents_M[contact_ev_name].push_back(contact_event);

                std::cout << "*LogEvents* - Start - \t" << contact_ev_name << "\t\t at "
                          << _timestamp_ms << std::endl;
            }
            else
            {
                // if the contact exists and it is open, end it
                if(this->nameToEvents_M[contact_ev_name].back()->IsOpen())
                {
                    // end contact event
                    this->nameToEvents_M[contact_ev_name].back()->End(_timestamp_ms);

                    std::cout << "*LogEvents* - End - \t" << contact_ev_name << "\t\t at "
                              << _timestamp_ms << std::endl;
                }
                else
                {
                    // create local contact GzEvent
                    sg_pp::PpEvent* contact_event = new sg_pp::PpEvent(
                            contact_ev_name, "&knowrob_sim;", "TouchingSituation",
                            "knowrob_sim:", "inContact", _timestamp_ms);

                    // add the two objects in contact
                    contact_event->AddObject(this->nameToEventObj_M[m_iter->first]);

                    contact_event->AddObject(this->nameToEventObj_M[m_iter->second]);

                    // add local event to the map
                    this->nameToEvents_M[contact_ev_name].push_back(contact_event);

                    std::cout << "*LogEvents* - Start - \t" << contact_ev_name << "\t\t at "
                              << _timestamp_ms << std::endl;
                }
            }
        }
    }
}

//////////////////////////////////////////////////
void LogEvents::CheckTranslEvent(
        const double _timestamp_ms,
        const bool _transf_detect_flag)
{

    // check if new transfer was detected
    // if transl ev not init, create event
    // TODO add init to events
    if(_transf_detect_flag && this->transfEvent == NULL)
    {
        // TODO check for multiple particle types
        // save type of first particle from pool
        std::string particle_type = (*this->transfParticlePool.begin())->GetParentModel()->GetName();

        // TODO property namespace, and porperty should belong to the OBJECT
        // create the contact GzEvent
        this->transfEvent = new PpEvent(
                    "ParticleTranslation-" + particle_type, "&knowrob_sim;", "ParticleTranslation",
                    "knowrob:", "particleType", _timestamp_ms);

        this->transfEvent->AddObject(this->nameToEventObj_M[particle_type]);
//        this->translEvent->AddObject(new GzEventObj(particle_type));

        std::cout << "*LogEvents* - Start - \t" << this->transfEvent->GetName()
                  << " [" << particle_type << "]"
                  << "\t\t at " << _timestamp_ms << std::endl;
    }

    if((_timestamp_ms - this->transfTs > this->transfEvDurThresh) && this->transfEvent != NULL)
    {
        // TODO hack for getting the nr of particles
        // add the nr of particles scooped
//        this->translEvent->AddObject(
//                    new GzEventObj(std::to_string(this->transfParticlePool.size())));

        // finish gz event
        // end grasping event
        this->transfEvent->End(this->transfTs);

        std::cout << "*LogEvents* - End - \t" << this->transfEvent->GetName()
                  << " [" << this->transfParticlePool.size() << "]"
                  << "\t\t at " << this->transfTs << std::endl;

        // add grasp event to the map of all events
        this->nameToEvents_M[this->transfEvent->GetName()].push_back(this->transfEvent);

        // set pointer to NULL, do not delete it, since later we will access the memory using the saved events
        this->transfEvent = NULL;
        this->transfParticlePool.clear();
    }
}

//////////////////////////////////////////////////
void LogEvents::CheckToolEvent(
        const double _timestamp_ms,
        const std::string _contact_with_tool_model,
        const std::string _tool_name)
{
    // check for difference between current and prev grasp
    if (_contact_with_tool_model != this->prevContactWithToolModel)
    {
        if (!_contact_with_tool_model.empty())
        {
            this->prevContactWithToolModel = _contact_with_tool_model;

            // the name of the tool contact event
            std::string tool_contact_ev_name = "ToolContact-"
                    + _tool_name + "-" + _contact_with_tool_model;

            // TODO why is if needed here? the two parts seem similar
            // check if the tool contact event has been initialized
            if(this->toolGzEvent != NULL)
            {
                // create the contact GzEvent
                this->toolGzEvent = new sg_pp::PpEvent(
                            tool_contact_ev_name, "&knowrob_sim;", "TouchingSituation",
                            "knowrob_sim:", "inContact", _timestamp_ms);

                // add the two objects in contact
                this->toolGzEvent->AddObject(this->nameToEventObj_M[_tool_name]);

                this->toolGzEvent->AddObject(this->nameToEventObj_M[_contact_with_tool_model]);

                std::cout << "*LogEvents* - Start - \t" << tool_contact_ev_name
                          << "\t\t\t at " << _timestamp_ms  << std::endl;
            }
            // init first grasp
            else
            {
                // create the contact GzEvent
                this->toolGzEvent = new sg_pp::PpEvent(
                            tool_contact_ev_name, "&knowrob_sim;", "TouchingSituation",
                            "knowrob_sim:", "inContact", _timestamp_ms);

                // add the two objects in contact
                this->toolGzEvent->AddObject(this->nameToEventObj_M[_tool_name]);

                this->toolGzEvent->AddObject(this->nameToEventObj_M[_contact_with_tool_model]);

                std::cout << "*LogEvents* - Init - \t" << tool_contact_ev_name
                          << "\t\t\t at " << _timestamp_ms  << std::endl;            }
        }
        else
        {
            // delete name
            this->prevContactWithToolModel.clear();

            // end grasping event
            this->toolGzEvent->End(_timestamp_ms);

            //TODO in order to remove the this->graspGzEvent != NULL check, we can delete this->graspevent and set it to NULL
            // similarly to transl events (or the other way around)

            std::cout << "*LogEvents* - End - \t" << this->toolGzEvent->GetName()
                      << "\t\t\t at " << _timestamp_ms  << std::endl;

            // add grasp event to the map of all events
            this->nameToEvents_M[this->toolGzEvent->GetName()].push_back(this->toolGzEvent);
        }
    }
}

//////////////////////////////////////////////////
void LogEvents::EndActiveEvents()
{
    // iterate through the map
    for(std::map<std::string, std::list<sg_pp::PpEvent*> >::const_iterator m_it = this->nameToEvents_M.begin();
            m_it != this->nameToEvents_M.end(); m_it++)
    {
        // iterate through the events with the same name
        for(std::list<PpEvent*>::const_iterator ev_it = m_it->second.begin();
                ev_it != m_it->second.end(); ev_it++)
        {
            // if event still open end it at the end time
            if((*ev_it)->IsOpen())
            {
                (*ev_it)->End(this->world->GetSimTime().Double()+TIME_OFFSET);

                std::cout << "*LogEvents* - End - \t" << (*ev_it)->GetName() << "\t\t at "
                          << this->world->GetSimTime().Double() << " + timeoffset " << TIME_OFFSET << std::endl;
            }
        }
    }
}

//////////////////////////////////////////////////
void LogEvents::MergeEventDisconnections()
{
    std::cout << "*LogEvents* - Merging event disconnections" << std::endl;

    // iterate through the map
    for(std::map<std::string, std::list<sg_pp::PpEvent*> >::iterator m_it = this->nameToEvents_M.begin();
            m_it != this->nameToEvents_M.end(); m_it++)
    {
        // TODO use vector, with next?
        // iterate through the events with the same name
        for(std::list<PpEvent*>::iterator ev_it = m_it->second.begin();
                ev_it != m_it->second.end(); /*increment in the loop*/)
        {
            // save current event for comparison
            std::list<PpEvent*>::iterator curr_ev = ev_it;

            // increment current iteration (next event)
            ev_it++;

            // check that the next value is not the last
            if(ev_it != m_it->second.end())
            {
                // merge if the duration between the events is smaller than the given thresh
                if((*ev_it)->GetStartTime() - (*curr_ev)->GetEndTime() < this->eventDiscThresh)
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
    std::cout << "*LogEvents* - Writing contexts" << std::endl;

    // Write to owl file
    if(this->logLocation == "owl" || this->logLocation == "all")
    {
        // initialize the beliefstate
        this->beliefStateClient = new beliefstate_client::BeliefstateClient("bs_client");

        // starting new experiment
        this->beliefStateClient->startNewExperiment();

        // adding experiment name to the meta data
        this->beliefStateClient->setMetaDataField("experiment", this->collName);

        // TODO issue with setmatadatafield
        // adding experiment meta data with all the tools from the experiment
//        for(const auto m_iter : this->world->GetModels())
//        {
//            this->beliefStateClient->setMetaDataField("occuringObject", m_iter->GetName());
//        }

        // register the OWL namespace
        this->beliefStateClient->registerOWLNamespace("knowrob_sim",
                        "http://knowrob.org/kb/knowrob_sim.owl#");

        // iterate through the map
        for(std::map<std::string, std::list<sg_pp::PpEvent*> >::const_iterator m_it = this->nameToEvents_M.begin();
                m_it != this->nameToEvents_M.end(); m_it++)
        {
            // iterate through the events with the same name
            for(std::list<sg_pp::PpEvent*>::const_iterator ev_it = m_it->second.begin();
                    ev_it != m_it->second.end(); ev_it++)
            {
                // create local belief state context
                beliefstate_client::Context* curr_ctx;

                // open belief state context
                curr_ctx = new beliefstate_client::Context(this->beliefStateClient,
                        (*ev_it)->GetName(), (*ev_it)->GetClassNamespace(),
                        (*ev_it)->GetClass(), (*ev_it)->GetStartTime());

                // get the objects of the event
                std::vector<PpEventObj*> curr_objects = (*ev_it)->GetObjects();

                // iterate through the objects
                for(std::vector<PpEventObj*>::const_iterator obj_it = curr_objects.begin();
                        obj_it != curr_objects.end(); obj_it++)
                {

                    // TODO check first if object in the map, if not create a new one
                    // (hack version because of adding numbers as objects)
                    if (this->nameToBsObject_M.find((*obj_it)->GetName()) != this->nameToBsObject_M.end())
                    {
                        // TODO, this is needed to ensure unique hash of the same objects
                        // add object to the context
                        curr_ctx->addObject(
                                    this->nameToBsObject_M[(*obj_it)->GetName()],
                                (*ev_it)->GetPropertyNamespace() + (*ev_it)->GetProperty());
                    }
                    else // create new object
                    {
                        // TODO
                        // we hackingly add numbers as transl particles
                        curr_ctx->addObject(
                                    new beliefstate_client::Object("&knowrob;", (*obj_it)->GetName()),
                                    (*ev_it)->GetPropertyNamespace() + (*ev_it)->GetProperty());
                    }



                }

                // end belief state context
                curr_ctx->end(true, (*ev_it)->GetEndTime());
            }
        }

        // export belief state client
        this->beliefStateClient->exportFiles(this->collName);
    }

    // Write to mongodb
    if(this->logLocation == "mongo" || this->logLocation == "all")
    {

        // insert document object into the database, use scoped connection
        ScopedDbConnection scoped_connection(this->connName);

        // iterate through the map
        for(std::map<std::string, std::list<sg_pp::PpEvent*> >::const_iterator m_it = this->nameToEvents_M.begin();
                m_it != this->nameToEvents_M.end(); m_it++)
        {
            // TODO fix the grasp issue, (grasp event map includes the actual events with the right names)
            // iterate through the events with the same name
            for(std::list<PpEvent*>::const_iterator ev_it = m_it->second.begin();
                    ev_it != m_it->second.end(); ev_it++)
            {
                // all events
                BSONObjBuilder event_bb;

                // add to the time array
                event_bb.append("event", (*ev_it)->GetName())
                        .append("start",(*ev_it)->GetStartTime())
                        .append("end", (*ev_it)->GetEndTime());

                // insert document object into the database
                // stringstream strs;
                // strs << this->suffixTime;
                // string temp_str = strs.str();
                // const char* timechar = temp_str.c_str();
                scoped_connection->insert(this->dbName + "." + this->collName + "_ev", event_bb.obj());
            }
        }
        
        // let the pool know the connection is done
        scoped_connection.done();
    }
}

//////////////////////////////////////////////////
void LogEvents::WriteTimelines()
{
    std::cout << "*LogEvents* - Writing timelines" << std::endl;

    // create file
    std::ofstream timeline_file;

    // create directory
    boost::filesystem::create_directory("timelines");

    // path stringstream
    std::stringstream path_ss;

    path_ss << "timelines/tl_" << this->collName << ".html";

    // open and add values to the file
    timeline_file.open(path_ss.str().c_str());

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
    for(std::map<std::string, std::list<sg_pp::PpEvent*> >::const_iterator m_it = this->nameToEvents_M.begin();
            m_it != this->nameToEvents_M.end(); m_it++)
    {
        // iterate through the events with the same name
        for(std::list<PpEvent*>::const_iterator ev_it = m_it->second.begin();
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

    // close file
    timeline_file.close();
}

