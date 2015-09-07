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

#ifndef LOG_PARTICLES_HH
#define LOG_PARTICLES_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <libconfig.h++>
#include <mongo/client/dbclient.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

/// \brief Post Processing classes namespace
namespace sg_pp
{
/// \brief class LogParticles
class LogParticles
{
	/// \brief Constructor
	public: LogParticles(const gazebo::physics::WorldPtr _world,
			const std::string _db_name,
			const std::string _coll_name);

	/// \brief Destructor
	public: virtual ~LogParticles();

	/// \brief Initialise particles
	public: void InitParticles();

    /// \brief Write raw data to mongodb
    public: void WriteParticleData();

	/// \brief Load config file
	private: void ReadConfigFile();

	/// \brief Gazebo world
	private: const gazebo::physics::WorldPtr world;

	/// \brief Vector of the world models
	private: gazebo::physics::Model_V models;

	/// \brief Database name
	private: const std::string dbName;

	/// \brief Db collection name
	private: const std::string collName;

	/// \brief pointer of ContactManager, for getting contacts from physics engine
	private: gazebo::physics::ContactManager *contactManagerPtr;

	/// \brief Mug top event collision
    private: gazebo::physics::Collision* eventCollisionMug;

    /// \brief Hit hand thumb and fore finger event collision
    private: gazebo::physics::Collision *eventCollisionForeFinger, *eventCollisionThumb;

    /// \brief Event no contact collision vector
    private: std::set<gazebo::physics::Collision*> eventCollisions_S;

	/// \brief all particle collisions
    private: std::set<gazebo::physics::Collision*> allLiquidCollisions_S;

    /// \brief poured particle collisions
    private: std::set<gazebo::physics::Collision*> pouredLiquidCollisions_S;

    /// \brief particle collisions belonging to the pancake
    private: std::set<gazebo::physics::Collision*> pancakeCollision_S;

	/// \brief map of event collisions to a set of all its contacts model names
    private: std::map<gazebo::physics::Collision*, std::set<std::string> > eventCollToSetOfModelNames_M;

    /// \brief map of event collisions to a set of all its particle names
    private: std::map<gazebo::physics::Collision*, std::set<std::string> > eventCollToSetOfParticleNames_M;

    /// \brief name of the grasped model
    private: std::string graspedModelName;

    /// \brief flag for when the pancake is created
    private: bool pancakeCreated;
};
}
#endif
