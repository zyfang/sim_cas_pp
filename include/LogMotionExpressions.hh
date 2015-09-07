/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Georg Bartels <georg.bartels@cs.uni-bremen.de>,
 *  Institute for Artificial Intelligence, Universität Bremen.
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

#ifndef LOG_MOTION_EXPRESSIONS_HH
#define LOG_MOTION_EXPRESSIONS_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
//#include <mongo/client/dbclient.h>
#include <giskard/giskard.hpp>

namespace sg_pp
{
  class LogMotionExpressions
  {
    public:
      LogMotionExpressions(const gazebo::physics::WorldPtr& world, const std::string& db_name,
          const std::string& coll_name);

      void Init();
  
      virtual ~LogMotionExpressions() {}
  
      void WriteRawData();
  
    private:
      void ReadConfigFile();
      void ReadMotionExpressions();
 
//      /// \brief Return a contact bson object
//  	private: mongo::BSONObj CreateBSONContactObject(
//  			const gazebo::physics::Contact* _contact,
//  			const gazebo::physics::Collision* _collision);
//  
//  	/// \brief Return a collision bson object
//  	private: mongo::BSONObj CreateBSONCollisionObject(
//  			const gazebo::physics::CollisionPtr _collision,
//  			const mongo::BSONArray _contact_arr);
//  
//  	/// \brief Return a link bson object
//  	private: mongo::BSONObj CreateBSONLinkObject(
//  			const gazebo::physics::LinkPtr _link,
//  			const mongo::BSONArray _collision_arr);
//  
//  	/// \brief Return a link bson object
//  	private: mongo::BSONObj CreateBSONModelObject(
//  			const gazebo::physics::ModelPtr _model,
//  			const mongo::BSONArray _link_arr);
  
      gazebo::physics::WorldPtr world_;
      std::string db_name_, coll_name_, motion_file_;
      std::vector<std::string> expression_names_;
      std::vector< KDL::Expression<double>::Ptr > expressions_;
  };
}
#endif
