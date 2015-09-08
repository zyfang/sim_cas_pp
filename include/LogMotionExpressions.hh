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
#include <mongo/client/dbclient.h>
#include <giskard/giskard.hpp>

namespace sg_pp
{
  class LogMotionExpressions
  {
    public:
      LogMotionExpressions(const gazebo::physics::WorldPtr& world, const std::string& db_name,
          const std::string& coll_name, const std::string connection_name);

      void Init();
  
      virtual ~LogMotionExpressions() {}
  
      void WriteRawData();
  
    private:
      void ReadConfigFile();
      void ReadMotionExpressions();
      double GetTimestamp();
      const std::vector<double>& GetExpressionValues();
      std::vector<double> GetObservables();
 
      gazebo::physics::WorldPtr world_;
      gazebo::physics::ModelPtr controlled_model_, observed_model_;
      std::string db_name_, coll_name_, motion_file_,conn_name_;
      std::vector<std::string> expression_names_;
      std::vector< KDL::Expression<double>::Ptr > expressions_;
      std::vector<double> expression_values_;
  };

  //
  // various utility functions
  //

  inline mongo::BSONObj to_bson(const std::string& name, double value)
  {
    using namespace mongo;
    return BSON("name" << name << "value" << value);
  }

  inline mongo::BSONObj to_bson(const std::vector<std::string>& names,
      const std::vector<double>& values, double timestamp)
  {
    assert(names.size() == values.size());

    using namespace mongo;

    BSONArrayBuilder ab;
    for(size_t i=0; i<names.size(); ++i)
      ab.append(to_bson(names[i], values[i]));

    BSONArray a = ab.arr();
    
    return BSON("expressions" << a << "timestamp" << timestamp);
  }

  inline KDL::Rotation toKDL(const gazebo::math::Quaternion& rot)
  {
    return KDL::Rotation::Quaternion(rot.x, rot.y, rot.z, rot.w);
  }

  inline std::vector<double> toSTL(const gazebo::math::Vector3& v)
  {
    std::vector<double> result;
    result.push_back(v.x);
    result.push_back(v.y);
    result.push_back(v.z);
    return result;
  }

  inline std::vector<double> toSTL(const gazebo::math::Quaternion& q) 
  {
    double alpha, beta, gamma;
    toKDL(q).GetEulerZYX(alpha, beta, gamma);

    std::vector<double> result;
    result.push_back(alpha);
    result.push_back(beta);
    result.push_back(gamma);

    return result;
  }

  inline std::vector<double> conc(const std::vector<double>& a, const std::vector<double>& b)
  {
    std::vector<double> result = a;
    for(size_t i=0; i<b.size(); ++i)
      result.push_back(b[i]);
    return result;
  }

  inline std::vector<double> toSTL(const gazebo::math::Pose& p)
  {
    return conc(toSTL(p.pos), toSTL(p.rot));
  }
}
#endif
