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
#include "LogMotionExpressions.hh"
#include <libconfig.h++>

using namespace sg_pp;
using namespace gazebo;
using namespace mongo;

LogMotionExpressions::LogMotionExpressions(const gazebo::physics::WorldPtr& world, 
    const std::string& db_name, 
    const std::string& coll_name,
    const std::string connection_name,
    const int _timeoffset)
    : world_( world ), 
    db_name_( db_name ), 
    coll_name_( coll_name ),
    conn_name_( connection_name),
    TIME_OFFSET(_timeoffset)
{
  expression_names_.clear();
  expression_names_.push_back("mug-bottom-behind-maker");
  expression_names_.push_back("mug-bottom-left-maker");
  expression_names_.push_back("mug-bottom-above-maker");
  expression_names_.push_back("mug-top-behind-maker");
  expression_names_.push_back("mug-top-left-maker");
  expression_names_.push_back("mug-top-above-maker");
  expression_names_.push_back("mug-behind-itself");
  expression_names_.push_back("mug-left-itself");
  expression_names_.push_back("mug-above-itself");

  expression_values_.resize(expression_names_.size());
}

void LogMotionExpressions::Init()
{
  ReadConfigFile();
  ReadMotionExpressions();
} 
  
void LogMotionExpressions::WriteRawData()
{
  ScopedDbConnection scoped_connection(this->conn_name_);
  scoped_connection->insert(this->db_name_ + "." + this->coll_name_ + "_motion_expressions", 
      to_bson(expression_names_, GetExpressionValues(), GetTimestamp()));
  scoped_connection.done();
}
  
void LogMotionExpressions::ReadConfigFile()
{
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

  this->motion_file_ = cfg.lookup("motion_expressions.motion_file").c_str();
  std::cout << "*LogMotionExpressions* - motion file: " << this->motion_file_ << std::endl;

  std::string controlled_model_name = cfg.lookup("motion_expressions.controlled_model").c_str();
  std::cout << "*LogMotionExpressions* - controlled model: " << controlled_model_name << std::endl;
  controlled_model_ = world_->GetModel(controlled_model_name);
  assert(controlled_model_.get());

  std::string observed_model_name = cfg.lookup("motion_expressions.observed_model").c_str();
  std::cout << "*LogMotionExpressions* - observed model: " << observed_model_name << std::endl;
  observed_model_ = world_->GetModel(observed_model_name);
  assert(observed_model_.get());
}

void LogMotionExpressions::ReadMotionExpressions()
{
  YAML::Node node = YAML::LoadFile(motion_file_);

  std::vector<giskard::ScopeEntry> scope_spec = node.as< std::vector<giskard::ScopeEntry> >();

  giskard::Scope scope = giskard::generate(scope_spec);

  expressions_.clear();
  for(size_t i=0; i<expression_names_.size(); ++i)
    expressions_.push_back(scope.find_double_expression(expression_names_[i]));
}

double LogMotionExpressions::GetTimestamp()
{
  return world_->GetSimTime().Double()+TIME_OFFSET;
  // return world_->GetSimTime().nsec / 1000000.0 + this->world_->GetSimTime().sec * 1000.0;

}

const std::vector<double>& LogMotionExpressions::GetExpressionValues()
{
  assert(expressions_.size() == expression_values_.size());
  
  std::vector<double> observables = GetObservables();
  for(size_t i=0; i<expressions_.size(); ++i)
  {
    expressions_[i]->setInputValues(observables);
    expression_values_[i] = expressions_[i]->value();
  }

  return expression_values_;
}

std::vector<double> LogMotionExpressions::GetObservables()
{
  return conc(toSTL(controlled_model_->GetLinks()[0]->GetWorldPose()),
              toSTL(observed_model_->GetLinks()[0]->GetWorldPose()));
}
