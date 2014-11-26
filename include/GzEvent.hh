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

#ifndef GZEVENT_HH_
#define GZEVENT_HH_

#include <string>
#include <vector>
#include <iostream>
#include "GzEventObj.hh"

namespace kgpp
{

/// \brief class GzEvent
class GzEvent
{
	/// \brief Constructor
	public: GzEvent(const std::string _name,
			const std::string _class_namespace,
			const std::string _class_type,
			double _start_time);

	/// \brief Constructor
	public: GzEvent(const std::string _name,
			const std::string _class_namespace,
			const std::string _class_type,
			const std::string _property_namespace,
			const std::string _property,
			double _start_time);


	/// \brief Destructor
	public: virtual ~GzEvent();

	/// \brief Get the Id
	public: int GetId();

	/// \brief Get the event name
	public: const std::string GetName();

	/// \brief Get the class namespace
	public: const std::string GetClassNamespace();

	/// \brief Get the class type
	public: const std::string GetClass();

	//TODO property is connected with object
	/// \brief Get the property namespace
	public: const std::string GetPropertyNamespace();

	/// \brief Get the property type
	public: const std::string GetProperty();

	/// \brief Check if event is open
	public: bool IsOpen();

	/// \brief Get the start time
	public: double GetStartTime();

	/// \brief Set the start time
	public: void SetStartTime(double _start_time);

	/// \brief Get the start time
	public: double GetEndTime();

	/// \brief Set the start time
	public: void SetEndTime(double _end_time);

	/// \brief Get the start time
	public: double GetDuration();

	/// \brief Start Event
	public: void Start(double _start_time);

	/// \brief End Event
	public: void End(double _end_time);

	/// \brief Add object
	public: void AddObject(GzEventObj *_ev_obj);

	/// \brief Get all objects
	public: std::vector<GzEventObj*> GetObjects();

	/// \brief Unique ID of the event
	private: int id;

	/// \brief Event name
	private: const std::string name;

	/// \brief Event class namespace
	private: const std::string classNamespace;

	/// \brief Event class type
	private: const std::string classType;

	/// \brief Event property type
	private: const std::string propertyNamespace;

	/// \brief Event property type
	private: const std::string property;

	/// \brief State of the event, open, closed
	private: bool isOpen;

	/// \brief Start time of the event
	private: double startTime;

	/// \brief End time of the event
	private: double endTime;

	/// \brief Objects involved in the event
	private: std::vector<GzEventObj*> objects;
};

}

#endif /* GZEVENT_HH_ */
