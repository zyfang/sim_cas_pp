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

#include "GzEventObj.hh"

namespace hand_sim
{

/// \brief class GzEvent
class GzEvent
{
	/// \brief Constructor
	public: GzEvent(const std::string _name);

	/// \brief Constructor
	public: GzEvent(const std::string _name, long int _start_time);

	/// \brief Destructor
	public: virtual ~GzEvent();

	/// \brief Get the Id
	public: int GetId();

	/// \brief Get the event name
	public: const std::string GetName();

	/// \brief Get the start time
	public: long int GetStartTime();

	/// \brief Set the start time
	public: void SetStartTime(long int _start_time);

	/// \brief Get the start time
	public: long int GetEndTime();

	/// \brief Set the start time
	public: void SetEndTime(long int _end_time);

	/// \brief Get the start time
	public: long int GetDuration();

	/// \brief Start Event
	public: void StartEvent(long int _start_time);

	/// \brief Close Event
	public: void CloseEvent(long int _end_time);

	/// \brief Add object
	public: void AddObject(GzEventObj _ev_obj);

	/// \brief Get all objects
	public: std::vector<GzEventObj> GetObjects();

	/// \brief Unique ID of the event
	private: static int id;

	/// \brief Event name
	private: const std::string name;

	/// \brief State of the event, open, closed
	private: bool isOpen;

	/// \brief Start time of the event
	private: long int startTime;

	/// \brief End time of the event
	private: long int endTime;

	/// \brief Objects involved in the event
	private: std::vector<GzEventObj> objects;
};

}

#endif /* GZEVENT_HH_ */
