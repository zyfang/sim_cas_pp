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
//TODO rename PpEvent.hh ?
#include "GzEvent.hh"

using namespace sg_pp;

//////////////////////////////////////////////////
PpEvent::PpEvent(const std::string _name,
        const std::string _class_namespace,
        const std::string _class_type,
        double _start_time) :
		name(_name),
		classNamespace(_class_namespace),
		classType(_class_type)
{
    // add id to the event
    // TODO this should be static?!
	id++;

	// set the start time;
	this->Start(_start_time);
}

//////////////////////////////////////////////////
PpEvent::PpEvent(const std::string _name,
		const std::string _class_namespace,
		const std::string _class_type,
		const std::string _property_namespace,
		const std::string _property,
		double _start_time) :
		name(_name),
		classNamespace(_class_namespace),
		classType(_class_type),
		propertyNamespace(_property_namespace),
		property(_property)
{
	// add id to the event;
	id++;

	// set the start time;
	this->Start(_start_time);
}

//////////////////////////////////////////////////
PpEvent::~PpEvent()
{
}

//////////////////////////////////////////////////
int PpEvent::GetId()
{
	return id;
}

//////////////////////////////////////////////////
const std::string PpEvent::GetName()
{
	return this->name;
}

//////////////////////////////////////////////////
const std::string PpEvent::GetClassNamespace()
{
	return this->classNamespace;
}


//////////////////////////////////////////////////
const std::string PpEvent::GetClass()
{
	return this->classType;
}

//////////////////////////////////////////////////
const std::string PpEvent::GetPropertyNamespace()
{
	return this->propertyNamespace;
}

//////////////////////////////////////////////////
const std::string PpEvent::GetProperty()
{
	return this->property;
}

//////////////////////////////////////////////////
bool PpEvent::IsOpen()
{
	return this->isOpen;
}

//////////////////////////////////////////////////
double PpEvent::GetStartTime()
{
	return this->startTime;
}

//////////////////////////////////////////////////
void PpEvent::SetStartTime(double _start_time)
{
	this->startTime = _start_time;
}

//////////////////////////////////////////////////
double PpEvent::GetEndTime()
{
	return this->endTime;
}

//////////////////////////////////////////////////
void PpEvent::SetEndTime(double _end_time)
{
	this->endTime = _end_time;
}

//////////////////////////////////////////////////
double PpEvent::GetDuration()
{
	if(!this->isOpen)
	{
		return this->endTime - this->startTime;
	}
	else
	{
		std::cout << "Event not closed, duration unknown.." << std::endl;
		return -1;
	}
}

//////////////////////////////////////////////////
void PpEvent::Start(double _start_time)
{
	// close the event
	this->isOpen = true;

	// set the time when the event started
	this->SetStartTime(_start_time);
}

//////////////////////////////////////////////////
void PpEvent::End(double _end_time)
{
	// close the event
	this->isOpen = false;

	// set the time when the event closed
	this->SetEndTime(_end_time);
}

//////////////////////////////////////////////////
void PpEvent::AddObject(PpEventObj* _ev_obj)
{
	this->objects.push_back(_ev_obj);
}

//////////////////////////////////////////////////
std::vector<PpEventObj*> PpEvent::GetObjects()
{
	return this->objects;
}
