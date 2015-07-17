/****************************************************************************/
/// @file    MSPersonControl.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @date    Mon, 9 Jul 2001
/// @version $Id: MSPersonControl.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// Stores all persons in the net and handles their waiting for cars.
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.sourceforge.net/
// Copyright (C) 2001-2012 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <vector>
#include <algorithm>
#include "MSNet.h"
#include "MSPerson.h"
#include "MSVehicle.h"
#include "MSPersonControl.h"
#include <utils/iodevices/OutputDevice.h>
#include <utils/options/OptionsCont.h>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
MSPersonControl::MSPersonControl() {}


MSPersonControl::~MSPersonControl() {
    for (std::map<std::string, MSPerson*>::iterator i = myPersons.begin(); i != myPersons.end(); ++i) {
        delete(*i).second;
    }
    myPersons.clear();
    myWaiting.clear();
}


bool
MSPersonControl::add(const std::string& id, MSPerson* person) {
    if (myPersons.find(id) == myPersons.end()) {
        myPersons[id] = person;
        return true;
    }
    return false;
}


void
MSPersonControl::erase(MSPerson* person) {
    const std::string& id = person->getID();
    if (OptionsCont::getOptions().isSet("tripinfo-output")) {
        OutputDevice& od = OutputDevice::getDeviceByOption("tripinfo-output");
        od.openTag("personinfo") << " id=\"" << id << "\" ";
        od << "depart=\"" << time2string(person->getDesiredDepart()) << "\">\n";
        person->tripInfoOutput(od);
        od.closeTag();
    }
    if (OptionsCont::getOptions().isSet("vehroute-output")) {
        OutputDevice& od = OutputDevice::getDeviceByOption("vehroute-output");
        od.openTag("person") << " id=\"" << id
                             << "\" depart=\"" << time2string(person->getDesiredDepart())
                             << "\" arrival=\"" << time2string(MSNet::getInstance()->getCurrentTimeStep())
                             << "\">\n";
        od.closeTag();
        od << "\n";
    }
    const std::map<std::string, MSPerson*>::iterator i = myPersons.find(id);
    if (i != myPersons.end()) {
        delete i->second;
        myPersons.erase(i);
    }
}

void MSPersonControl::setArrival(const SUMOTime time, MSPerson* person)
{
#ifdef INT4SUMOTIME
  const SUMOTime step = time % DELTA_T == 0 ? time : (time / DELTA_T + 1) * DELTA_T;
#endif
#ifdef SUMOREAL4SUMOTIME
  const SUMOTime step = (int)time % (int)DELTA_T == 0 ? time : (time / DELTA_T + 1) * DELTA_T;
#endif
  if(myArrivals.find(step) == myArrivals.end())
  {
    myArrivals[step] = PersonVector();
  }
  myArrivals[step].push_back(person);
}

void MSPersonControl::checkArrivedPersons(MSNet* net, const SUMOTime time)
{
  while (myArrivals.find(time) != myArrivals.end())
  {
    const PersonVector& persons = myArrivals[time];
    // we cannot use an iterator here because there might be additions to the vector while proceeding
    for(size_t i = 0; i < persons.size(); ++i)
    {
      persons[i]->proceed(net, time);
    }
    myArrivals.erase(time);
  }
}

void
MSPersonControl::addWaiting(const MSEdge* const edge, MSPerson* person)
{
  if(myWaiting.find(edge) == myWaiting.end())
  {
    myWaiting[edge] = std::vector<MSPerson*>();
  }
  myWaiting[edge].push_back(person);
}

bool MSPersonControl::boardAnyWaiting(const MSEdge* const edge, MSVehicle* vehicle)
{
    bool ret = false;
    if (myWaiting.find(edge) != myWaiting.end()) {
        PersonVector& waitPersons = myWaiting[edge];
        for (PersonVector::iterator i = waitPersons.begin(); i != waitPersons.end();) {
            const std::string& line = vehicle->getParameter().line == "" ? vehicle->getParameter().id : vehicle->getParameter().line;
            if ((*i)->isWaitingFor(line)) {
                vehicle->addPerson(*i);
                i = waitPersons.erase(i);
                ret = true;
            } else {
                ++i;
            }
        }
    }
    return ret;
}


bool
MSPersonControl::hasPersons() const {
    return !myPersons.empty();
}


bool
MSPersonControl::hasPedestrians() const {
    return !myArrivals.empty();
}


void
MSPersonControl::abortWaiting() {
    while (!myPersons.empty()) {
        std::map<std::string, MSPerson*>::iterator i = myPersons.begin();
        WRITE_WARNING("Person " + i->first + " aborted waiting for a ride that will never come.");
        erase(i->second);
    }
}

/****************************************************************************/
