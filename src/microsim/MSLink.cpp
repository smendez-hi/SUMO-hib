/****************************************************************************/
/// @file    MSLink.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @author  Laura Bieker
/// @date    Sept 2002
/// @version $Id: MSLink.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// A connnection between lanes
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
#include "MSLink.h"
#include "MSLane.h"
#include <iostream>
#include <cassert>
#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS

// ===========================================================================
// static member variables
// ===========================================================================
SUMOTime MSLink::myLookaheadTime = TIME2STEPS(3);

// ===========================================================================
// member method definitions
// ===========================================================================
#ifndef HAVE_INTERNAL_LANES
MSLink::MSLink
 (MSLane* succLane,LinkDirection dir,LinkState state,SUMOReal length)
 :myLane(succLane),myRequestIdx(0),myRespondIdx(0),myState(state),
  myDirection(dir),myLength(length),myAmCont(false)
#else
MSLink::MSLink
 (MSLane* succLane,MSLane* via,LinkDirection dir,LinkState state,
  SUMOReal length)
 :myLane(succLane),myRequestIdx(0),myRespondIdx(0),myState(state),
  myDirection(dir),myLength(length),myJunctionInlane(via),myAmCont(false)
#endif
{}

MSLink::~MSLink() {}

void MSLink::setRequestInformation
 (unsigned int requestIdx,unsigned int respondIdx,bool isCrossing,bool isCont,
  const std::vector<MSLink*> &foeLinks,const std::vector<MSLane*> &foeLanes)
{
    myRequestIdx = requestIdx;
    myRespondIdx = respondIdx;
    myIsCrossing = isCrossing;
    myAmCont = isCont;
    myFoeLinks = foeLinks;
    myFoeLanes = foeLanes;
}

void MSLink::setApproaching
 (SUMOVehicle* approaching,SUMOTime arrivalTime,SUMOReal speed,
  bool setRequest)
{
    LinkApproachingVehicles::iterator i = find_if(myApproachingVehicles.begin(), myApproachingVehicles.end(), vehicle_in_request_finder(approaching));
    if (i != myApproachingVehicles.end()) {
        myApproachingVehicles.erase(i);
    }
    const SUMOTime leaveTime = arrivalTime + TIME2STEPS((approaching->getVehicleType().getLengthWithGap() + getLength()) / speed);
    ApproachingVehicleInformation approachInfo(arrivalTime, leaveTime, approaching, setRequest);
    myApproachingVehicles.push_back(approachInfo);
}

void MSLink::addBlockedLink(MSLink* link) {
    myBlockedFoeLinks.insert(link);
}

bool MSLink::willHaveBlockedFoe() const {
    for (std::set<MSLink*>::const_iterator i = myBlockedFoeLinks.begin(); i != myBlockedFoeLinks.end(); ++i) {
        if ((*i)->isBlockingAnyone()) {
            return true;
        }
    }
    return false;
}

void MSLink::removeApproaching(SUMOVehicle* veh) {
    LinkApproachingVehicles::iterator i = find_if(myApproachingVehicles.begin(), myApproachingVehicles.end(), vehicle_in_request_finder(veh));
    if (i != myApproachingVehicles.end()) {
        myApproachingVehicles.erase(i);
    }
}

bool MSLink::opened(SUMOTime arrivalTime, SUMOReal arrivalSpeed, SUMOReal vehicleLength) const {
    if (myState == LINKSTATE_TL_RED) {
        return false;
    }
    if (myAmCont) {
        return true;
    }
#ifdef HAVE_INTERNAL_LANES
    const SUMOReal length = myJunctionInlane == 0 ? getLength() : myJunctionInlane->getLength();
#else
    const SUMOReal length = getLength();
#endif
    const SUMOTime leaveTime = arrivalTime + TIME2STEPS((length + vehicleLength) / arrivalSpeed);
    for (std::vector<MSLink*>::const_iterator i = myFoeLinks.begin(); i != myFoeLinks.end(); ++i) {
#ifdef HAVE_MESOSIM
        if (MSGlobals::gUseMesoSim) {
            if ((*i)->getState() == LINKSTATE_TL_RED) {
                continue;
            }
        }
#endif
        if ((*i)->blockedAtTime(arrivalTime, leaveTime)) {
            return false;
        }
    }
    for (std::vector<MSLane*>::const_iterator i = myFoeLanes.begin(); i != myFoeLanes.end(); ++i) {
        if ((*i)->getVehicleNumber() > 0 || (*i)->getPartialOccupator() != 0) {
            return false;
        }
    }
    return true;
}


bool
MSLink::blockedAtTime(SUMOTime arrivalTime, SUMOTime leaveTime) const {
    for (LinkApproachingVehicles::const_iterator i = myApproachingVehicles.begin(); i != myApproachingVehicles.end(); ++i) {
        if (!(*i).willPass) {
            continue;
        }
        if (!(((*i).leavingTime + myLookaheadTime < arrivalTime) || ((*i).arrivalTime - myLookaheadTime > leaveTime))) {
            return true;
        }
    }
    return false;
}


bool
MSLink::hasApproachingFoe(SUMOTime arrivalTime, SUMOTime leaveTime) const {
    for (std::vector<MSLink*>::const_iterator i = myFoeLinks.begin(); i != myFoeLinks.end(); ++i) {
        if ((*i)->blockedAtTime(arrivalTime, leaveTime)) {
            return true;
        }
    }
    for (std::vector<MSLane*>::const_iterator i = myFoeLanes.begin(); i != myFoeLanes.end(); ++i) {
        if ((*i)->getVehicleNumber() > 0 || (*i)->getPartialOccupator() != 0) {
            return true;
        }
    }
    return false;
}


LinkDirection
MSLink::getDirection() const {
    return myDirection;
}


void
MSLink::setTLState(LinkState state, SUMOTime /*t*/) {
    myState = state;
}


MSLane*
MSLink::getLane() const {
    return myLane;
}


#ifdef HAVE_INTERNAL_LANES
MSLane*
MSLink::getViaLane() const {
    return myJunctionInlane;
}
#endif


unsigned int
MSLink::getRespondIndex() const {
    return myRespondIdx;
}



/****************************************************************************/

