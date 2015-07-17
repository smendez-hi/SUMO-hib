/****************************************************************************/
/// @file    MSBusStop.cpp
/// @author  Daniel Krajzewicz
/// @date    Mon, 13.12.2005
/// @version $Id: MSBusStop.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// A lane area vehicles can halt at
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

#include <cassert>
#include "MSTrigger.h"
#include "MSBusStop.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
MSBusStop::MSBusStop(const std::string& id,
                     const std::vector<std::string> &lines,
                     MSLane& lane,
                     SUMOReal begPos, SUMOReal endPos)
    : MSTrigger(id), myLines(lines), myLane(lane),
      myBegPos(begPos), myEndPos(endPos), myLastFreePos(endPos) {
    computeLastFreePos();
}


MSBusStop::~MSBusStop() {}


const MSLane&
MSBusStop::getLane() const {
    return myLane;
}


SUMOReal
MSBusStop::getBeginLanePosition() const {
    return myBegPos;
}


SUMOReal
MSBusStop::getEndLanePosition() const {
    return myEndPos;
}


void
MSBusStop::enter(void* what, SUMOReal beg, SUMOReal end) {
    myEndPositions[what] = std::pair<SUMOReal, SUMOReal>(beg, end);
    computeLastFreePos();
}


SUMOReal
MSBusStop::getLastFreePos() const {
    return myLastFreePos;
}


void
MSBusStop::leaveFrom(void* what) {
    assert(myEndPositions.find(what) != myEndPositions.end());
    myEndPositions.erase(myEndPositions.find(what));
    computeLastFreePos();
}


void
MSBusStop::computeLastFreePos() {
    myLastFreePos = myEndPos;
    std::map<void*, std::pair<SUMOReal, SUMOReal> >::iterator i;
    for (i = myEndPositions.begin(); i != myEndPositions.end(); i++) {
        if (myLastFreePos > (*i).second.second) {
            myLastFreePos = (*i).second.second;
        }
    }
}



/****************************************************************************/

