/****************************************************************************/
/// @file    MSMoveReminder.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    2008-10-27
/// @version $Id: MSMoveReminder.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// Something on a lane to be noticed about vehicle movement
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

#include <string>
#include <cassert>
#include "MSLane.h"
#include "MSMoveReminder.h"


// ===========================================================================
// method definitions
// ===========================================================================
MSMoveReminder::MSMoveReminder(MSLane* const lane, const bool doAdd)
    : myLane(lane) {
    if (myLane != 0 && doAdd) {
        // add reminder to lane
        myLane->addMoveReminder(this);
    }
}


#ifdef HAVE_MESOSIM
void
MSMoveReminder::updateDetector(SUMOVehicle& veh, SUMOReal entryPos, SUMOReal leavePos,
                               SUMOTime entryTime, SUMOTime currentTime, SUMOTime leaveTime) {
    std::map<SUMOVehicle*, std::pair<SUMOTime, SUMOReal> >::iterator j = myLastVehicleUpdateValues.find(&veh);
    if (j != myLastVehicleUpdateValues.end()) {
        // the vehicle already has reported its values before; use these
        entryTime = (*j).second.first;
        entryPos = (*j).second.second;
        myLastVehicleUpdateValues.erase(j);
    }
    const SUMOReal timeOnLane = STEPS2TIME(currentTime - entryTime);
    const SUMOReal speed = (leavePos - entryPos) / STEPS2TIME(leaveTime - entryTime);
    myLastVehicleUpdateValues[&veh] = std::pair<SUMOTime, SUMOReal>(currentTime, entryPos + speed * timeOnLane);
    notifyMoveInternal(veh, timeOnLane, speed);
}
#endif
/****************************************************************************/

