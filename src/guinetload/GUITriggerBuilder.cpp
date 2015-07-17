/****************************************************************************/
/// @file    GUITriggerBuilder.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Mon, 26.04.2004
/// @version $Id: GUITriggerBuilder.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// Builds trigger objects for guisim
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
#include <fstream>
#include <guisim/GUILaneSpeedTrigger.h>
#include <guisim/GUINet.h>
#include <guisim/GUITriggeredRerouter.h>
#include <guisim/GUIBusStop.h>
#include "GUITriggerBuilder.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
GUITriggerBuilder::GUITriggerBuilder() {}


GUITriggerBuilder::~GUITriggerBuilder() {}


MSLaneSpeedTrigger*
GUITriggerBuilder::buildLaneSpeedTrigger(MSNet& net,
        const std::string& id, const std::vector<MSLane*> &destLanes,
        const std::string& file) {
    GUILaneSpeedTrigger* lst = new GUILaneSpeedTrigger(id, destLanes, file);
    static_cast<GUINet&>(net).getVisualisationSpeedUp().addAdditionalGLObject(lst);
    return lst;
}


void
GUITriggerBuilder::buildRerouter(MSNet& net, const std::string& id,
                                 std::vector<MSEdge*> &edges,
                                 SUMOReal prob, const std::string& file, bool off) {
    static_cast<GUINet&>(net).getVisualisationSpeedUp().addAdditionalGLObject(new GUITriggeredRerouter(id, edges, prob, file, off));
}


void
GUITriggerBuilder::buildBusStop(MSNet& net, const std::string& id,
                                const std::vector<std::string> &lines,
                                MSLane* lane,
                                SUMOReal frompos, SUMOReal topos) throw(InvalidArgument) {
    GUIBusStop* stop = new GUIBusStop(id, lines, *lane, frompos, topos);
    if (!net.addBusStop(stop)) {
        delete stop;
        throw InvalidArgument("Could not build bus stop '" + id + "'; probably declared twice.");
    }
    static_cast<GUINet&>(net).getVisualisationSpeedUp().addAdditionalGLObject(stop);
}


/****************************************************************************/

