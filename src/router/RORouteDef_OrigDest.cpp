/****************************************************************************/
/// @file    RORouteDef_OrigDest.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id: RORouteDef_OrigDest.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// A route where only the origin and the destination edges are known
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
#include <iostream>
#include <cassert>
#include "ROEdge.h"
#include "RORouteDef.h"
#include "RORoute.h"
#include "RORouteDef_OrigDest.h"
#include <utils/common/SUMOAbstractRouter.h>
#include "ROVehicle.h"
#include "ROHelper.h"
#include <utils/common/MsgHandler.h>
#include <utils/iodevices/OutputDevice.h>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// member method definitions
// ===========================================================================
RORouteDef_OrigDest::RORouteDef_OrigDest(const std::string& id,
        const RGBColor* const color,
        const ROEdge* from,
        const ROEdge* to,
        bool removeFirst)
    : RORouteDef(id, color), myFrom(from), myTo(to), myCurrent(0),
      myRemoveFirst(removeFirst) {}


RORouteDef_OrigDest::~RORouteDef_OrigDest() {
    delete myCurrent;
}


RORoute*
RORouteDef_OrigDest::buildCurrentRoute(SUMOAbstractRouter<ROEdge, ROVehicle> &router,
                                       SUMOTime begin, const ROVehicle& veh) const {
    std::vector<const ROEdge*> edges;
    router.compute(myFrom, myTo, &veh, begin, edges);
    if (myRemoveFirst && edges.size() > 2) {
        edges.erase(edges.begin());
        edges.erase(edges.end() - 1);
    }
    return new RORoute(myID, 0, 1, edges, copyColorIfGiven());
}


void
RORouteDef_OrigDest::addAlternative(SUMOAbstractRouter<ROEdge, ROVehicle> &router,
                                    const ROVehicle* const veh, RORoute* current, SUMOTime begin) {
    myCurrent = current;
    myStartTime = begin;
    current->setCosts(router.recomputeCosts(current->getEdgeVector(), veh, begin));
}


RORouteDef*
RORouteDef_OrigDest::copy(const std::string& id) const {
    return new RORouteDef_OrigDest(id, copyColorIfGiven(), myFrom, myTo,
                                   myRemoveFirst);
}


OutputDevice&
RORouteDef_OrigDest::writeXMLDefinition(SUMOAbstractRouter<ROEdge, ROVehicle> &router,
                                        OutputDevice& dev, const ROVehicle* const veh, bool asAlternatives, bool withExitTimes) const {
    return myCurrent->writeXMLDefinition(router, dev, veh, asAlternatives, withExitTimes);
}


/****************************************************************************/

