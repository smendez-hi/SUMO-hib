/****************************************************************************/
/// @file    RORouteDef_Complete.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id: RORouteDef_Complete.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// A complete route definition (with all passed edges being known)
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
#include <deque>
#include <iterator>
#include "ROEdge.h"
#include "RORouteDef.h"
#include "RORoute.h"
#include <utils/common/SUMOAbstractRouter.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/UtilExceptions.h>
#include "RORouteDef_Complete.h"
#include "ROHelper.h"
#include <utils/iodevices/OutputDevice.h>
#include <utils/options/OptionsCont.h>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
RORouteDef_Complete::RORouteDef_Complete(const std::string& id,
        const RGBColor* const color,
        const std::vector<const ROEdge*> &edges,
        bool tryRepair)
    : RORouteDef(id, color), myEdges(edges), myTryRepair(tryRepair) {
}

std::string toString(std::vector<const ROEdge*> edges)
{
  std::string result_string = "";
  for(int i = 0; i < edges.size(); i++)
  {
    result_string += edges.at(i)->getID();
  }
}

RORouteDef_Complete::~RORouteDef_Complete() {}

RORoute* RORouteDef_Complete::buildCurrentRoute
 (SUMOAbstractRouter<ROEdge, ROVehicle> &router, SUMOTime begin,
  const ROVehicle& veh) const
{
    if (myTryRepair) {
        const std::vector<const ROEdge*> &oldEdges = myEdges;
        if (oldEdges.size() == 0) {
            MsgHandler* m = (
             OptionsCont::getOptions().getBool("ignore-errors") ?
             MsgHandler::getWarningInstance() :
             MsgHandler::getErrorInstance());
            m->inform("Could not repair empty route of vehicle '" +
             veh.getID() + "'.");
            return new RORoute(myID, 0, 1, std::vector<const ROEdge*>(),
             copyColorIfGiven());
        }
        std::vector<const ROEdge*> newEdges;
        newEdges.push_back(*(oldEdges.begin()));
        for(std::vector<const ROEdge*>::const_iterator i = oldEdges.begin() + 1;
         i != oldEdges.end();
         ++i)
        {
            if ((*(i - 1))->isConnectedTo(*i)) {
                newEdges.push_back(*i);
            } else {
                std::vector<const ROEdge*> edges;
                router.compute(*(i - 1), *i, &veh, begin, edges);
                if (edges.size() == 0) {
                    return 0;
                }
                std::copy(edges.begin() + 1, edges.end(),
                 back_inserter(newEdges));
            }
        }
        if(myEdges != newEdges)
        {
          WRITE_WARNING("Repaired route of vehicle '" + veh.getID() + "'.");
          if(OptionsCont::getOptions().getLoadVerbosity() ||
           OptionsCont::getOptions().getBuildVerbosity())
          {
            WRITE_WARNING(" Old edges: " + toString(myEdges));
            WRITE_WARNING(" New edges: " + toString(newEdges));
          }
        }
        myEdges = newEdges;
    }
    SUMOReal costs = router.recomputeCosts(myEdges, &veh, begin);
    if(costs < 0)
        throw ProcessError("Route '" + getID() + "' (vehicle '" +
         veh.getID() + "') is not valid.");
    return new RORoute(myID, 0, 1, myEdges, copyColorIfGiven());
}

void RORouteDef_Complete::addAlternative
 (SUMOAbstractRouter<ROEdge, ROVehicle> &, const ROVehicle* const,
  RORoute* current, SUMOTime begin)
{
    myStartTime = begin;
    myEdges = current->getEdgeVector();
    delete current;
}

RORouteDef* RORouteDef_Complete::copy(const std::string& id) const
{
    return new RORouteDef_Complete(id, copyColorIfGiven(), myEdges,
     myTryRepair);
}

OutputDevice& RORouteDef_Complete::writeXMLDefinition
 (SUMOAbstractRouter<ROEdge, ROVehicle> &router, OutputDevice& dev,
  const ROVehicle* const veh, bool asAlternatives, bool withExitTimes) const
{
    // (optional) alternatives header
    if (asAlternatives) {
        dev.openTag("routeDistribution") << " last=\"0\">\n";
    }
    // the route
    dev.openTag("route");
    if(asAlternatives)
    {
        dev << " cost=\"" << router.recomputeCosts(myEdges, veh,
         veh->getDepartureTime());
        dev << "\" probability=\"1.00\"";
    }
    if (myColor != 0) {
        dev << " color=\"" << *myColor << "\"";
    }
    dev << " edges=\"" << myEdges;
    if (withExitTimes) {
        SUMOReal time = STEPS2TIME(veh->getDepartureTime());
        dev << "\" exitTimes=\"";
        std::vector<const ROEdge*>::const_iterator i = myEdges.begin();
        for (; i != myEdges.end(); ++i) {
            if (i != myEdges.begin()) {
                dev << " ";
            }
            time += (*i)->getTravelTime(veh, (SUMOTime) time);
            dev << time;
        }
    }
    (dev << "\"").closeTag(true);
    // (optional) alternatives end
    if (asAlternatives) {
        dev.closeTag();
    }
    return dev;
}
