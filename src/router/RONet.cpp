/****************************************************************************/
/// @file    RONet.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id: RONet.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// The router's network representation
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
#include <fstream>
#include <deque>
#include <queue>
#include "ROEdge.h"
#include "RONode.h"
#include "RONet.h"
#include "RORoute.h"
#include "RORouteDef.h"
#include "ROVehicle.h"
#include <utils/common/SUMOVTypeParameter.h>
#include <utils/common/SUMOAbstractRouter.h>
#include <utils/options/OptionsCont.h>
#include <utils/common/UtilExceptions.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/ToString.h>
#include <utils/common/RandHelper.h>
#include <utils/common/SUMOVehicleClass.h>
#include <utils/iodevices/OutputDevice.h>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
RONet::RONet
 ()
 :myVehicleTypes(),myRoutesOutput(0),myRouteAlternativesOutput(0),
  myReadRouteNo(0),myDiscardedRouteNo(0),myWrittenRouteNo(0),
  myHaveRestrictions(false),oc(OptionsCont::getOptions())
{}

RONet::~RONet(){
  myNodes.clear();
  myEdges.clear();
  myVehicleTypes.clear();
  myRoutes.clear();
  myVehicles.clear();
}

bool RONet::addEdge(ROEdge* edge) {
  if (!myEdges.add(edge->getID(), edge)) {
    WRITE_ERROR("The edge '" + edge->getID() + "' occurs at least twice.");
    delete edge;
    return false;
  }
  return true;
}

void RONet::addNode(RONode* node) {
  if (!myNodes.add(node->getID(), node)) {
    WRITE_ERROR("The node '" + node->getID() + "' occurs at least twice.");
    delete node;
  }
}

bool RONet::addRouteDef(RORouteDef* def)
{
  if(oc.getLoadVerbosity())
    std::cout<<"----> bool RONet::addRouteDef(RORouteDef* def)"<<std::endl;
    return myRoutes.add(def->getID(), def);
}

void RONet::openOutput(const std::string& filename, bool useAlternatives)
{
  if(oc.getLoadVerbosity())
    std::cout<<"----> void RONet::openOutput(const std::string& filename, bool useAlternatives)"<<std::endl;
    myRoutesOutput = &OutputDevice::getDevice(filename);
    myRoutesOutput->writeXMLHeader("routes", "", "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:noNamespaceSchemaLocation=\"http://sumo.sf.net/xsd/routes_file.xsd\"");
    if (useAlternatives) {
        size_t len = filename.length();
        if (len > 4 && filename.substr(len - 4) == ".xml") {
            myRouteAlternativesOutput = &OutputDevice::getDevice(filename.substr(0, len - 4) + ".alt.xml");
        } else {
            myRouteAlternativesOutput = &OutputDevice::getDevice(filename + ".alt");
        }
        myRouteAlternativesOutput->writeXMLHeader("route-alternatives", "", "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:noNamespaceSchemaLocation=\"http://sumo.sf.net/xsd/routes_file.xsd\"");
    }
}

void RONet::closeOutput() {
    // end writing
    if (myRoutesOutput != 0) {
        myRoutesOutput->close();
    }
    // only if opened
    if (myRouteAlternativesOutput != 0) {
        myRouteAlternativesOutput->close();
    }
}

SUMOVTypeParameter* RONet::getVehicleTypeSecure
 (const std::string& id)
{
  // check whether the type was already known
  SUMOVTypeParameter* type = myVehicleTypes.get(id);
  if(type != 0) {
    return type;
  }
  if(id == ""){
        // ok, no vehicle type was given within the user input
        //  return the default type
        return 0;
  }
  // Assume, the user will define the type somewhere else
  //  return a type which contains the id only
  
  if(oc.getAnyVerbosity())
    std::cout<<"SUMOVTypeParameter* RONet::getVehicleTypeSecure(const std::stri"
     <<"ng& id)\n (will now execute... type=new SUMOVTypeParameter(DEFAULT_VTYP"
     <<"E_ID);"<<std::endl;
  
  //TASK Code not verified
  type=new SUMOVTypeParameter(DEFAULT_VTYPE_ID);
  // Original code was:
  //CODE//type = new SUMOVTypeParameter();
  
  type->id = id;
  type->onlyReferenced = true;
  addVehicleType(type);
  return type;
}

bool RONet::addVehicleType(SUMOVTypeParameter* type) {
    if (!myVehicleTypes.add(type->id, type)) {
        WRITE_ERROR("The vehicle type '" + type->id + "' occurs at least twice.");
        delete type;
        return false;
    }
    return true;
}

bool RONet::addVehicle(const std::string& id, ROVehicle* veh) {
    if (myVehIDs.find(id) == myVehIDs.end() && myVehicles.add(id, veh)) {
        myVehIDs.insert(id);
        myReadRouteNo++;
        return true;
    }
    WRITE_ERROR("The vehicle '" + id + "' occurs at least twice.");
    return false;
}

bool RONet::computeRoute(OptionsCont& options, SUMOAbstractRouter<ROEdge, ROVehicle> &router,
                    const ROVehicle* const veh) {
    MsgHandler* mh = MsgHandler::getErrorInstance();
    std::string noRouteMsg = "The vehicle '" + veh->getID() + "' has no valid route.";
    if (options.getBool("ignore-errors")) {
        mh = MsgHandler::getWarningInstance();
    }
    RORouteDef* const routeDef = veh->getRouteDefinition();
    // check if the route definition is valid
    if (routeDef == 0) {
        mh->inform(noRouteMsg);
        return false;
    }
    // check whether the route was already saved
    if (routeDef->isSaved()) {
        return true;
    }
    //
    RORoute* current = routeDef->buildCurrentRoute(router, veh->getDepartureTime(), *veh);
    if (current == 0 || current->size() == 0) {
        delete current;
        mh->inform(noRouteMsg);
        return false;
    }
    // check whether we have to evaluate the route for not containing loops
    if (options.getBool("remove-loops")) {
        current->recheckForLoops();
    }
    // check whether the route is still valid
    if (current->size() == 0) {
        delete current;
        mh->inform(noRouteMsg + " (after removing loops)");
        return false;
    }
    // add built route
    routeDef->addAlternative(router, veh, current, veh->getDepartureTime());
    return true;
}

SUMOTime RONet::saveAndRemoveRoutesUntil
 (OptionsCont& options, SUMOAbstractRouter<ROEdge, ROVehicle> &router,
  SUMOTime time)
{
  OptionsCont &oc = OptionsCont::getOptions();
  if(oc.getLoadVerbosity())
    std::cout<<"----> SUMOTime RONet::saveAndRemoveRoutesUntil(OptionsCont& opt"
      <<"ptions, SUMOAbstractRouter<ROEdge, ROVehicle> &router,SUMOTime time)"
      <<std::endl;
    SUMOTime lastTime = -1;
    // write all vehicles (and additional structures)
    while (myVehicles.size() != 0) {
        // get the next vehicle
        const ROVehicle* const veh = myVehicles.getTopVehicle();
        SUMOTime currentTime = veh->getDepartureTime();
        // check whether it shall not yet be computed
        if (currentTime > time) {
            lastTime = currentTime;
            break;
        }
        // check whether to print the output
        if (lastTime != currentTime && lastTime != -1) {
            // report writing progress
#ifdef INT4SUMOTIME
            if (options.getInt("stats-period") >= 0 && (currentTime % options.getInt("stats-period")) == 0) {
#endif
#ifdef SUMOREAL4SUMOTIME
            if (options.getInt("stats-period") >= 0 && ((int) currentTime % options.getInt("stats-period")) == 0) {
#endif              
                WRITE_MESSAGE("Read: " + toString(myReadRouteNo) + ",  Discarded: " + toString(myDiscardedRouteNo) + ",  Written: " + toString(myWrittenRouteNo));
            }
        }
        lastTime = currentTime;

        // ok, compute the route (try it)
        if (computeRoute(options, router, veh)) {
            // write the route
            veh->saveAllAsXML(router, *myRoutesOutput, myRouteAlternativesOutput, options.getBool("exit-times"));
            myWrittenRouteNo++;
            // remove the route if it is not longer used
            /*
            if (!myRoutes.erase(route->getID())) {
                WRITE_WARNING("Could not remove " + route->getID());
            }
            */
        } else {
            myDiscardedRouteNo++;
        }
        // and the vehicle
        myVehicles.erase(veh->getID());
    }
    return lastTime;
}

bool RONet::furtherStored() {
    return myVehicles.size() > 0;
}

ROEdge* RONet::getRandomSource() {
    // check whether an edge may be returned
    checkSourceAndDestinations();
    if (mySourceEdges.size() == 0) {
        return 0;
    }
    // choose a random edge
    return RandHelper::getRandomFrom(mySourceEdges);
}

const ROEdge* RONet::getRandomSource() const {
    // check whether an edge may be returned
    checkSourceAndDestinations();
    if (mySourceEdges.size() == 0) {
        return 0;
    }
    // choose a random edge
    return RandHelper::getRandomFrom(mySourceEdges);
}

ROEdge* RONet::getRandomDestination() {
    // check whether an edge may be returned
    checkSourceAndDestinations();
    if (myDestinationEdges.size() == 0) {
        return 0;
    }
    // choose a random edge
    return RandHelper::getRandomFrom(myDestinationEdges);
}

const ROEdge* RONet::getRandomDestination() const {
    // check whether an edge may be returned
    checkSourceAndDestinations();
    if (myDestinationEdges.size() == 0) {
        return 0;
    }
    // choose a random edge
    return RandHelper::getRandomFrom(myDestinationEdges);
}

void RONet::checkSourceAndDestinations() const {
    if (myDestinationEdges.size() != 0 || mySourceEdges.size() != 0) {
        return;
    }
    const std::map<std::string, ROEdge*> &edges = myEdges.getMyMap();
    for (std::map<std::string, ROEdge*>::const_iterator i = edges.begin(); i != edges.end(); ++i) {
        ROEdge* e = (*i).second;
        ROEdge::EdgeType type = e->getType();
        // !!! add something like "classified edges only" for using only sources or sinks
        if (type != ROEdge::ET_SOURCE) {
            myDestinationEdges.push_back(e);
        }
        if (type != ROEdge::ET_SINK) {
            mySourceEdges.push_back(e);
        }
    }
}

unsigned int RONet::getEdgeNo() const {
    return (unsigned int) myEdges.size();
}

const std::map<std::string, ROEdge*>& RONet::getEdgeMap() const {
    return myEdges.getMyMap();
}

bool RONet::hasRestrictions() const {
    return myHaveRestrictions;
}

void RONet::setRestrictionFound() {
    myHaveRestrictions = true;
}
