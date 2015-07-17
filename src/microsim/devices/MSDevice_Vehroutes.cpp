/****************************************************************************/
/// @file    MSDevice_Vehroutes.cpp
/// @author  Daniel Krajzewicz
/// @author  Laura Bieker
/// @author  Michael Behrisch
/// @date    Fri, 30.01.2009
/// @version $Id: MSDevice_Vehroutes.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// A device which collects info on the vehicle trip
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

#include <microsim/MSNet.h>
#include <microsim/MSLane.h>
#include "utils/options/OptionsCont.h"
#include "utils/iodevices/OutputDevice_String.h"
#include "MSDevice_Vehroutes.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS

// ===========================================================================
// static member variables
// ===========================================================================
bool MSDevice_Vehroutes::mySaveExits = false;
bool MSDevice_Vehroutes::myLastRouteOnly = false;
bool MSDevice_Vehroutes::mySorted = false;
bool MSDevice_Vehroutes::myWithTaz = false;
MSDevice_Vehroutes::StateListener MSDevice_Vehroutes::myStateListener;
std::map<const SUMOTime, int> MSDevice_Vehroutes::myDepartureCounts;
std::map<const SUMOTime, std::string> MSDevice_Vehroutes::myRouteInfos;

// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// static initialisation methods
// ---------------------------------------------------------------------------
void MSDevice_Vehroutes::init()
{
  if(OptionsCont::getOptions().isSet("vehroute-output"))
  {
    OutputDevice::createDeviceByOption("vehroute-output", "routes");
        mySaveExits = OptionsCont::getOptions().getBool("vehroute-output.exit-times");
        myLastRouteOnly = OptionsCont::getOptions().getBool("vehroute-output.last-route");
        mySorted = OptionsCont::getOptions().getBool("vehroute-output.sorted");
        myWithTaz = OptionsCont::getOptions().getBool("device.rerouting.with-taz");
        MSNet::getInstance()->addVehicleStateListener(&myStateListener);
    }
}

MSDevice_Vehroutes* MSDevice_Vehroutes::buildVehicleDevices
 (SUMOVehicle& v, std::vector<MSDevice*> &into, unsigned int maxRoutes)
{
  if(OptionsCont::getOptions().getAnyVerbosity()>2)
  {
    printf("MSDevice_Vehroutes* MSDevice_Vehroutes::buildVehicleDevices(...)\n");
    printf("  v.getID().c_str()=%s\n",v.getID().c_str());
    printf("  v.getVehicleType().getID().c_str()=%s\n",
           v.getVehicleType().getID().c_str());
  }
  if (maxRoutes < INT_MAX) {
      return new MSDevice_Vehroutes(v, "vehroute_" + v.getID(), maxRoutes);
  }
  if (OptionsCont::getOptions().isSet("vehroute-output")) {
      if (myLastRouteOnly) {
         maxRoutes = 0;
      }
      myStateListener.myDevices[&v] = new MSDevice_Vehroutes(v, "vehroute_" + v.getID(), maxRoutes);
      into.push_back(myStateListener.myDevices[&v]);
      return myStateListener.myDevices[&v];
  }
  return 0;
}

// ---------------------------------------------------------------------------
// MSDevice_Vehroutes::StateListener-methods
// ---------------------------------------------------------------------------
void
MSDevice_Vehroutes::StateListener::vehicleStateChanged(const SUMOVehicle* const vehicle, MSNet::VehicleState to) {
    if (to == MSNet::VEHICLE_STATE_NEWROUTE) {
        myDevices[vehicle]->addRoute();
    }
}

// ---------------------------------------------------------------------------
// MSDevice_Vehroutes-methods
// ---------------------------------------------------------------------------
MSDevice_Vehroutes::MSDevice_Vehroutes
 (SUMOVehicle& holder,const std::string& id,unsigned int maxRoutes)
 :MSDevice(holder,id),myCurrentRoute(&holder.getRoute()),myMaxRoutes(maxRoutes),
  myLastSavedAt(0),oc(OptionsCont::getOptions())
{
  if(oc.getAnyVerbosity()>1)
    printf("MSDevice_Vehroutes::MSDevice_Vehroutes(...id=%s...)\n",id.c_str());
  myCurrentRoute->addReference();
}


MSDevice_Vehroutes::~MSDevice_Vehroutes() {
  for(std::vector < RouteReplaceInfo > :: iterator i = myReplacedRoutes.begin();
      i != myReplacedRoutes.end(); ++i)
  {
    (*i).route->release();
  }
    myCurrentRoute->release();
    myStateListener.myDevices.erase(&myHolder);
}


bool
MSDevice_Vehroutes::notifyEnter(SUMOVehicle& veh, MSMoveReminder::Notification reason) {
    if (mySorted && reason == NOTIFICATION_DEPARTED && myStateListener.myDevices[&veh] == this) {
        myDepartureCounts[MSNet::getInstance()->getCurrentTimeStep()]++;
    }
    return mySaveExits;
}


bool
MSDevice_Vehroutes::notifyLeave(SUMOVehicle& veh, SUMOReal /*lastPos*/, MSMoveReminder::Notification reason) {
    if (mySaveExits && reason != NOTIFICATION_LANE_CHANGE) {
        if (reason != NOTIFICATION_TELEPORT && myLastSavedAt == veh.getEdge()) { // need to check this for internal lanes
            myExits.back() = MSNet::getInstance()->getCurrentTimeStep();
        } else {
            myExits.push_back(MSNet::getInstance()->getCurrentTimeStep());
            myLastSavedAt = veh.getEdge();
        }
    }
    return mySaveExits;
}


void
MSDevice_Vehroutes::writeXMLRoute(OutputDevice& os, int index) const {
    // check if a previous route shall be written
    os.openTag("route");
    if (index >= 0) {
        assert((int) myReplacedRoutes.size() > index);
        // write edge on which the vehicle was when the route was valid
        os << " replacedOnEdge=\"";
        if (myReplacedRoutes[index].edge) {
            os << myReplacedRoutes[index].edge->getID();
        }
        // write the time at which the route was replaced
        os << "\" replacedAtTime=\"" << time2string(myReplacedRoutes[index].time) << "\" probability=\"0\" edges=\"";
        // get the route
        int i = index;
        while (i > 0 && myReplacedRoutes[i - 1].edge) {
            i--;
        }
        const MSEdge* lastEdge = 0;
        for (; i < index; ++i) {
            myReplacedRoutes[i].route->writeEdgeIDs(os, lastEdge, myReplacedRoutes[i].edge);
            lastEdge = myReplacedRoutes[i].edge;
        }
        myReplacedRoutes[index].route->writeEdgeIDs(os, lastEdge);
    } else {
        os << " edges=\"";
        const MSEdge* lastEdge = 0;
        if (myHolder.getNumberReroutes() > 0) {
            assert(myReplacedRoutes.size() <= myHolder.getNumberReroutes());
            unsigned int i = static_cast<unsigned int>(myReplacedRoutes.size());
            while (i > 0 && myReplacedRoutes[i - 1].edge) {
                i--;
            }
            for (; i < myReplacedRoutes.size(); ++i) {
                myReplacedRoutes[i].route->writeEdgeIDs(os, lastEdge, myReplacedRoutes[i].edge);
                lastEdge = myReplacedRoutes[i].edge;
            }
        }
        myCurrentRoute->writeEdgeIDs(os, lastEdge);
        if (mySaveExits) {
            os << "\" exitTimes=\"";
            for (std::vector<SUMOTime>::const_iterator it = myExits.begin(); it != myExits.end(); ++it) {
                if (it != myExits.begin()) {
                    os << " ";
                }
                os << time2string(*it);
            }
        }
    }
    (os << "\"").closeTag(true);
}


void
MSDevice_Vehroutes::generateOutput() const {
    OutputDevice_String od(1);
    od.openTag("vehicle").writeAttr(SUMO_ATTR_ID, myHolder.getID());
    if (myHolder.getVehicleType().getID() != DEFAULT_VTYPE_ID) {
        od.writeAttr(SUMO_ATTR_TYPE, myHolder.getVehicleType().getID());
    }
    od.writeAttr(SUMO_ATTR_DEPART, time2string(myHolder.getDeparture()));
    od.writeAttr("arrival", time2string(MSNet::getInstance()->getCurrentTimeStep()));
    if (myWithTaz) {
        od.writeAttr(SUMO_ATTR_FROM_TAZ, myHolder.getParameter().fromTaz).writeAttr(SUMO_ATTR_TO_TAZ, myHolder.getParameter().toTaz);
    }
    od << ">\n";
    if (myReplacedRoutes.size() > 0) {
        od.openTag("routeDistribution") << ">\n";
        for (unsigned int i = 0; i < myReplacedRoutes.size(); ++i) {
            writeXMLRoute(od, i);
        }
    }
    writeXMLRoute(od);
    if (myReplacedRoutes.size() > 0) {
        od.closeTag();
    }
    od.closeTag();
    od << "\n";
    if (mySorted) {
        myRouteInfos[myHolder.getDeparture()] += od.getString();
        myDepartureCounts[myHolder.getDeparture()]--;
        std::map<const SUMOTime, int>::iterator it = myDepartureCounts.begin();
        while (it != myDepartureCounts.end() && it->second == 0) {
            OutputDevice::getDeviceByOption("vehroute-output") << myRouteInfos[it->first];
            myRouteInfos.erase(it->first);
            myDepartureCounts.erase(it);
            it = myDepartureCounts.begin();
        }
    } else {
        OutputDevice::getDeviceByOption("vehroute-output") << od.getString();
    }
}


const MSRoute*
MSDevice_Vehroutes::getRoute(int index) const {
    return myReplacedRoutes[index].route;
}


void
MSDevice_Vehroutes::addRoute() {
    if (myMaxRoutes > 0) {
        if (myHolder.getDeparture() >= 0) {
            myReplacedRoutes.push_back(RouteReplaceInfo(myHolder.getEdge(), MSNet::getInstance()->getCurrentTimeStep(), myCurrentRoute));
        } else {
            myReplacedRoutes.push_back(RouteReplaceInfo(0, MSNet::getInstance()->getCurrentTimeStep(), myCurrentRoute));
        }
        if (myReplacedRoutes.size() > myMaxRoutes) {
            myReplacedRoutes.front().route->release();
            myReplacedRoutes.erase(myReplacedRoutes.begin());
        }
    } else {
        myCurrentRoute->release();
    }
    myCurrentRoute = &myHolder.getRoute();
    myCurrentRoute->addReference();
}


/****************************************************************************/

