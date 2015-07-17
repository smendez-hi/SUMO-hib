/****************************************************************************/
/// @file    MSBaseVehicle.cpp
/// @author  Michael Behrisch
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @date    Mon, 8 Nov 2010
/// @version $Id: MSBaseVehicle.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// A base class for vehicle implementations
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

#include <iostream>
#include <cassert>
#include <utils/common/StdDefs.h>
#include <utils/common/MsgHandler.h>
#include <utils/options/OptionsCont.h>
#include "MSVehicleType.h"
#include "MSEdge.h"
#include "MSLane.h"
#include "MSMoveReminder.h"
#include <microsim/devices/MSDevice_Vehroutes.h>
#include <microsim/devices/MSDevice_Tripinfo.h>
#include <microsim/devices/MSDevice_Routing.h>
#include <microsim/devices/MSDevice_Person.h>
#include <microsim/devices/MSDevice_HBEFA.h>
#include <microsim/devices/MSDevice_FEV.h>
#include "MSBaseVehicle.h"
#include "scenload/VehicleToTrack.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS

// ===========================================================================
// method definitions
// ===========================================================================
MSBaseVehicle::MSBaseVehicle
 (SUMOVehicleParameter* pars,const MSRoute* route,const MSVehicleType* type/*,
  bool ecoGemRouter*/)
 :myParameter(pars),myRoute(route),myType(type),myCurrEdge(route->begin()),
  myIndividualMaxSpeed(0.0),myHasIndividualMaxSpeed(false),
  myReferenceSpeed(-1.0),myMoveReminders(0),myDeparture(-1),myArrivalPos(-1),
  myNumberReroutes(0),oc(OptionsCont::getOptions())
{
  if(OptionsCont::getOptions().getInt("simulationVerbosity")>1)
  {
    std::cout<<"MSBaseVehicle::MSBaseVehicle(...)"<<std::endl;
  }  
  // init devices
  MSDevice_Vehroutes::buildVehicleDevices(*this, myDevices);
  MSDevice_Tripinfo::buildVehicleDevices(*this, myDevices);
  MSDevice_Routing::buildVehicleDevices(*this, myDevices/*, ecoGemRouter*/);
  MSDevice_HBEFA::buildVehicleDevices(*this, myDevices);
  // Only this fev thingy if is this a FEV
  //if(type->getVehicleClass()->getVehicleClassID() == DEFAULT_ELECVTYPE_ID)
  if(//type->getID()==ECOMILE_ELECVTYPE_ID||
    type->getID()=="ecomile"||
    //type->getID()==NIDO_ELECVTYPE_ID||
    type->getID()=="nido"||
    //type->getID()==MIEV_ELECVTYPE_ID||
    type->getID()=="miev"||
    //type->getID()==TEMSA_ELECVTYPE_ID||
    type->getID()=="temsa"||
    //Nuevos coches
    type->getID()=="ducato"||
    type->getID()=="comarth"||
    type->getID()=="transit"||
    type->getID()=="urbino"||
    type->getID()==DEFAULT_ELECVTYPE_ID)
  {
    MSDevice_FEV::buildVehicleDevices(*this, myDevices);
  }
  else
  {
    // Should be one of...
    if(type->getID()!=DEFAULT_VTYPE_ID)
    {
      // Otherwise stop on error
      std::cout<<"unknown error at "<<__FILE__<<":"<<__LINE__<<"(unknown vehicl"
       <<"e type?)"<<std::endl;
      // exit(-1); // DO NOT!
    }
  }

  for(std::vector<MSDevice *>::iterator dev = myDevices.begin();
    dev != myDevices.end(); dev++)
  {
    myMoveReminders.push_back(std::make_pair(*dev, 0.));
  }
  myRoute->addReference();
  calculateArrivalPos();
    
  /* TASK Attempt to catch the 'to track' status of this vehicle, if mentioned
   */
  /* Is this a vehicle to track? */
  /*
  std::vector < VehicleToTrack * > vehiclesList =
    VehicleToTrack::vehiclesToTrackWithCharacteristicsList;
  int found = -1;
  for(int i = 0; i < vehiclesList.size(); i++)
    if(vehiclesList.at(i)->getIdentifier()==getID())
      found = i;
  if(found > -1)
  {
  */
    /* Take care, this could be exactly the opposite that the needed to do... */
    /* Then catch it's info from the vehicles to track list */
    /*
    getVehicleType().setMass(atof(vehiclesList.at(found)->getMass().c_str()));
    myType->setNumCells(atof(vehiclesList.at(found)->getNumberCells().c_str()));
    */
    /* ... */
  /*
  }
  */
}

MSBaseVehicle::~MSBaseVehicle()
{
  if(oc.getAnyVerbosity()>2)
    std::cout<<"MSBaseVehicle::~MSBaseVehicle(...)\n"<<std::endl;
  myRoute->release();
  delete myParameter;
  for(std::vector< MSDevice * >::iterator dev = myDevices.begin();
      dev != myDevices.end();
      ++dev)
  {
    delete(*dev);
  }
}


const std::string& MSBaseVehicle::getID() const {
  if(oc.getAnyVerbosity()>2)
    std::cout<<"const std::string& MSBaseVehicle::getId()\n"<<std::endl;
  if(oc.getAnyVerbosity()>3)
    std::cout<<"  (will return "<<myParameter->id.c_str()<<")"<<std::endl;
  return myParameter->id;
}


const SUMOVehicleParameter&
MSBaseVehicle::getParameter() const {
    return *myParameter;
}


const MSRoute& MSBaseVehicle::getRoute() const {
  if(oc.getAnyVerbosity()>1)
    printf("const MSRoute& MSBaseVehicle::getRoute()\n");    
  return *myRoute;
}


const MSVehicleType&
MSBaseVehicle::getVehicleType() const {
    return *myType;
}


SUMOReal
MSBaseVehicle::getMaxSpeed() const {
    if (myHasIndividualMaxSpeed) {
        return myIndividualMaxSpeed;
    }
    return myType->getMaxSpeed();
}


SUMOReal
MSBaseVehicle::adaptMaxSpeed(SUMOReal referenceSpeed) {
    if (myType->hasSpeedDeviation() && referenceSpeed != myReferenceSpeed) {
        myHasIndividualMaxSpeed = true;
        myIndividualMaxSpeed = myType->getMaxSpeedWithDeviation(referenceSpeed);
        myReferenceSpeed = referenceSpeed;
    }
    if (myHasIndividualMaxSpeed) {
        return myIndividualMaxSpeed;
    }
    return MIN2(myType->getMaxSpeed(), referenceSpeed);
}


const MSEdge*
MSBaseVehicle::succEdge(unsigned int nSuccs) const {
    if (myCurrEdge + nSuccs < myRoute->end()) {
        return *(myCurrEdge + nSuccs);
    } else {
        return 0;
    }
}


const MSEdge*
MSBaseVehicle::getEdge() const {
    return *myCurrEdge;
}

void MSBaseVehicle::reroute
 (SUMOTime t,SUMOAbstractRouter<MSEdge,SUMOVehicle> &router,bool withTaz)
{
    if(oc.getAnyVerbosity())
      std::cout<<"void MSBaseVehicle::reroute(...)\n"<<std::endl;
    // check whether to reroute
    std::vector<const MSEdge*> edges;
    if (withTaz && MSEdge::dictionary(myParameter->fromTaz + "-source") && MSEdge::dictionary(myParameter->toTaz + "-sink")) {
        router.compute(MSEdge::dictionary(myParameter->fromTaz + "-source"), MSEdge::dictionary(myParameter->toTaz + "-sink"), this, t, edges);
        if (edges.size() >= 2) {
            edges.erase(edges.begin());
            edges.pop_back();
        }
    } else {
        router.compute(*myCurrEdge, myRoute->getLastEdge(), this, t, edges);
    }
    if (edges.empty()) {
        WRITE_WARNING("No route for vehicle '" + getID() + "' found.");
        return;
    }
    replaceRouteEdges(edges, withTaz);
}

bool MSBaseVehicle::replaceRouteEdges(const MSEdgeVector& edges, bool onInit)
{
    if(oc.getAnyVerbosity())
      std::cout<<"bool MSBaseVehicle::replaceRouteEdges(...)"<<std::endl;
    // build a new id, first
    std::string id = getID();
    if (id[0] != '!') {
        id = "!" + id;
    }
    if (myRoute->getID().find("!var#") != std::string::npos) {
        id = myRoute->getID().substr(0, myRoute->getID().rfind("!var#") + 4) + toString(getNumberReroutes() + 1);
    } else {
        id = id + "!var#1";
    }
    MSRoute* newRoute = new MSRoute(id, edges, 0, myRoute->getColor(), myRoute->getStops());
    if (!MSRoute::dictionary(id, newRoute)) {
        delete newRoute;
        return false;
    }
    if (!replaceRoute(newRoute, onInit)) {
        newRoute->addReference();
        newRoute->release();
        return false;
    }
    return true;
}

SUMOReal MSBaseVehicle::getPreDawdleAcceleration() const {
    return 0;
}

void MSBaseVehicle::onDepart() {
    myDeparture = MSNet::getInstance()->getCurrentTimeStep();
    MSNet::getInstance()->getVehicleControl().vehicleDeparted(*this);
}

SUMOTime MSBaseVehicle::getDeparture() const {
    return myDeparture;
}

unsigned int MSBaseVehicle::getNumberReroutes() const {
    return myNumberReroutes;
}

void MSBaseVehicle::addPerson(MSPerson* /*person*/) {
}

bool MSBaseVehicle::isStopped() const {
    return false;
}

bool MSBaseVehicle::hasValidRoute(std::string& msg) const {
    if(OptionsCont::getOptions().getInt("verbosity"))
    {
      printf("bool MSBaseVehicle::hasValidRoute(...)\n");    
    }
    MSRouteIterator last = myRoute->end() - 1;
    // check connectivity, first
    for (MSRouteIterator e = myCurrEdge; e != last; ++e) {
        if ((*e)->allowedLanes(**(e + 1), myType->getVehicleClass()) == 0) {
            msg = "No connection between '" + (*e)->getID() + "' and '" + (*(e + 1))->getID() + "'.";
            return false;
        }
    }
    last = myRoute->end();
    // check usable lanes, then
    for (MSRouteIterator e = myCurrEdge; e != last; ++e) {
        if ((*e)->prohibits(this)) {
            msg = "Edge '" + (*e)->getID() + "' prohibits.";
            return false;
        }
    }
    return true;
}

void MSBaseVehicle::addReminder(MSMoveReminder* rem)
{
  if(oc.getAnyVerbosity())
    std::cout<<"void MSBaseVehicle::addReminder(...)\n"<<std::endl;
  myMoveReminders.push_back(std::make_pair(rem, 0.));
}

void MSBaseVehicle::removeReminder(MSMoveReminder* rem)
{
  if(oc.getAnyVerbosity()) 
    std::cout<<"void MSBaseVehicle::removeReminder(...)\n"<<std::endl;
  for (MoveReminderCont::iterator r = myMoveReminders.begin(); r != myMoveReminders.end(); ++r) {
    if (r->first == rem) {
      myMoveReminders.erase(r);
      return;
    }
  }
}

void MSBaseVehicle::activateReminders(const MSMoveReminder::Notification reason)
{
  if(oc.getAnyVerbosity())
    printf("void MSBaseVehicle::activateReminders(...)\n");
    for (MoveReminderCont::iterator rem = myMoveReminders.begin(); rem != myMoveReminders.end();) {
        if (rem->first->notifyEnter(*this, reason)) {
            ++rem;
        } else {
            rem = myMoveReminders.erase(rem);
        }
    }
}

void MSBaseVehicle::calculateArrivalPos()
{
    if(oc.getAnyVerbosity()>1)
      printf("void MSBaseVehicle::calculateArrivalPos()\n");    
    const SUMOReal lastLaneLength = (myRoute->getLastEdge()->getLanes())[0]->getLength();
    switch (myParameter->arrivalPosProcedure) {
        case ARRIVAL_POS_GIVEN:
            // Maybe we should warn the user about invalid inputs!
            myArrivalPos = MIN2(myParameter->arrivalPos, lastLaneLength);
            if (myArrivalPos < 0) {
                myArrivalPos = MAX2(myArrivalPos + lastLaneLength, static_cast<SUMOReal>(0));
            }
            break;
        case ARRIVAL_POS_RANDOM:
            myArrivalPos = RandHelper::rand(static_cast<SUMOReal>(0), lastLaneLength);
            break;
        case ARRIVAL_POS_MAX:
        case ARRIVAL_POS_DEFAULT:
            myArrivalPos = lastLaneLength;
            break;
    }
}

const MSVehicleType *MSBaseVehicle::getMyType() const
{
    return myType;
}
