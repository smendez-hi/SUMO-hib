/****************************************************************************/
/// @file    MSEdge.cpp
/// @author  Christian Roessel
/// @author  Jakob Erdmann
/// @author  Christoph Sommer
/// @author  Daniel Krajzewicz
/// @author  Laura Bieker
/// @author  Michael Behrisch
/// @author  Sascha Krieg
/// @date    Tue, 06 Mar 2001
/// @version $Id: MSEdge.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// A road/street connecting two junctions
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
#include "MSEdge.h"
#include "MSLane.h"
#include "MSLaneChanger.h"
#include "MSGlobals.h"
#include <algorithm>
#include <iostream>
#include <cassert>
#include "MSVehicle.h"
#include <utils/common/StringTokenizer.h>
#include <utils/options/OptionsCont.h>
#include "MSEdgeWeightsStorage.h"
#ifdef HAVE_MESOSIM
#include <mesosim/MELoop.h>
#include <mesosim/MESegment.h>
#endif
#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS

static int criticalWarningCounter1 = 0, criticalWarningCounter2 = 0;
// ===========================================================================
// static member definitions
// ===========================================================================
MSEdge::DictType MSEdge::myDict;
std::vector<MSEdge*> MSEdge::myEdges;

// ===========================================================================
// member method definitions
// ===========================================================================
MSEdge::MSEdge
 (const std::string& id, unsigned int numericalID, 
  const std::string& streetName)
 :myID(id), myNumericalID(numericalID), myLanes(0), myLaneChanger(0),
  myVaporizationRequests(0), myLastFailedInsertionTime(-1),
  myStreetName(streetName), slopeSet(false)
{}

MSEdge::~MSEdge()
{
    delete myLaneChanger;
    for (AllowedLanesCont::iterator i1 = myAllowed.begin(); i1 != myAllowed.end(); i1++) {
        delete(*i1).second;
    }
    for (ClassedAllowedLanesCont::iterator i2 = myClassedAllowed.begin(); i2 != myClassedAllowed.end(); i2++) {
        for (AllowedLanesCont::iterator i1 = (*i2).second.begin(); i1 != (*i2).second.end(); i1++) {
            delete(*i1).second;
        }
    }
    delete myLanes;
    // Note: Lanes are delete using MSLane::clear();
}

void MSEdge::initialize
 (std::vector<MSLane*>* lanes, EdgeBasicFunction function, SUMOReal slope)
{
    assert(function == EDGEFUNCTION_DISTRICT || lanes != 0);
    myLanes = lanes;
    myFunction = function;
    if (myLanes && myLanes->size() > 1 && function != EDGEFUNCTION_INTERNAL) {
        myLaneChanger = new MSLaneChanger(myLanes, OptionsCont::getOptions().getBool("lanechange.allow-swap"));
    }
    mySlope = slope;
}

#include <stdio.h>

void MSEdge::closeBuilding()
{
  if(OptionsCont::getOptions().getInt("buildVerbosity") > 1)
  {
    printf("----> void MSEdge::closeBuilding\n");
    std::cout<<" getID()="<<getID()<<std::endl;
    /*
    if(getFromJunction())
    {
      std::cout<<"getFromJunction()="<<getFromJunction()->getID()<<std::endl;
    }
    */
    /*
    if(getToJunction())
    {
      std::cout<<" getToJunction()="<<getToJunction()<<std::endl;
      try
      {
        if(getToJunction()->getID().length() > 0)
        {
          std::cout<<" getToJunction()->getID().length() > 0"<<std::endl;
          std::cout<<" getToJunction()="<<getToJunction()->getID()<<std::endl;
        }
      }
      catch(std::runtime_error)
      {
        std::cout<<" CRITICAL runtime_error catched"<<std::endl;
      }
    }
    */
  }
  myAllowed[0] = new std::vector<MSLane*>();
  for(std::vector<MSLane*>::iterator i = myLanes->begin(); i != myLanes->end(); ++i)
  {
    myAllowed[0]->push_back(*i);
    const MSLinkCont& lc = (*i)->getLinkCont();
    for(MSLinkCont::const_iterator j = lc.begin(); j != lc.end(); ++j)
    {
      MSLane* toL = (*j)->getLane();
      if(toL != 0) {
        MSEdge& to = toL->getEdge();
        //
        if(std::find(mySuccessors.begin(), mySuccessors.end(), &to) == mySuccessors.end())
        {
            mySuccessors.push_back(&to);
        }
        if(std::find(to.myPredeccesors.begin(), to.myPredeccesors.end(), this) == to.myPredeccesors.end())
        {
            to.myPredeccesors.push_back(this);
        }
        //
        if(myAllowed.find(&to) == myAllowed.end())
        {
            myAllowed[&to] = new std::vector<MSLane*>();
        }
        myAllowed[&to]->push_back(*i);
      }
    }
  }
  std::sort(mySuccessors.begin(), mySuccessors.end(), by_id_sorter());
  rebuildAllowedLanes();
  if(OptionsCont::getOptions().getInt("buildVerbosity") > 1)
    printf("<---- void MSEdge::closeBuilding\n");  
}

void MSEdge::rebuildAllowedLanes() {
    // build the classed allowed lanes
    myHaveClassConstraints = false;
    // build list of vehicle classes that are constrained
    // ... all others will be not regarded (allowed) ...
    SUMOVehicleClasses vclasses;
    for (std::vector<MSLane*>::const_iterator i2 = myLanes->begin(); i2 != myLanes->end(); ++i2) {
        const SUMOVehicleClasses& allowed = (*i2)->getAllowedClasses();
        for (SUMOVehicleClasses::const_iterator j = allowed.begin(); j != allowed.end(); j++) {
            vclasses.insert(*j);
        }
        const SUMOVehicleClasses& disallowed = (*i2)->getNotAllowedClasses();
        for (SUMOVehicleClasses::const_iterator j = disallowed.begin(); j != disallowed.end(); j++) {
            vclasses.insert(*j);
        }
    }
    // go through these classes
    for (SUMOVehicleClasses::const_iterator j = vclasses.begin(); j != vclasses.end(); ++j) {
        // go through connected edges
        for (AllowedLanesCont::iterator i1 = myAllowed.begin(); i1 != myAllowed.end(); ++i1) {
            delete myClassedAllowed[*j][(*i1).first];
            myClassedAllowed[*j][(*i1).first] = new std::vector<MSLane*>();
            // go through lanes approaching current edge
            for (std::vector<MSLane*>::iterator i2 = (*i1).second->begin(); i2 != (*i1).second->end(); ++i2) {
                // allows the current vehicle class?
                if ((*i2)->allowsVehicleClass(*j)) {
                    // -> may be used
                    myClassedAllowed[*j][(*i1).first]->push_back(*i2);
                }
            }
            // assert that 0 is returned if no connection is allowed for a class
            if (myClassedAllowed[*j][(*i1).first]->size() == 0) {
                delete myClassedAllowed[*j][(*i1).first];
                myClassedAllowed[*j][(*i1).first] = 0;
            }
        }
        myHaveClassConstraints = true;
    }
}

// ------------ Access to the edge's lanes
MSLane* MSEdge::leftLane(const MSLane* const lane) const
{
    std::vector<MSLane*>::iterator laneIt = find(myLanes->begin(), myLanes->end(), lane);
    if (laneIt == myLanes->end() || laneIt == myLanes->end() - 1) {
        return 0;
    }
    return *(laneIt + 1);
}

MSLane* MSEdge::rightLane(const MSLane* const lane) const
{
    std::vector<MSLane*>::iterator laneIt = find(myLanes->begin(), myLanes->end(), lane);
    if (laneIt == myLanes->end() || laneIt == myLanes->begin()) {
        return 0;
    }
    return *(laneIt - 1);
}

const std::vector<MSLane*>* MSEdge::allowedLanes
 (const MSEdge& destination, SUMOVehicleClass vclass) const
{
    return allowedLanes(&destination, vclass);
}

const std::vector<MSLane*>* MSEdge::allowedLanes
 (SUMOVehicleClass vclass) const
{
    return allowedLanes(0, vclass);
}

const std::vector<MSLane*>* MSEdge::allowedLanes
 (const MSEdge* destination, SUMOVehicleClass vclass) const
{
    if (myHaveClassConstraints && vclass != SVC_UNKNOWN) {
        ClassedAllowedLanesCont::const_iterator i = myClassedAllowed.find(vclass);
        if (i != myClassedAllowed.end()) {
            const AllowedLanesCont& c = (*i).second;
            AllowedLanesCont::const_iterator j = (*i).second.find(destination);
            if (j == c.end()) {
                // Destination-edge not found.
                return 0;
            }
            return (*j).second;
        }
    }
    AllowedLanesCont::const_iterator it = myAllowed.find(destination);
    if (it != myAllowed.end()) {
        return it->second;
    } else {
        // Destination-edge not found.
        return 0;
    }
}


// ------------
SUMOTime
MSEdge::incVaporization(SUMOTime) {
    ++myVaporizationRequests;
    return 0;
}


SUMOTime
MSEdge::decVaporization(SUMOTime) {
    --myVaporizationRequests;
    return 0;
}


MSLane*
MSEdge::getFreeLane(const std::vector<MSLane*>* allowed, const SUMOVehicleClass vclass) const {
    if (allowed == 0) {
        allowed = allowedLanes(vclass);
    }
    MSLane* res = 0;
    if (allowed != 0) {
        unsigned int noCars = INT_MAX;
        for (std::vector<MSLane*>::const_iterator i = allowed->begin(); i != allowed->end(); ++i) {
            if ((*i)->getVehicleNumber() < noCars) {
                res = (*i);
                noCars = (*i)->getVehicleNumber();
            }
        }
    }
    return res;
}


MSLane*
MSEdge::getDepartLane(const MSVehicle& veh) const {
    switch (veh.getParameter().departLaneProcedure) {
        case DEPART_LANE_GIVEN:
            if ((int) myLanes->size() <= veh.getParameter().departLane || !(*myLanes)[veh.getParameter().departLane]->allowsVehicleClass(veh.getVehicleType().getVehicleClass())) {
                return 0;
            }
            return (*myLanes)[veh.getParameter().departLane];
        case DEPART_LANE_RANDOM:
            return RandHelper::getRandomFrom(*allowedLanes(veh.getVehicleType().getVehicleClass()));
        case DEPART_LANE_FREE:
            return getFreeLane(0, veh.getVehicleType().getVehicleClass());
        case DEPART_LANE_ALLOWED_FREE:
            if (veh.getRoute().size() == 1) {
                return getFreeLane(0, veh.getVehicleType().getVehicleClass());
            } else {
                return getFreeLane(allowedLanes(**(veh.getRoute().begin() + 1)), veh.getVehicleType().getVehicleClass());
            }
        case DEPART_LANE_BEST_FREE: {
            const std::vector<MSVehicle::LaneQ> &bl = veh.getBestLanes(false, (*myLanes)[0]);
            SUMOReal bestLength = -1;
            for (std::vector<MSVehicle::LaneQ>::const_iterator i = bl.begin(); i != bl.end(); ++i) {
                if ((*i).length > bestLength) {
                    bestLength = (*i).length;
                }
            }
            std::vector<MSLane*> *bestLanes = new std::vector<MSLane*>();
            for (std::vector<MSVehicle::LaneQ>::const_iterator i = bl.begin(); i != bl.end(); ++i) {
                if ((*i).length == bestLength) {
                    bestLanes->push_back((*i).lane);
                }
            }
            MSLane* ret = getFreeLane(bestLanes, veh.getVehicleType().getVehicleClass());
            delete bestLanes;
            return ret;
        }
        case DEPART_LANE_DEFAULT:
        default:
            break;
    }
    if (!(*myLanes)[0]->allowsVehicleClass(veh.getVehicleType().getVehicleClass())) {
        return 0;
    }
    return (*myLanes)[0];
}


bool
MSEdge::insertVehicle(SUMOVehicle& v, SUMOTime time) const {
    // when vaporizing, no vehicles are inserted...
    if (isVaporizing()) {
        return false;
    }
#ifdef HAVE_MESOSIM
    if (MSGlobals::gUseMesoSim) {
        const SUMOVehicleParameter& pars = v.getParameter();
        SUMOReal pos = 0.0;
        switch (pars.departPosProcedure) {
            case DEPART_POS_GIVEN:
                if (pars.departPos >= 0.) {
                    pos = pars.departPos;
                } else {
                    pos = pars.departPos + getLanes()[0]->getLength();
                }
                break;
            case DEPART_POS_RANDOM:
            case DEPART_POS_RANDOM_FREE:
                pos = RandHelper::rand(getLanes()[0]->getLength());
                break;
            default:
                break;
        }
        bool result = false;
        MESegment* segment = MSGlobals::gMesoNet->getSegmentForEdge(*this, pos);
        MEVehicle* veh = static_cast<MEVehicle*>(&v);
        if (pars.departPosProcedure == DEPART_POS_FREE) {
            while (segment != 0 && !result) {
                result = segment->initialise(veh, time);
                segment = segment->getNextSegment();
            }
        } else {
            result = segment->initialise(veh, time);
        }
        return result;
    }
#endif
    MSLane* insertionLane = getDepartLane(static_cast<MSVehicle&>(v));
    return insertionLane != 0 && insertionLane->insertVehicle(static_cast<MSVehicle&>(v));
}


void
MSEdge::changeLanes(SUMOTime t) {
    if (myFunction == EDGEFUNCTION_INTERNAL) {
        return;
    }
    assert(myLaneChanger != 0);
    myLaneChanger->laneChange(t);
}



#ifdef HAVE_INTERNAL_LANES
const MSEdge*
MSEdge::getInternalFollowingEdge(MSEdge* followerAfterInternal) const {
    //@todo to be optimized
    for (std::vector<MSLane*>::const_iterator i = myLanes->begin(); i != myLanes->end(); ++i) {
        MSLane* l = *i;
        const MSLinkCont& lc = l->getLinkCont();
        for (MSLinkCont::const_iterator j = lc.begin(); j != lc.end(); ++j) {
            MSLink* link = *j;
            if (&link->getLane()->getEdge() == followerAfterInternal) {
                return &link->getViaLane()->getEdge();
            }
        }
    }
    return 0;
}
#endif


SUMOReal MSEdge::getCurrentTravelTime() const
{
    SUMOReal v = 0;
#ifdef HAVE_MESOSIM
    if (MSGlobals::gUseMesoSim) {
        MESegment* first = MSGlobals::gMesoNet->getSegmentForEdge(*this);
        unsigned segments = 0;
        do {
            v += first->getMeanSpeed();
            first = first->getNextSegment();
            segments++;
        } while (first != 0);
        v /= (SUMOReal) segments;
    } else {
#endif
        for (std::vector<MSLane*>::iterator i = myLanes->begin(); i != myLanes->end(); ++i) {
            v += (*i)->getMeanSpeed();
        }
        v /= (SUMOReal) myLanes->size();
#ifdef HAVE_MESOSIM
    }
#endif
    if (v != 0) {
        return (*myLanes)[0]->getLength() / v;
    } else {
        return 1000000.;
    }
}

bool MSEdge::prohibits(const SUMOVehicle* const vehicle) const
{
    if (myFunction == EDGEFUNCTION_DISTRICT || !myHaveClassConstraints) {
        return false;
    }
    SUMOVehicleClass vclass = vehicle->getVehicleType().getVehicleClass();
    for (std::vector<MSLane*>::iterator i = myLanes->begin(); i != myLanes->end(); ++i) {
        if ((*i)->allowsVehicleClass(vclass)) {
            return false;
        }
    }
    return true;
}


bool
MSEdge::dictionary(const std::string& id, MSEdge* ptr) {
    DictType::iterator it = myDict.find(id);
    if (it == myDict.end()) {
        // id not in myDict.
        myDict[id] = ptr;
        while (myEdges.size() < ptr->getNumericalID() + 1) {
            myEdges.push_back(0);
        }
        myEdges[ptr->getNumericalID()] = ptr;
        return true;
    }
    return false;
}


MSEdge*
MSEdge::dictionary(const std::string& id) {
    DictType::iterator it = myDict.find(id);
    if (it == myDict.end()) {
        // id not in myDict.
        return 0;
    }
    return it->second;
}


MSEdge*
MSEdge::dictionary(size_t id) {
    assert(myEdges.size() > id);
    return myEdges[id];
}


size_t
MSEdge::dictSize() {
    return myDict.size();
}

void MSEdge::clear() {
    for (DictType::iterator i = myDict.begin(); i != myDict.end(); ++i) {
        delete(*i).second;
    }
    myDict.clear();
}

void MSEdge::insertIDs(std::vector<std::string> &into) {
    for (DictType::iterator i = myDict.begin(); i != myDict.end(); ++i) {
        into.push_back((*i).first);
    }
}

void MSEdge::parseEdgesList(const std::string& desc, std::vector<const MSEdge*> &into,
                       const std::string& rid) {
    StringTokenizer st(desc);
    parseEdgesList(st.getVector(), into, rid);
}

void MSEdge::parseEdgesList(const std::vector<std::string> &desc, std::vector<const MSEdge*> &into,
                       const std::string& rid) {
    for (std::vector<std::string>::const_iterator i = desc.begin(); i != desc.end(); ++i) {
        const MSEdge* edge = MSEdge::dictionary(*i);
        // check whether the edge exists
        if (edge == 0) {
            throw ProcessError("The edge '" + *i + "' within the route " + rid + " is not known."
                               + "\n The route can not be build.");
        }
        into.push_back(edge);
    }
}

void MSEdge::setSlope(SUMOReal arg)
{
  OptionsCont &oc = OptionsCont::getOptions();
  const unsigned short int warningPeriod = 10;
  if(getSlopeSetStatus())
  {
    if(oc.getBuildVerbosity()||oc.getSimulationVerbosity())
    {
      if(criticalWarningCounter1%10==0)
      {
      std::cout<<"CRITICAL setting slope for second time"<<std::endl;
      std::cout<<" (MSEdge was:"<<getID()<<")"<<std::endl;
      std::cout<<" (reported "<<criticalWarningCounter1<<" times)"<<std::endl;
      criticalWarningCounter1++;
      }
    }
    if(getSlope()!=arg)
    {
      if(oc.getBuildVerbosity()||oc.getSimulationVerbosity())
      {
        if(!criticalWarningCounter2%10==0)
        {
        std::cout<<"CRITICAL second slope ("<<arg<<") was not the same as the f"
          <<"irst slope ("<<getSlope()<<")"<<std::endl;
        std::cout<<" (reported "<<criticalWarningCounter1<<" times)"<<std::endl;
        criticalWarningCounter2++;
        }
      }
      //exit(0xffff); // Segfaults without the exit ?
      //exit(-1); // Segfaults without the exit ?
    }
  }
  else
  {
    if(oc.getBuildVerbosity()>1||oc.getSimulationVerbosity()>1)
      std::cout<<"MSEdge::setSlope(SUMOReal arg="<<arg<<")"<<std::endl;
    mySlope = arg;
    for(int i=0;i<getLanes().size();i++)
    {
      getLanes().at(i)->setSlope(arg);
    }
    slopeSet = true;
  }
}

MSJunction * MSEdge::getFromJunction()/* const*/ { return myFromJunction; }

void MSEdge::setFromJunction(MSJunction *arg) { myFromJunction = arg; }

MSJunction * MSEdge::getToJunction()/* const*/ { return myToJunction; }

void MSEdge::setToJunction(MSJunction *arg) { myToJunction = arg; }

std::string MSEdge::getNameOfMyToJunction() const { return nameOfMyToJunction; }

void MSEdge::setNameOfMyToJunction(std::string arg) { nameOfMyToJunction = arg; }