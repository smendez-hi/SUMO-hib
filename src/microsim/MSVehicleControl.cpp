/****************************************************************************/
/// @file    MSVehicleControl.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Wed, 10. Dec 2003
/// @version $Id: MSVehicleControl.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// The class responsible for building and deletion of vehicles
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

#include "MSVehicleControl.h"
#include "MSVehicle.h"
#include "MSLane.h"
#include <microsim/devices/MSDevice.h>
#include <utils/common/FileHelpers.h>
#include <utils/common/RGBColor.h>
#include <utils/common/SUMOVTypeParameter.h>
#include <utils/iodevices/BinaryInputDevice.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/options/OptionsCont.h>
#include "scenload/VehicleToTrack.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS

// ===========================================================================
// member method definitions
// ===========================================================================
MSVehicleControl::MSVehicleControl
 (bool ecoGemRouterFlag)
 :myLoadedVehNo(0), myRunningVehNo(0), myEndedVehNo(0),
  myTotalDepartureDelay(0), myTotalTravelTime(0),
  myDefaultVTypeMayBeDeleted(true), myWaitingForPerson(0)
{
  /* This is the function which is calling MSVehicleType::build */
  if(OptionsCont::getOptions().getInt("simulationVerbosity")>1)
    std::cout<<"----> MSVehicleControl::MSVehicleControl(bool ecoGemRouterFlag="
     <<ecoGemRouterFlag<<")"<<std::endl;
  if(OptionsCont::getOptions().getInt("simulationVerbosity")>1)     
    std::cout<<"[050] MSVehicleControl::MSVehicleControl(bool ecoGemRouterFlag="
     <<ecoGemRouterFlag<<")\n (will now execute... SUMOVTypeParameter defType(D"
     <<"EFAULT_VTYPE_ID);)"<<std::endl;
  SUMOVTypeParameter defType(DEFAULT_VTYPE_ID);
  myVTypeDict[DEFAULT_VTYPE_ID] = MSVehicleType::build(defType);
  if(OptionsCont::getOptions().getInt("simulationVerbosity")>1)
    std::cout<<"[060] MSVehicleControl::MSVehicleControl(bool ecoGemRouterFlag="
     <<ecoGemRouterFlag<<")\n (will now execute... SUMOVTypeParameter defType(D"
     <<"DEFAULT_ELECVTYPE_ID);)"<<std::endl;
  /* HIB - 2012 Aug 20 (uprego) */
  SUMOVTypeParameter defElecType(DEFAULT_ELECVTYPE_ID);
  myVTypeDict[DEFAULT_ELECVTYPE_ID] = MSVehicleType::build(defElecType);
  /* Catch vehicles to track */
  /* scenload catching */
  if((OptionsCont::getOptions().getInt("buildVerbosity") > 1) ||
       (OptionsCont::getOptions().getInt("simulationVerbosity") > 1))
    std::cout<<"attempting to catch vehicles to track"<<std::endl;
  if(VehicleToTrack::getVehiclesToTrackWithCharacteristicsList() &&
     VehicleToTrack::getVehiclesToTrackWithCharacteristicsList()->size() > 0)
  {
    if((OptionsCont::getOptions().getInt("buildVerbosity") > 1) ||
       (OptionsCont::getOptions().getInt("simulationVerbosity") > 1))
      std::cout<<"catched vehicles to track"<<std::endl;
    
    std::cout<<"VehicleToTrack::getVehiclesToTrackWithCharacteristicsList()->size()="<<VehicleToTrack::getVehiclesToTrackWithCharacteristicsList()->size()<<std::endl;
    for(int i=0; i<VehicleToTrack::getVehiclesToTrackWithCharacteristicsList()->size(); i++)
    {
      /* Create a vtype for each one */
      // ALERT with segfaults
      //const std::string _ = VehicleToTrack::getVehiclesToTrackWithCharacteristicsList().at(i)->getIdentifier(); // TODO ACTIVATE
      //SUMOVTypeParameter customType(_); // TODO ACTIVATE
      /*SUMOVTypeParameter customType(VehicleToTrack::getVehiclesToTrackWithCharacteristicsList().at(i)->getIdentifier());*/
      // ALERT with segfaults
      //myVTypeDict[VehicleToTrack::getVehiclesToTrackWithCharacteristicsList().at(i)->getIdentifier()] = MSVehicleType::build(customType); // TODO ACTIVATE
      /*
      if(VehicleToTrack::getVehiclesToTrackWithCharacteristicsList().at(i)->getIdentifier() == getID())
      {
      */      
        /* TODO PROCESS */
        /*this->getMyType()->getCooperLosses();*/
        ;
      /*
      }
      */
    }
    
  }
  else
  {
    if((OptionsCont::getOptions().getInt("buildVerbosity") > 1) ||
       (OptionsCont::getOptions().getInt("simulationVerbosity") > 1))
      std::cout<<"no vehicles to track catched"<<std::endl;
  }  
  
  ecoGemRouter = ecoGemRouterFlag;
}


MSVehicleControl::~MSVehicleControl() {
    // delete vehicles
    for (VehicleDictType::iterator i = myVehicleDict.begin(); i != myVehicleDict.end(); ++i) {
        delete(*i).second;
    }
    myVehicleDict.clear();
    // delete vehicle type distributions
    for (VTypeDistDictType::iterator i = myVTypeDistDict.begin(); i != myVTypeDistDict.end(); ++i) {
        delete(*i).second;
    }
    myVTypeDistDict.clear();
    // delete vehicle types
    for (VTypeDictType::iterator i = myVTypeDict.begin(); i != myVTypeDict.end(); ++i) {
        delete(*i).second;
    }
    myVTypeDict.clear();
}

SUMOVehicle* MSVehicleControl::buildVehicle
 (SUMOVehicleParameter* defs, const MSRoute* route, const MSVehicleType* type)
{
  OptionsCont &oc=OptionsCont::getOptions();
  if(oc.getAnyVerbosity()>2)
  {
    std::cout << "SUMOVehicle* MSVehicleControl::buildVehicle(...)" << std::endl;
    std::cout << " defs->getID()=" << defs->id << std::endl;
    std::cout << " route->getID()=" << route->getID() << std::endl;
    std::cout << " type->getID()=" << type->getID() << std::endl;
  }
  myLoadedVehNo++;
  MSVehicle* built=new MSVehicle(defs,route,type,myLoadedVehNo-1
    /*,ecoGemRouter*/);
  MSNet::getInstance()->informVehicleStateListener
   (built, MSNet::VEHICLE_STATE_BUILT);
  return built;
}

void MSVehicleControl::scheduleVehicleRemoval(SUMOVehicle* veh) {
    assert(myRunningVehNo > 0);
    for (std::vector<MSDevice*>::const_iterator i = veh->getDevices().begin();
         i != veh->getDevices().end();
         ++i)
    {
        (*i)->generateOutput();
    }
    if (OptionsCont::getOptions().isSet("tripinfo-output")) {
        OutputDevice::getDeviceByOption("tripinfo-output").closeTag(veh->
         getDevices().size() == 1);
    }
    myTotalTravelTime += STEPS2TIME(MSNet::getInstance()->getCurrentTimeStep() -
     veh->getDeparture());
    myRunningVehNo--;
    MSNet::getInstance()->informVehicleStateListener(veh,
     MSNet::VEHICLE_STATE_ARRIVED);
    deleteVehicle(veh);
}

void MSVehicleControl::printMeanWaitingTime(OutputDevice& od) const {
    if (getDepartedVehicleNo() == 0) {
        od << -1.;
    } else {
        od << (myTotalDepartureDelay / (SUMOReal) getDepartedVehicleNo());
    }
}

void MSVehicleControl::printMeanTravelTime(OutputDevice& od) const
{
    if (myEndedVehNo == 0) {
        od << -1.;
    } else {
        od << (myTotalTravelTime / (SUMOReal) myEndedVehNo);
    }
}

void MSVehicleControl::vehicleDeparted(const SUMOVehicle& v)
{
    ++myRunningVehNo;
    myTotalDepartureDelay += STEPS2TIME(v.getDeparture() -
     STEPFLOOR(v.getParameter().depart));
    MSNet::getInstance()->informVehicleStateListener(&v,
     MSNet::VEHICLE_STATE_DEPARTED);
}

void MSVehicleControl::saveState
 (std::ostream& /*os*/)
{}

void MSVehicleControl::loadState
 (BinaryInputDevice& /*bis*/,const SUMOTime /*offset*/)
{}

bool MSVehicleControl::addVehicle(const std::string& id, SUMOVehicle* v) {
    VehicleDictType::iterator it = myVehicleDict.find(id);
    if (it == myVehicleDict.end()) {
        // id not in myVehicleDict.
        myVehicleDict[id] = v;
        return true;
    }
    return false;
}


SUMOVehicle*
MSVehicleControl::getVehicle(const std::string& id) const {
    VehicleDictType::const_iterator it = myVehicleDict.find(id);
    if (it == myVehicleDict.end()) {
        return 0;
    }
    return it->second;
}


void
MSVehicleControl::deleteVehicle(SUMOVehicle* veh) {
    myEndedVehNo++;
    myVehicleDict.erase(veh->getID());
    delete veh;
}


MSVehicleControl::constVehIt
MSVehicleControl::loadedVehBegin() const {
    return myVehicleDict.begin();
}


MSVehicleControl::constVehIt
MSVehicleControl::loadedVehEnd() const {
    return myVehicleDict.end();
}


bool
MSVehicleControl::checkVType(const std::string& id) {
    if (id == DEFAULT_VTYPE_ID) {
        if (myDefaultVTypeMayBeDeleted) {
            delete myVTypeDict[id];
            myVTypeDict.erase(myVTypeDict.find(id));
            myDefaultVTypeMayBeDeleted = false;
        } else {
            return false;
        }
    } else {
        if (myVTypeDict.find(id) != myVTypeDict.end() || myVTypeDistDict.find(id) != myVTypeDistDict.end()) {
            return false;
        }
    }
    return true;
}

bool
MSVehicleControl::addVType(MSVehicleType* vehType) {
    if (checkVType(vehType->getID())) {
        myVTypeDict[vehType->getID()] = vehType;
        return true;
    }
    return false;
}


bool
MSVehicleControl::addVTypeDistribution(const std::string& id, RandomDistributor<MSVehicleType*> *vehTypeDistribution) {
    if (checkVType(id)) {
        myVTypeDistDict[id] = vehTypeDistribution;
        return true;
    }
    return false;
}


bool
MSVehicleControl::hasVTypeDistribution(const std::string& id) const {
    return myVTypeDistDict.find(id) != myVTypeDistDict.end();
}

MSVehicleType* MSVehicleControl::getVType(const std::string& id)
{
  /* Here it is cautious to create a MSVehicleType for each of the vehicles to
   * track */
  //for(;;);
  VTypeDictType::iterator it = myVTypeDict.find(id);
  if(it == myVTypeDict.end()){
    VTypeDistDictType::iterator it2 = myVTypeDistDict.find(id);
    if (it2 == myVTypeDistDict.end()) {
        return 0;
    }
    return it2->second->get();
  }
  if(id == DEFAULT_VTYPE_ID){
    myDefaultVTypeMayBeDeleted = false;
  }
  return it->second;
}

void MSVehicleControl::insertVTypeIDs(std::vector<std::string> &into) const
{
    into.reserve(into.size() + myVTypeDict.size() + myVTypeDistDict.size());
    for (VTypeDictType::const_iterator i = myVTypeDict.begin(); i != myVTypeDict.end(); ++i) {
        into.push_back((*i).first);
    }
    for (VTypeDistDictType::const_iterator i = myVTypeDistDict.begin(); i != myVTypeDistDict.end(); ++i) {
        into.push_back((*i).first);
    }
}

void MSVehicleControl::addWaiting(const MSEdge* const edge, SUMOVehicle* vehicle) {
    if (myWaiting.find(edge) == myWaiting.end()) {
        myWaiting[edge] = std::vector<SUMOVehicle*>();
    }
    myWaiting[edge].push_back(vehicle);
}

void MSVehicleControl::removeWaiting(const MSEdge* const edge, SUMOVehicle* vehicle) {
    if (myWaiting.find(edge) != myWaiting.end()) {
        std::vector<SUMOVehicle*>::iterator it = std::find(myWaiting[edge].begin(), myWaiting[edge].end(), vehicle);
        if (it != myWaiting[edge].end()) {
            myWaiting[edge].erase(it);
        }
    }
}


SUMOVehicle*
MSVehicleControl::getWaitingVehicle(const MSEdge* const edge, const std::set<std::string> &lines) {
    if (myWaiting.find(edge) != myWaiting.end()) {
        for (std::vector<SUMOVehicle*>::const_iterator it = myWaiting[edge].begin(); it != myWaiting[edge].end(); ++it) {
            const std::string& line = (*it)->getParameter().line == "" ? (*it)->getParameter().id : (*it)->getParameter().line;
            if (lines.count(line)) {
                return (*it);
            }
        }
    }
    return 0;
}


void
MSVehicleControl::abortWaiting() {
    for (VehicleDictType::iterator i = myVehicleDict.begin(); i != myVehicleDict.end(); ++i) {
        WRITE_WARNING("Vehicle " + i->first + " aborted waiting for a person that will never come.");
    }
}

/****************************************************************************/

