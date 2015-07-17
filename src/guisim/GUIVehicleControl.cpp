/****************************************************************************/
/// @file    GUIVehicleControl.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Wed, 10. Dec 2003
/// @version $Id: GUIVehicleControl.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// The class responsible for building and deletion of vehicles (gui-version)
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

#include <utils/foxtools/MFXMutex.h>
#include <utils/options/OptionsCont.h>
#include <gui/GUIGlobals.h>
#include "GUIVehicleControl.h"
#include "GUIVehicle.h"
#include "GUINet.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS

// ===========================================================================
// member method definitions
// ===========================================================================
GUIVehicleControl::GUIVehicleControl(/*bool ecoGemRouter*/)
    : MSVehicleControl(/*ecoGemRouter*/) {}

GUIVehicleControl::~GUIVehicleControl() {
    // just to quit cleanly on a failure
    if (myLock.locked()) {
        myLock.unlock();
    }
}

SUMOVehicle* GUIVehicleControl::buildVehicle
 (SUMOVehicleParameter* defs, const MSRoute* route, const MSVehicleType* type/*,
                                bool ecoGemRouter*/)
{
  if(OptionsCont::getOptions().getAnyVerbosity()>1)
  {
    std::cout << "SUMOVehicle* GUIVehicleControl::buildVehicle(...)"
              << std::endl;
  }
  myLoadedVehNo++;
  MSVehicle* built = new GUIVehicle(defs, route, type, myLoadedVehNo - 1/*,
   ecoGemRouter*/);
  MSNet::getInstance()->informVehicleStateListener(built, MSNet::VEHICLE_STATE_BUILT);
  return built;
}

bool GUIVehicleControl::addVehicle(const std::string& id, SUMOVehicle* v) {
  myLock.lock();
  const bool result = MSVehicleControl::addVehicle(id, v);
  myLock.unlock();
  return result;
}

void GUIVehicleControl::deleteVehicle(SUMOVehicle* veh) {
    myLock.lock();
    MSVehicleControl::deleteVehicle(veh);
    myLock.unlock();
}

void GUIVehicleControl::insertVehicleIDs(std::vector<GUIGlID> &into) {
    myLock.lock();
    into.reserve(myVehicleDict.size());
    for (VehicleDictType::iterator i = myVehicleDict.begin(); i != myVehicleDict.end(); ++i) {
        SUMOVehicle* veh = (*i).second;
        if (veh->isOnRoad()) {
            if(OptionsCont::getOptions().getAnyVerbosity()>1)
              printf("((*i).second)->getGlID() == %d\n", static_cast<GUIVehicle*>((*i).second)->getGlID());
            into.push_back(static_cast<GUIVehicle*>((*i).second)->getGlID());
        }
    }
    myLock.unlock();
}
