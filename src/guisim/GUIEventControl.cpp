/****************************************************************************/
/// @file    GUIEventControl.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Mon, 04 Feb 2008
/// @version $Id: GUIEventControl.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// Stores time-dependant events and executes them at the proper time (guisim)
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

#include <cassert>
#include <utils/foxtools/MFXMutex.h>
#include "GUIEventControl.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// member definitions
// ===========================================================================
GUIEventControl::GUIEventControl() {}


GUIEventControl::~GUIEventControl() {
}


SUMOTime
GUIEventControl::addEvent(Command* operation,
                          SUMOTime execTimeStep,
                          AdaptType type) {
    myLock.lock();
    SUMOTime ret = MSEventControl::addEvent(operation, execTimeStep, type);
    myLock.unlock();
    return ret;
}


void
GUIEventControl::execute(SUMOTime execTime) {
    myLock.lock();
    try {
        MSEventControl::execute(execTime);
    } catch (ProcessError&) {
        myLock.unlock();
        throw;
    }
    myLock.unlock();
}



/****************************************************************************/

