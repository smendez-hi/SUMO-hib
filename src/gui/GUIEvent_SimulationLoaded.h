/****************************************************************************/
/// @file    GUIEvent_SimulationLoaded.h
/// @author  Daniel Krajzewicz
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id: GUIEvent_SimulationLoaded.h 11671 2012-01-07 20:14:30Z behrisch $
///
// Event send when the simulation has been loaded by GUILadThread
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
#ifndef GUIEvent_SimulationLoaded_h
#define GUIEvent_SimulationLoaded_h


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
#include <utils/gui/events/GUIEvent.h>
#include <utils/common/SUMOTime.h>


// ===========================================================================
// class declarations
// ===========================================================================
class GUINet;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class  GUIEvent_SimulationLoaded
 *
 * Throw to GUIApplicationWindow from GUILoadThread after a simulation has
 * been loaded or the loading process failed
 */
class GUIEvent_SimulationLoaded : public GUIEvent {
public:
    /// constructor
    GUIEvent_SimulationLoaded(GUINet* net,
                              SUMOTime startTime, SUMOTime endTime,
                              const std::string& file,
                              const std::string& settingsFile)
        : GUIEvent(EVENT_SIMULATION_LOADED),
          myNet(net), myBegin(startTime), myEnd(endTime),
          myFile(file), mySettingsFile(settingsFile) { }

    /// destructor
    ~GUIEvent_SimulationLoaded() { }

public:
    /// the loaded net
    GUINet*          myNet;

    /// the time the simulation shall start with
    SUMOTime     myBegin;

    /// the time the simulation shall end with
    SUMOTime     myEnd;

    /// the name of the loaded file
    std::string     myFile;

    /// the name of the settings file to load
    std::string     mySettingsFile;

};


#endif

/****************************************************************************/

