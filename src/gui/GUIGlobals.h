/****************************************************************************/
/// @file    GUIGlobals.h
/// @author  Daniel Krajzewicz
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @date    2004
/// @version $Id: GUIGlobals.h 11671 2012-01-07 20:14:30Z behrisch $
///
// Some global variables (yep)
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
#ifndef GUIGlobals_h
#define GUIGlobals_h

// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <vector>
#include <fx.h>
#include <utils/common/SUMOTime.h>
#include <utils/gui/globjects/GUIGlObjectStorage.h>

// ===========================================================================
// class declarations
// ===========================================================================
class GUINet;

// ===========================================================================
// global variables declarations
// ===========================================================================
/// the window shall be closed when the simulation has ended
extern bool gQuitOnEnd;

/// List of breakpoint
extern std::vector<SUMOTime> gBreakpoints;

#endif