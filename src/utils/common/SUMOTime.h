/****************************************************************************/
/// @file    SUMOTime.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Fri, 29.04.2005
/// @version $Id: SUMOTime.h 11671 2012-01-07 20:14:30Z behrisch $
///
// Variables, methods, and tools for internal time representation
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
#ifndef SUMOTime_h
#define SUMOTime_h

// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif
#include "additionalConfig.h"
#include <climits>
#include <string>
#include "UtilExceptions.h"

// ===========================================================================
// type definitions
// ===========================================================================
#if defined INT4SUMOTIME
typedef int SUMOTime;
#define SUMOTime_MAX INT_MAX
#elif defined SUMOREAL4SUMOTIME
typedef SUMOReal SUMOTime;
#define SUMOTime_MAX SUMOREAL_MAX
#endif

/*   Believe it or not, duarouter does not behave correctly SUMOTIME_MAXSTRING
 * being set to 2147483648 (which is 32 bitted MAX_INT)
 */
#define SUMOTIME_MAXSTRING "2147483" // INT_MAX / 1000
//#define SUMOTIME_MAXSTRING "2147483648" // INT_MAX / 1000

#ifndef HAVE_SUBSECOND_TIMESTEPS

// the step length in s
#if defined INT4SUMOTIME
#define DELTA_T 100 // DANGER
//#define DELTA_T 1 // DANGER (IS THIS THE ORIGINAL?)
//#define DELTA_T .1 // DANGER
#elif defined SUMOREAL4SUMOTIME
#define DELTA_T .1
#endif

#define TS (static_cast<SUMOReal>(1.))

// x*deltaT
#define SPEED2DIST(x) (x)
// x/deltaT
#define DIST2SPEED(x) (x)
// x*deltaT*deltaT
#define ACCEL2DIST(x) (x)
// x*deltaT
#define ACCEL2SPEED(x) (x)
// x/deltaT
#define SPEED2ACCEL(x) (x)

#define STEPS2TIME(x) (static_cast<SUMOReal>(x))
#define TIME2STEPS(x) (static_cast<SUMOTime>(x))
#define STEPFLOOR(x) (x)

#else

// the step length in ms
extern SUMOTime DELTA_T;

// the step length in seconds as SUMOReal
#define TS (static_cast<SUMOReal>(DELTA_T/1000.))

// x*deltaT
#define SPEED2DIST(x) ((x)*TS)
// x/deltaT
#define DIST2SPEED(x) ((x)/TS)
// x*deltaT*deltaT
#define ACCEL2DIST(x) ((x)*TS*TS)
// x*deltaT
#define ACCEL2SPEED(x) ((x)*TS)
// x*deltaT
#define SPEED2ACCEL(x) ((x)/TS)

#define STEPS2TIME(x) (static_cast<SUMOReal>((x)/1000.))
#define TIME2STEPS(x) (static_cast<SUMOTime>((x)*1000))
#define STEPFLOOR(x) (int(x/DELTA_T)*DELTA_T)

#endif

// ===========================================================================
// method declarations
// ===========================================================================
SUMOTime string2time(const std::string& r) throw(EmptyData, NumberFormatException, ProcessError);
std::string time2string(SUMOTime t) ;

#endif