/****************************************************************************/
/// @file    AbstractMutex.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    2005-07-12
/// @version $Id: AbstractMutex.h 11671 2012-01-07 20:14:30Z behrisch $
///
// An abstract class for encapsulating mutex implementations
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
#ifndef AbstractMutex_h
#define AbstractMutex_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class AbstractMutex
 * @brief An abstract class for encapsulating mutex implementations
 *
 * This class defines access to a mutex. The implementation may differ.
 * Within gui-applications, FXMutexes may be used while this is improper
 *  for command-line applications. Normally, they do not need mutexes unless
 *  a synchronized communication with an external application is established.
 *  In these cases, a further class should be implemented.
 */
class AbstractMutex {
public:
    /// @brief Constructor
    AbstractMutex() { }


    /// @brief Destructor
    virtual ~AbstractMutex() { }


    /// @brief Locks the mutex
    virtual void lock() = 0;


    /// @brief Unlocks the mutex
    virtual void unlock() = 0;

};


#endif

/****************************************************************************/

