/****************************************************************************/
/// @file    TraCIServerAPI_Edge.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    07.05.2009
/// @version $Id: TraCIServerAPI_Edge.h 11671 2012-01-07 20:14:30Z behrisch $
///
// APIs for getting/setting edge values via TraCI
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
#ifndef TraCIServerAPI_Edge_h
#define TraCIServerAPI_Edge_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "TraCIException.h"
#include "TraCIServer.h"
#include <foreign/tcpip/storage.h>


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class TraCIServerAPI_Edge
 * @brief APIs for getting/setting edge values via TraCI
 */
class TraCIServerAPI_Edge {
public:
    /** @brief Processes a get value command (Command 0xaa: Get Edge Variable)
     *
     * @param[in] server The TraCI-server-instance which schedules this request
     * @param[in] inputStorage The storage to read the command from
     * @param[out] outputStorage The storage to write the result to
     */
    static bool processGet(traci::TraCIServer& server, tcpip::Storage& inputStorage,
                           tcpip::Storage& outputStorage);


    /** @brief Processes a set value command (Command 0xca: Change Edge State)
     *
     * @param[in] server The TraCI-server-instance which schedules this request
     * @param[in] inputStorage The storage to read the command from
     * @param[out] outputStorage The storage to write the result to
     */
    static bool processSet(traci::TraCIServer& server, tcpip::Storage& inputStorage,
                           tcpip::Storage& outputStorage);


private:
    /// @brief invalidated copy constructor
    TraCIServerAPI_Edge(const TraCIServerAPI_Edge& s);

    /// @brief invalidated assignment operator
    TraCIServerAPI_Edge& operator=(const TraCIServerAPI_Edge& s);


};


#endif

/****************************************************************************/

