/****************************************************************************/
/// @file    MSRouteLoader.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Wed, 6 Nov 2002
/// @version $Id: MSRouteLoader.h 11671 2012-01-07 20:14:30Z behrisch $
///
// A class that performs the loading of routes
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
#ifndef MSRouteLoader_h
#define MSRouteLoader_h

// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <xercesc/sax2/XMLReaderFactory.hpp>
#include <string>
#include <microsim/MSNet.h>
#include "MSVehicleContainer.h"
#include "MSRouteHandler.h"

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSRouteLoader
 */
class MSRouteLoader
{
  public:
    /// Constructor
    MSRouteLoader(MSNet& net, MSRouteHandler* handler);

    /// Destructor
    ~MSRouteLoader();

    /**   Loads vehicles until a vehicle is read that starts after the specified 
     ** time */
    void loadUntil(SUMOTime time);

    /// Resets the reader
    void init();

    /// Returns the information whether new data is available
    bool moreAvailable() const;
    
  private:
    /// The used SAX2XMLReader
    SAX2XMLReader* myParser;

    /// The token for saving the current position
    XMLPScanToken  myToken;

    /// Information whether more vehicles should be available
    bool myMoreAvailable;

    MSRouteHandler* myHandler;
};

#endif
