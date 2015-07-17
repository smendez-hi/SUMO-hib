/****************************************************************************/
/// @file    ROVehicle.cpp
/// @author  Daniel Krajzewicz
/// @author  Axel Wegener
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id: ROVehicle.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// A vehicle as used by router
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

#include <utils/common/TplConvert.h>
#include <utils/common/ToString.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/SUMOVTypeParameter.h>
#include <utils/options/OptionsCont.h>
#include <utils/iodevices/OutputDevice.h>
#include <string>
#include <iostream>
#include "RORouteDef.h"
#include "ROVehicle.h"
#include "RORouteDef_Alternatives.h"
#include "RORoute.h"
#include "ROHelper.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
ROVehicle::ROVehicle(const SUMOVehicleParameter& pars,
                     RORouteDef* route, SUMOVTypeParameter* type)
    : myParameter(pars), myType(type), myRoute(route) {}


ROVehicle::~ROVehicle() {}


void
ROVehicle::saveAllAsXML(SUMOAbstractRouter<ROEdge, ROVehicle> &router, OutputDevice& os,
                        OutputDevice* const altos, bool withExitTimes) const {
    // check whether the vehicle's type was saved before
    if (myType != 0 && !myType->saved) {
        // ... save if not
        myType->write(os);
        if (altos != 0) {
            myType->write(*altos);
        }
        myType->saved = true;
    }

    // write the vehicle (new style, with included routes)
    myParameter.writeAs("vehicle", os, OptionsCont::getOptions());
    if (altos != 0) {
        myParameter.writeAs("vehicle", *altos, OptionsCont::getOptions());
    }

    // check whether the route shall be saved
    if (!myRoute->isSaved()) {
        myRoute->writeXMLDefinition(router, os, this, false, withExitTimes);
        if (altos != 0) {
            myRoute->writeXMLDefinition(router, *altos, this, true, withExitTimes);
        }
    }
    os.closeTag();
    if (altos != 0) {
        altos->closeTag();
    }
}


ROVehicle*
ROVehicle::copy(const std::string& id, unsigned int depTime,
                RORouteDef* newRoute) {
    SUMOVehicleParameter pars(myParameter);
    pars.id = id;
    pars.depart = depTime;
    return new ROVehicle(pars, newRoute, myType);
}


/****************************************************************************/

