/****************************************************************************/
/// @file    RORDLoader_SUMOBase.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id: RORDLoader_SUMOBase.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// The base class for SUMO-native route handlers
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

#include "RORDLoader_SUMOBase.h"
#include <utils/common/SUMOVTypeParameter.h>
#include "RORouteDef.h"
#include "RONet.h"
#include <utils/common/UtilExceptions.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/StringTokenizer.h>
#include <utils/common/ToString.h>
#include "ROVehicle.h"
#include "RORouteDef_Alternatives.h"
#include "RORouteDef_Complete.h"
#include "RORoute.h"
#include <utils/xml/SUMOVehicleParserHelper.h>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS

// ===========================================================================
// method definitions
// ===========================================================================
RORDLoader_SUMOBase::RORDLoader_SUMOBase
 (RONet& net, SUMOTime begin, SUMOTime end, const SUMOReal beta,
  const SUMOReal gawronA, const SUMOReal logitGamma, const int maxRouteNumber,
  const bool tryRepair, const bool withTaz, const bool keepRoutes,
  const bool skipRouteCalculation, const std::string& file)
 :ROTypedXMLRoutesLoader(net, begin, end, file),
  myVehicleParameter(0), myCurrentIsOk(true), myAltIsValid(true),
  myHaveNextRoute(false), myCurrentAlternatives(0), myBeta(beta),
  myGawronA(gawronA), myLogitGamma(logitGamma),
  myMaxRouteNumber(maxRouteNumber), myCurrentRoute(0), myCurrentDepart(-1),
  myTryRepair(tryRepair), myWithTaz(withTaz), myKeepRoutes(keepRoutes),
  mySkipRouteCalculation(skipRouteCalculation), myColor(0), myCurrentVType(0),
  myHaveWarnedAboutDeprecatedVType(false),
  myHaveWarnedAboutDeprecatedRoute(false)
{}

RORDLoader_SUMOBase::~RORDLoader_SUMOBase()
{
  // clean up (on failure)
  delete myCurrentAlternatives;
  delete myCurrentRoute;
  delete myVehicleParameter;
  delete myColor;
}

void RORDLoader_SUMOBase::myStartElement
 (int element, const SUMOSAXAttributes& attrs)
{
  switch(element)
  {
   case SUMO_TAG_ROUTE:
    startRoute(attrs);
    break;
   case SUMO_TAG_VEHICLE:
    // try to parse the vehicle definition
    delete myVehicleParameter;
    myVehicleParameter = 0;
    myVehicleParameter =
     SUMOVehicleParserHelper::parseVehicleAttributes(attrs);
    if(myVehicleParameter != 0)
      myCurrentDepart = myVehicleParameter->depart;
    myCurrentIsOk = myVehicleParameter != 0;
    break;
   case SUMO_TAG_VTYPE__DEPRECATED:
    if(!myHaveWarnedAboutDeprecatedVType)
    {
      myHaveWarnedAboutDeprecatedVType = true;
      WRITE_WARNING("'" + toString(SUMO_TAG_VTYPE__DEPRECATED) +
       "' is deprecated; please use '" + toString(SUMO_TAG_VTYPE) + "'.");
    }
   case SUMO_TAG_VTYPE:
    myCurrentVType = SUMOVehicleParserHelper::beginVTypeParsing(attrs);
    break;
   case SUMO_TAG_ROUTE_DISTRIBUTION:
    myAltIsValid = true;
    startAlternative(attrs);
    if(!myCurrentIsOk)
      myAltIsValid = false;
    break;
   default:
    break;
  }
  // parse embedded vtype information
  if (myCurrentVType != 0 && element != SUMO_TAG_VTYPE && element != SUMO_TAG_VTYPE__DEPRECATED) {
    SUMOVehicleParserHelper::parseVTypeEmbedded(*myCurrentVType, element,
     attrs);
    return;
  }
  if(!myCurrentIsOk)
    throw ProcessError();
}

void RORDLoader_SUMOBase::startRoute(const SUMOSAXAttributes& attrs)
{
    delete myColor;
    myColor = 0;
    if (!myAltIsValid) {
        return;
    }
    if (myCurrentAlternatives == 0) {
        myCurrentIsOk = true;
        if (myVehicleParameter != 0) {
            if (attrs.hasAttribute(SUMO_ATTR_ID)) {
                WRITE_ERROR("Internal routes do not have an id (vehicle '" + myVehicleParameter->id + "').");
                myCurrentIsOk = false;
                return;
            }
            myCurrentRouteName = "!" + myVehicleParameter->id;
        } else {
            myCurrentRouteName = attrs.getStringReporting(SUMO_ATTR_ID, 0, myCurrentIsOk);
        }
    } else {
        // parse route alternative...
        myCost = attrs.getOptSUMORealReporting(SUMO_ATTR_COST, myCurrentAlternatives->getID().c_str(), myCurrentIsOk, -1);
        myProbability = attrs.getSUMORealReporting(SUMO_ATTR_PROB, myCurrentAlternatives->getID().c_str(), myCurrentIsOk);
        if (myCurrentIsOk && myCost < 0 && myCost != -1) {
            WRITE_ERROR("Invalid cost in alternative for route '" + myCurrentAlternatives->getID() + "' (" + toString<SUMOReal>(myCost) + ").");
            myCurrentIsOk = false;
            return;
        }
        if (myCurrentIsOk && myProbability < 0) {
            WRITE_ERROR("Invalid probability in alternative for route '" + myCurrentAlternatives->getID() + "' (" + toString<SUMOReal>(myProbability) + ").");
            myCurrentIsOk = false;
            return;
        }
    }
    if (attrs.hasAttribute(SUMO_ATTR_COLOR)) {
        myColor = new RGBColor(RGBColor::parseColorReporting(
                                   attrs.getString(SUMO_ATTR_COLOR),
                                   attrs.getObjectType(), myCurrentRouteName.c_str(), true, myCurrentIsOk));
    }
    if (attrs.hasAttribute(SUMO_ATTR_EDGES)) {
        myCharacters(SUMO_TAG_ROUTE, attrs.getStringReporting(SUMO_ATTR_EDGES, myCurrentRouteName.c_str(), myCurrentIsOk));
    } else {
        if (!myHaveWarnedAboutDeprecatedRoute) {
            WRITE_WARNING("Defining routes as a nested string is deprecated, use the edges attribute instead.");
            myHaveWarnedAboutDeprecatedRoute = true;
        }
    }
}


void
RORDLoader_SUMOBase::startAlternative(const SUMOSAXAttributes& attrs) {
    // try to get the id
    myCurrentIsOk = true;
    std::string id;
    if (myVehicleParameter != 0) {
        id = myVehicleParameter->id;
        if (id == "") {
            WRITE_ERROR("Missing 'id' of a routeDistribution.");
            myCurrentIsOk = false;
            return;
        }
        id = "!" + id;
    } else {
        id = attrs.getStringReporting(SUMO_ATTR_ID, 0, myCurrentIsOk);
        if (!myCurrentIsOk) {
            return;
        }
    }
    // try to get the index of the last element
    int index = attrs.getIntReporting(SUMO_ATTR_LAST, id.c_str(), myCurrentIsOk);
    if (myCurrentIsOk && index < 0) {
        WRITE_ERROR("Negative index of a route alternative (id='" + id + "').");
        myCurrentIsOk = false;
        return;
    }
    // build the alternative cont
    myCurrentAlternatives = new RORouteDef_Alternatives(id, index, myBeta, myGawronA, myLogitGamma,
            myMaxRouteNumber, myKeepRoutes, mySkipRouteCalculation);
}

void
RORDLoader_SUMOBase::myCharacters(int element,
                                  const std::string& chars) {
    // process routes only, all other elements do
    //  not have embedded characters
    if (element != SUMO_TAG_ROUTE) {
        return;
    }
    if (!myAltIsValid) {
        return;
    }
    if (myCurrentRoute != 0) {
        return;
    }
    // check whether the costs and the probability are valid
    if (myCurrentAlternatives != 0 && !myCurrentIsOk) {
        return;
    }
    // build the list of edges
    std::vector<const ROEdge*> *list = new std::vector<const ROEdge*>();
    if (myWithTaz && myVehicleParameter->wasSet(VEHPARS_TAZ_SET)) {
        ROEdge* edge = myNet.getEdge(myVehicleParameter->fromTaz + "-source");
        if (edge != 0) {
            list->push_back(edge);
        } else {
            WRITE_ERROR("The vehicle '" + myVehicleParameter->id + "' contains the unknown zone '" + myVehicleParameter->fromTaz + "'.");
            myCurrentIsOk = false;
        }
    }
    StringTokenizer st(chars);
    while (myCurrentIsOk && st.hasNext()) { // !!! too slow !!!
        const std::string id = st.next();
        ROEdge* edge = myNet.getEdge(id);
        if (edge != 0) {
            list->push_back(edge);
        } else {
            if (!myTryRepair) {
                std::string rid = myCurrentAlternatives != 0 ? myCurrentAlternatives->getID() : myCurrentRouteName;
                WRITE_ERROR("The route '" + rid + "' contains the unknown edge '" + id + "'.");
                myCurrentIsOk = false;
            }
        }
    }
    if (myWithTaz && myVehicleParameter->wasSet(VEHPARS_TAZ_SET)) {
        ROEdge* edge = myNet.getEdge(myVehicleParameter->toTaz + "-sink");
        if (edge != 0) {
            list->push_back(edge);
        } else {
            WRITE_ERROR("The vehicle '" + myVehicleParameter->id + "' contains the unknown zone '" + myVehicleParameter->toTaz + "'.");
            myCurrentIsOk = false;
        }
    }
    if (myCurrentIsOk) {
        if (myCurrentAlternatives != 0) {
            myCurrentAlternatives->addLoadedAlternative(
                new RORoute(myCurrentAlternatives->getID(), myCost, myProbability, *list, myColor));
        } else {
            myCurrentRoute = new RORouteDef_Complete(myCurrentRouteName, myColor, *list, myTryRepair);
        }
        myColor = 0;
    }
    delete list;
}


void
RORDLoader_SUMOBase::myEndElement(int element) {
    switch (element) {
        case SUMO_TAG_ROUTE:
            if (!myAltIsValid) {
                return;
            }
            if (myCurrentRoute != 0 && myCurrentIsOk) {
                if (myCurrentAlternatives == 0) {
                    myNet.addRouteDef(myCurrentRoute);
                    myCurrentRoute = 0;
                }
                if (myVehicleParameter == 0) {
                    myHaveNextRoute = true;
                }
                myCurrentRoute = 0;
            }
            break;
        case SUMO_TAG_ROUTE_DISTRIBUTION:
            if (!myCurrentIsOk) {
                return;
            }
            if (myVehicleParameter == 0) {
                myHaveNextRoute = true;
            }
            myNet.addRouteDef(myCurrentAlternatives);
            myCurrentRoute = 0;
            myCurrentAlternatives = 0;
            break;
        case SUMO_TAG_VEHICLE:
            closeVehicle();
            delete myVehicleParameter;
            myVehicleParameter = 0;
            myHaveNextRoute = true;
            break;
        case SUMO_TAG_VTYPE__DEPRECATED:
        case SUMO_TAG_VTYPE: {
            SUMOVehicleParserHelper::closeVTypeParsing(*myCurrentVType);
            myNet.addVehicleType(myCurrentVType);
            myCurrentVType = 0;
        }
        default:
            break;
    }
    if (!myCurrentIsOk) {
        throw ProcessError();
    }
}


bool
RORDLoader_SUMOBase::closeVehicle() {
    // get the vehicle id
    if (myVehicleParameter->depart < myBegin || myVehicleParameter->depart >= myEnd) {
        myCurrentIsOk = false;
        return false;
    }
    // get vehicle type
    SUMOVTypeParameter* type = myNet.getVehicleTypeSecure(myVehicleParameter->vtypeid);
    // get the route
    RORouteDef* route = myNet.getRouteDef(myVehicleParameter->routeid);
    if (route == 0) {
        route = myNet.getRouteDef("!" + myVehicleParameter->id);
    }
    if (route == 0) {
        WRITE_ERROR("The route of the vehicle '" + myVehicleParameter->id + "' is not known.");
        myCurrentIsOk = false;
        return false;
    }
    // build the vehicle
    if (!MsgHandler::getErrorInstance()->wasInformed()) {
        ROVehicle* veh = new ROVehicle(*myVehicleParameter, route, type);
        myNet.addVehicle(myVehicleParameter->id, veh);
        return true;
    }
    return false;
}

void RORDLoader_SUMOBase::beginNextRoute()
{
  myHaveNextRoute = false;
}
