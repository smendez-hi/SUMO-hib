/****************************************************************************/
/// @file    GUIEdgeControlBuilder.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id: GUIEdgeControlBuilder.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// Derivation of NLEdgeControlBuilder which build gui-edges
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

#include <vector>
#include <string>
#include <map>
#include <algorithm>
#include <guisim/GUIEdge.h>
#include <guisim/GUINet.h>
#include <guisim/GUILane.h>
#include <guisim/GUIInternalLane.h>
#include <microsim/MSJunction.h>
#include <netload/NLBuilder.h>
#include "GUIEdgeControlBuilder.h"
#include <gui/GUIGlobals.h>
#include <utils/options/OptionsCont.h>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS

// ===========================================================================
// method definitions
// ===========================================================================
GUIEdgeControlBuilder::GUIEdgeControlBuilder
 () 
 :NLEdgeControlBuilder(),oc(OptionsCont::getOptions())
{}

GUIEdgeControlBuilder::~GUIEdgeControlBuilder
 ()
{}

MSEdge* GUIEdgeControlBuilder::closeEdge()
{
  if(oc.getAnyVerbosity()>1)
    std::cout<<"----> MSEdge* GUIEdgeControlBuilder::closeEdge()\n"<<std::endl;
  MSEdge* ret = NLEdgeControlBuilder::closeEdge();
  static_cast<GUIEdge*>(ret)->initGeometry();
  return ret;
}

MSLane* GUIEdgeControlBuilder::addLane
 (const std::string& id, SUMOReal maxSpeed, SUMOReal length,
  const PositionVector& shape, SUMOReal width,
  const SUMOVehicleClasses& allowed,
  const SUMOVehicleClasses& disallowed, SUMOReal slope)
{
  if(oc.getAnyVerbosity()>1)
    std::cout<<"----> MSLane* GUIEdgeControlBuilder::addLane(...)\n"<<std::endl;
    MSLane* lane = 0;
    switch (myFunction) {
        case MSEdge::EDGEFUNCTION_INTERNAL:
            lane = new GUIInternalLane(id, maxSpeed, length, myActiveEdge,
                                       myCurrentNumericalLaneID++, shape, width,
                                       allowed, disallowed/*, slope*/);
            break;
        case MSEdge::EDGEFUNCTION_NORMAL:
        case MSEdge::EDGEFUNCTION_CONNECTOR:
            lane = new GUILane(id, maxSpeed, length, myActiveEdge,
                               myCurrentNumericalLaneID++, shape, width,
                               allowed, disallowed/*, slope*/);
            break;
        default:
            throw InvalidArgument("A lane with an unknown type occured ("+
                                  toString(myFunction) + ")");
    }
    myLaneStorage->push_back(lane);
    return lane;
}

MSEdge* GUIEdgeControlBuilder::buildEdge
 (const std::string& id, const std::string& streetName)
{
  if(oc.getAnyVerbosity()>1)
    std::cout<<"----> MSEdge* GUIEdgeControlBuilder::buildEdge(...)\n"<<std::endl;
  return new GUIEdge(id, myCurrentNumericalEdgeID++, streetName);
}
