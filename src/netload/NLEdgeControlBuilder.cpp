/****************************************************************************/
/// @file    NLEdgeControlBuilder.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Mon, 9 Jul 2001
/// @version $Id: NLEdgeControlBuilder.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// Interface for building edges
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
#include <iterator>
#include <microsim/MSLane.h>
#include <microsim/MSInternalLane.h>
#include <microsim/MSEdge.h>
#include <microsim/MSEdgeControl.h>
#include <utils/common/StringTokenizer.h>
#include <utils/common/UtilExceptions.h>
#include "NLBuilder.h"
#include "NLEdgeControlBuilder.h"
#include <utils/options/OptionsCont.h>
#include <utils/iodevices/OutputDevice.h>

#ifdef HAVE_MESOSIM
#include <mesosim/MELoop.h>
#endif

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS

// ===========================================================================
// method definitions
// ===========================================================================
NLEdgeControlBuilder::NLEdgeControlBuilder()
 :myCurrentNumericalLaneID(0), myCurrentNumericalEdgeID(0), myEdges(0)
{
  myActiveEdge = (MSEdge*) 0;
  myLaneStorage = new std::vector<MSLane*>();
}

NLEdgeControlBuilder::~NLEdgeControlBuilder
 ()
{
  delete myLaneStorage;
}

void NLEdgeControlBuilder::beginEdgeParsing
 (const std::string& id, MSEdge::EdgeBasicFunction function,
  const std::string& streetName)
{
  OptionsCont &oc = OptionsCont::getOptions();
    if(oc.getBuildVerbosity() > 1)
      printf("----> void NLEdgeControlBuilder::beginEdgeParsing(...)\n");
    myActiveEdge = buildEdge(id, streetName);
    if (MSEdge::dictionary(id) != 0) {
        throw InvalidArgument("Another edge with the id '" + id + "' exists.");
    }
    myEdges.push_back(myActiveEdge);
    myFunction = function;
}

MSLane* NLEdgeControlBuilder::addLane
 (const std::string& id, SUMOReal maxSpeed, SUMOReal length,
  const PositionVector& shape, SUMOReal width,
  const SUMOVehicleClasses& allowed, const SUMOVehicleClasses& disallowed,
  SUMOReal slope)
{
  OptionsCont &oc = OptionsCont::getOptions();
    if(oc.getBuildVerbosity() > 1)
      printf("----> MSLane* NLEdgeControlBuilder::addLane(...)\n");
    MSLane* lane = 0;
    switch (myFunction) {
        case MSEdge::EDGEFUNCTION_INTERNAL:
            lane = new MSInternalLane(id, maxSpeed, length, myActiveEdge,
                                      myCurrentNumericalLaneID++, shape, width,
                                      allowed, disallowed, slope);
            break;
        case MSEdge::EDGEFUNCTION_NORMAL:
        case MSEdge::EDGEFUNCTION_CONNECTOR:
            lane = new MSLane(id, maxSpeed, length, myActiveEdge,
                              myCurrentNumericalLaneID++, shape, width, allowed,
                              disallowed, slope);
            break;
        default:
            throw InvalidArgument("Unrecognised edge type.");
    }
    myLaneStorage->push_back(lane);
    return lane;
}


MSEdge* NLEdgeControlBuilder::closeEdge()
{
  OptionsCont &oc = OptionsCont::getOptions();
    if(oc.getBuildVerbosity() > 1)
      printf("----> MSEdge* NLEdgeControlBuilder::closeEdge()\n");    
    std::vector<MSLane*> *lanes = new std::vector<MSLane*>();
    lanes->reserve(myLaneStorage->size());
    copy(myLaneStorage->begin(), myLaneStorage->end(), back_inserter(*lanes));
    myLaneStorage->clear();
    myActiveEdge->initialize(lanes, myFunction);
    return myActiveEdge;
}


MSEdgeControl* NLEdgeControlBuilder::build()
{
  OptionsCont &oc = OptionsCont::getOptions();
  if(oc.getBuildVerbosity() > 1)
    std::cout<<"----> MSEdgeControl* NLEdgeControlBuilder::build()\n"<<std::endl;
  for(EdgeCont::iterator i1 = myEdges.begin(); i1 != myEdges.end(); i1++)
  {
    if(oc.getBuildVerbosity() > 2)
      std::cout<<"[500] MSEdgeControl* NLEdgeControlBuilder::build()\n"<<std::endl;
    (*i1)->closeBuilding();
#ifdef HAVE_MESOSIM
    if(MSGlobals::gUseMesoSim)
    {
      MSGlobals::gMesoNet->buildSegmentsFor(**i1, OptionsCont::getOptions());
    }
#endif
  }
  if(oc.getBuildVerbosity() > 1)
    std::cout<<"<---- MSEdgeControl* NLEdgeControlBuilder::build()\n"<<std::endl;
  return new MSEdgeControl(myEdges);
}


MSEdge* NLEdgeControlBuilder::buildEdge
 (const std::string& id, const std::string& streetName)
{
  OptionsCont &oc = OptionsCont::getOptions();
    if(oc.getBuildVerbosity() > 1)
      std::cout<<"----> MSEdge* NLEdgeControlBuilder::buildEdge(...)\n"<<std::endl;
    return new MSEdge(id, myCurrentNumericalEdgeID++, streetName);
}