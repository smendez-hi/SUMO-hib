/****************************************************************************/
/// @file    MSLogicJunction.cpp
/// @author  Christian Roessel
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Wed, 12 Dez 2001
/// @version $Id: MSLogicJunction.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// with one ore more logics.
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

#include "MSLinkCont.h"
#include "MSLogicJunction.h"
#include "MSLane.h"
#include "MSInternalLane.h"
#include "utils/options/OptionsCont.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// member method definitions
// ===========================================================================
/* -------------------------------------------------------------------------
 * methods from MSLogicJunction
 * ----------------------------------------------------------------------- */
MSLogicJunction::MSLogicJunction
 (const std::string& id, const Position& position, const PositionVector& shape,
  std::vector<MSLane*> incoming, const SUMOReal height
#ifdef HAVE_INTERNAL_LANES
  , std::vector<MSLane*> internal
#endif
 )
 :MSJunction(id, position, shape, height),
  myIncomingLanes(incoming),
#ifdef HAVE_INTERNAL_LANES
  myInternalLanes(internal),
#endif
  myInnerState(false)
{
  int i;
  if(OptionsCont::getOptions().getInt("buildVerbosity") > 1)
    std::cout<<"----> MSLogicJunction::MSLogicJunction(...)"<<std::endl;
  static_cast < MSJunction * > (this)->setDynamicClassName("MSNoLogicJunction");
  for(i=0;i<incoming.size();i++)
  {
    if(incoming.at(i)->getToJunction() == NULL)
    {
      // Already setting this in MSRightOfWayJunction...
      //incoming.at(i)->setToJunction(this);
    }
    /*
    if(incoming.at(i).getEdge()->getRightLane() == incoming.at(i))
    {
      if(incoming.at(i).getEdge()->getLeftLane().getJunctionTo() == NULL)
      {
        ;
      }
    }
    */
  }
}

MSLogicJunction::~MSLogicJunction(){}

void MSLogicJunction::postloadInit()
{
  /*
  if(getID()=="1565") {
      int bla = 0;
  }
  // inform links where they have to report approaching vehicles to
  size_t requestPos = 0;
  std::vector<MSLane*>::iterator i;
  // going through the incoming lanes...
  for(i=myIncomingLanes.begin(); i!=myIncomingLanes.end(); ++i) {
      const MSLinkCont &links = (*i)->getLinkCont();
      // ... set information for every link
      for(MSLinkCont::const_iterator j=links.begin(); j!=links.end(); j++) {
          (*j)->setRequestInformation(&myRequest, requestPos,
              &myRespond, requestPos/, clearInfo/);
          requestPos++;
      }
  }
  #ifdef HAVE_INTERNAL_LANES
  // set information for the internal lanes
  requestPos = 0;
  for(i=myInternalLanes.begin(); i!=myInternalLanes.end(); ++i) {
      // ... set information about participation
      static_cast<MSInternalLane*>(*i)->setParentJunctionInformation(
          &myInnerState, requestPos++);
  }
  #endif
  */
}