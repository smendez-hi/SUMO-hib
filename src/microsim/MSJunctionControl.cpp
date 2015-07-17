/****************************************************************************/
/// @file    MSJunctionControl.cpp
/// @author  Christian Roessel
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Tue, 06 Mar 2001
/// @version $Id: MSJunctionControl.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// Container for junctions; performs operations on all stored junctions
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

#include "MSJunctionControl.h"

#include <algorithm>
#include "MSEdgeControl.h"
#include "MSJunction.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS

// ===========================================================================
// member method definitions
// ===========================================================================
MSJunctionControl::MSJunctionControl(){}

MSJunctionControl::~MSJunctionControl(){}

void MSJunctionControl::postloadInitContainer()
{
    const std::vector<MSJunction*> &junctions = buildAndGetStaticVector();
    for (std::vector<MSJunction*>::const_iterator i = junctions.begin(); i != junctions.end(); ++i) {
        (*i)->postloadInit();
    }
}

void MSJunctionControl::patchHeights(MSEdgeControl *edges)
{
  /* TODO */
  ;
  /*
  SUMOReal heightDiff, opp, adj;
  MSEdge *edge;
  for(int i = 0; i < getEdges().size(); i++)
  {
    edge = getEdges().at(i);
    heightDiff = abs(junctions->get(edge->getJunctionFrom())->getHeight(),
                     junctions->get(edge->getJunctionTo())->getHeight());
    opp = heightDiff;
    adj = edge->getLength();
    junction->setHeight(edge->getLength()*tan(edge->getSlope()) + 
      otherJunction->getHeight());
  }
  */
}