/****************************************************************************/
/// @file    MSRightOfWayJunction.cpp
/// @author  Christian Roessel
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Wed, 12 Dez 2001
/// @version $Id: MSRightOfWayJunction.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// junction.
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

#include "MSRightOfWayJunction.h"
#include "MSLane.h"
#include "MSJunctionLogic.h"
#include "MSBitSetLogic.h"
#include "MSGlobals.h"
#include "MSInternalLane.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <utils/common/RandHelper.h>
#include "utils/options/OptionsCont.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS

// ===========================================================================
// method definitions
// ===========================================================================
MSRightOfWayJunction::MSRightOfWayJunction
 (const std::string& id, const Position& position, const PositionVector& shape,
  std::vector<MSLane*> incoming, const SUMOReal height,
#ifdef HAVE_INTERNAL_LANES
  std::vector<MSLane*> internal,
#endif
  MSJunctionLogic* logic)
 :MSLogicJunction(id, position, shape, incoming, height
#ifdef HAVE_INTERNAL_LANES
  , internal),
#else
  ),
#endif
  myLogic(logic)
{
  int i;
  if(OptionsCont::getOptions().getInt("buildVerbosity") > 1)
  {
    std::cout<<"----> MSRightOfWayJunction::MSRightOfWayJunction(...)"<<std::endl;
    std::cout<<"MSRightOfWayJunction:"<<std::endl;
    std::cout<<" id: "<<id<<std::endl;
  }
  static_cast < MSJunction * > (this)->setDynamicClassName("MSRightOfWayJunction");
  for(i=0;i<incoming.size();i++)
  {
    if(incoming.at(i)->getToJunction() == NULL)
    {
      //incoming.at(i)->setToJunction(this);
    }
    /*
    if(incoming.at(i)->getEdge().leftLane(incoming.at(i)) == incoming.at(i))
    {
      if(incoming.at(i)->getRightLane()->getToJunction() == NULL)
      {
        ;
      }
      else
      {
        ;
      }
    }
    */
  }
}

MSRightOfWayJunction::~MSRightOfWayJunction()
{
  delete myLogic;
}

/*   [UPREGO] This function is called from
 * MSJunctionControl::postloadInitContainer()
 */
void MSRightOfWayJunction::postloadInit()
{
  // inform links where they have to report approaching vehicles to
  unsigned int requestPos = 0;
  std::vector<MSLane*>::iterator i;
  // going through the incoming lanes...
  unsigned int maxNo = 0;
  /* [UPREGO] How sorted? */
  std::vector<std::pair<MSLane*,MSLink*> >sortedLinks;
  /* [UPREGO] Iterate with i over incoming lanes of this junction */
  for (i = myIncomingLanes.begin(); i != myIncomingLanes.end(); ++i) {
    /* [UPREGO] Links of this incoming lane? */
    const MSLinkCont& links = (*i)->getLinkCont();
    // ... set information for every link
    for(MSLinkCont::const_iterator j = links.begin(); j != links.end(); j++)
    {
      /* [UPREGO] ??? */
      if(myLogic->getLogicSize() <= requestPos)
      {
        std::cout<<"MSRightOfWayJunction.cpp:123 throwing exception!"<<std::endl;
        std::cout<<" getID(){"<<getID()<<"}"<<std::endl;
        //throw ProcessError("Found invalid logic position of a link (network error)");
        /* TODO TASK FIXME ALERT DANGER if exception commented */      
      }
      /*   [UPREGO] Create new link with:
       * - An incoming lane (i) of this junction (this) 
       * - and one link (j) of the links container of the incoming lane
       */
      sortedLinks.push_back(std::make_pair(*i, *j));
      /* [UPREGO] And note it in maxNo */
      ++maxNo;
    }
  }
  bool isCrossing = myLogic->isCrossing();
  for (i = myIncomingLanes.begin(); i != myIncomingLanes.end(); ++i) {
    const MSLinkCont& links = (*i)->getLinkCont();
    // ... set information for every link
    for(MSLinkCont::const_iterator j = links.begin(); j != links.end(); j++)
    {
      if(myLogic->getLogicSize() <= requestPos)
      {
        std::cout<<"MSRightOfWayJunction.cpp:144 throwing exception!"<<std::endl;
        std::cout<<" getID(){"<<getID()<<"}"<<std::endl;
        //throw ProcessError("Found invalid logic position of a link (network error)");
        /* TODO TASK FIXME ALERT DANGER if exception commented */                
      }
      const MSLogicJunction::LinkFoes& foeLinks = myLogic->getFoesFor(requestPos);
      const std::bitset<64> &internalFoes = myLogic->getInternalFoesFor(requestPos);
      bool cont = myLogic->getIsCont(requestPos);
      myLinkFoeLinks[*j] = std::vector<MSLink*>();
      for (unsigned int c = 0; c < maxNo; ++c) {
        if (foeLinks.test(c)) {
          myLinkFoeLinks[*j].push_back(sortedLinks[c].second);
        }
      }
      std::vector<MSLink*> foes;
      for (unsigned int c = 0; c < maxNo; ++c) {
        if (internalFoes.test(c)) {
          MSLink* foe = sortedLinks[c].second;
          foes.push_back(foe);
#ifdef HAVE_INTERNAL_LANES
          MSLane* l = foe->getViaLane();
          if (l == 0) {
            continue;
          }
          const MSLinkCont& lc = l->getLinkCont();
          for (MSLinkCont::const_iterator q = lc.begin(); q != lc.end(); ++q) {
            if ((*q)->getViaLane() != 0) {
              foes.push_back(*q);
            }
          }
#endif
        }
      }
      myLinkFoeInternalLanes[*j] = std::vector<MSLane*>();
#ifdef HAVE_INTERNAL_LANES
      if (MSGlobals::gUsingInternalLanes && myInternalLanes.size() > 0) {
        int li = 0;
        for (unsigned int c = 0; c < sortedLinks.size(); ++c) {
          if (sortedLinks[c].second->getLane() == 0) { // dead end
            continue;
          }
          if (internalFoes.test(c)) {
            myLinkFoeInternalLanes[*j].push_back(myInternalLanes[li]);
          }
          ++li;
        }
      }
#endif
      (*j)->setRequestInformation(requestPos, requestPos, isCrossing, cont, myLinkFoeLinks[*j], myLinkFoeInternalLanes[*j]);
      for (std::vector<MSLink*>::const_iterator k = foes.begin(); k != foes.end(); ++k) {
        (*j)->addBlockedLink(*k);
        (*k)->addBlockedLink(*j);
      }
      requestPos++;
    }
  }
#ifdef HAVE_INTERNAL_LANES
  // set information for the internal lanes
  requestPos = 0;
  for (i = myInternalLanes.begin(); i != myInternalLanes.end(); ++i) {
    // ... set information about participation
    static_cast<MSInternalLane*>(*i)->setParentJunctionInformation(&myInnerState, requestPos++);
  }
#endif
}