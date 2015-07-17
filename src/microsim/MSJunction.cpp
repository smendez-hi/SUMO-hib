/****************************************************************************/
/// @file    MSJunction.cpp
/// @author  Christian Roessel
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Wed, 12 Dez 2001
/// @version $Id: MSJunction.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// The base class for an intersection
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

#include "utils/common/MsgHandler.h"
#include "utils/options/OptionsCont.h"
#include "MSJunction.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS

// ===========================================================================
// class declarations
// ===========================================================================
class MSLink;

// ===========================================================================
// member method definition
// ===========================================================================
MSJunction::MSJunction
 (const std::string& id, const Position& position, const PositionVector& shape,
  const SUMOReal height)
 :myID(id), myPosition(position), myShape(shape), myHeight(height)
{
  OptionsCont &oc = OptionsCont::getOptions();
  if(oc.getBuildVerbosity() > 1)
    std::cout << "MSJunction::MSJunction(...)\n";
    /* WRITE_MESSAGE("MSJunction::MSJunction(...)"); */
  if(height > 0)
  {
    if(oc.getBuildVerbosity() > 1 /* and this should be 2 */)
      std::cout<<"MSJunction{id="<<id<<", height="<<height<<"}"<<std::endl;
  }
  if(oc.getBuildVerbosity() > 2)
  {
    std::cout<<"MSJunction:"<<std::endl;
    std::cout<<" height: "<< height << std::endl;
    std::cout<<" id: "<< id << std::endl;
  }
}

MSJunction::~MSJunction()
{
  OptionsCont &oc = OptionsCont::getOptions();
  if(oc.getBuildVerbosity() > 2)
    std::cout << "MSJunction::~MSJunction(...)\n";
    /* WRITE_MESSAGE("MSJunction::~MSJunction(...)"); */
}

const Position& MSJunction::getPosition() const { return myPosition; }

void MSJunction::postloadInit() {}

/*const std::string& MSJunction::getID() const*/
std::string& MSJunction::getID()
{
  //if(myID)
    return myID;
  //else return "";
}

const std::string MSJunction::getNonReferencedId() const
/*std::string MSJunction::getNonReferencedId()*/
{
  return myID;
}

const bool MSJunction::isInternal() const
{
#ifdef HAVE_INTERNAL_LANES
  return myID.at(0)==':';
#else
  return false;
#endif
}