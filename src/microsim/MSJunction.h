/****************************************************************************/
/// @file    MSJunction.h
/// @author  Christian Roessel
/// @author  Daniel Krajzewicz
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @date    Wed, 12 Dez 2001
/// @version $Id: MSJunction.h 11671 2012-01-07 20:14:30Z behrisch $
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
#ifndef MSJunction_h
#define MSJunction_h

// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <vector>
#include <map>
#include <utils/geom/Position.h>
#include <utils/geom/PositionVector.h>
#include <utils/common/SUMOTime.h>
#include <utils/common/UtilExceptions.h>

class MSVehicle;
class MSLink;
class MSLane;

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSJunction
 * @brief The base class for an intersection
 */
class MSJunction
{
 public:
  /// @brief Destructor.
  virtual ~MSJunction();

  /** @brief Constructor
    * @param[in] id The id of the junction
    * @param[in] position The position of the junction
    * @param[in] shape The shape of the junction
    */
  MSJunction
   (const std::string& id, const Position& position,
    const PositionVector& shape, const SUMOReal height);

  /** performs some initialisation after the loading
      (e.g., link map computation) */
  virtual void postloadInit();

  /** returns the junction's position */
  const Position& getPosition() const;

  /// Returns the id of the junction
  /*const std::string& getID() const;*/
  std::string& getID();
  
  const std::string getNonReferencedId() const;
  /*std::string getNonReferencedId();*/

  /** @brief Returns this junction's shape
    * @return The shape of this junction
    */
  const PositionVector& getShape() const { return myShape; }
  
  // TODO Reactivate the const-correctness
  //const SUMOReal getHeight() const { return myHeight; }
  SUMOReal getHeight() { return myHeight; }

  virtual const std::vector<MSLink*> &getFoeLinks
    (const MSLink* const /*srcLink*/) const
  {
    return myEmptyLinks;
  }

  virtual const std::vector<MSLane*> &getFoeInternalLanes
    (const MSLink* const /*srcLink*/) const
  {
    return myEmptyLanes;
  }
  
  const std::string getDynamicClassName(){return myDynamicClassName;}
  
  void setDynamicClassName(std::string s){myDynamicClassName=s;}
  
  const bool isInternal() const;

 protected:
  /// @brief The id of the junction
  std::string myID;

  /// @brief The position of the junction
  Position myPosition;

  /// @brief The shape of the junction
  PositionVector myShape;

  std::vector<MSLink*> myEmptyLinks;
  std::vector<MSLane*> myEmptyLanes;
  
  SUMOReal myHeight;
  
  std::string myDynamicClassName;

 private:
  /// @brief Invalidated copy constructor.
  MSJunction(const MSJunction&);

  /// @brief Invalidated assignment operator.
  MSJunction& operator=(const MSJunction&);
};

#endif