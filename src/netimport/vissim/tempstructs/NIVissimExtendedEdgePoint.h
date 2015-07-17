/****************************************************************************/
/// @file    NIVissimExtendedEdgePoint.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id: NIVissimExtendedEdgePoint.h 11671 2012-01-07 20:14:30Z behrisch $
///
// -------------------
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
#ifndef NIVissimExtendedEdgePoint_h
#define NIVissimExtendedEdgePoint_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <utils/common/VectorHelper.h>
#include <utils/geom/Position.h>


// ===========================================================================
// class declarations
// ===========================================================================
class NBEdge;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 *
 */
class NIVissimExtendedEdgePoint {
public:
    /** @brief Constructor
     * @param[in] edgeid The id of the Vissim-edge
     * @param[in] lanes Lanes on which this point lies
     * @param[in] position The position of this point at the edge
     * @param[in] assignedVehicles Vehicle (type) indices which should be regarded by this point
     */
    NIVissimExtendedEdgePoint(int edgeid, const IntVector& lanes,
                              SUMOReal position, const IntVector& assignedVehicles) ;
    ~NIVissimExtendedEdgePoint();
    int getEdgeID() const;
    SUMOReal getPosition() const;
    Position getGeomPosition() const;
    const IntVector& getLanes() const;


    /** @brief Resets lane numbers if all lanes shall be used
     *
     * If myLanes contains a -1, the content of myLanes is replaced
     *  by indices of all lanes of the given edge.
     *
     * @param[in] The built edge
     */
    void recheckLanes(const NBEdge* const edge) ;

private:
    int myEdgeID;
    IntVector myLanes;
    SUMOReal myPosition;
    IntVector myAssignedVehicles;
};


#endif

/****************************************************************************/

