/****************************************************************************/
/// @file    NBJunctionTypesMatrix.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Sascha Krieg
/// @date    Tue, 20 Nov 2001
/// @version $Id: NBJunctionTypesMatrix.h 11671 2012-01-07 20:14:30Z behrisch $
///
// Definition of a junction's type in dependence
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
#ifndef NBJunctionTypesMatrix_h
#define NBJunctionTypesMatrix_h


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
#include "NBNode.h"


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class NBJunctionTypesMatrix
 * A class that stores the relationship between incoming edges and the
 * junction type resulting from their types.
 */
class NBJunctionTypesMatrix {
public:
    /** constructor */
    NBJunctionTypesMatrix();

    /** destructor */
    ~NBJunctionTypesMatrix();

    /** returns the type of the junction on the crossing of edges of the
        given types */
    SumoXMLNodeType getType(SUMOReal speed1, SUMOReal speed2) const;

private:
    /** returns the one-char name of the junction type between the two
        given ranges */
    char getNameAt(size_t pos1, size_t pos2) const;

    /**
     * priority_finder
     * Searches for the named priority in the range container
     */
    class range_finder {
    public:
        /** constructor */
        explicit range_finder(SUMOReal speed) : mySpeed(speed) { }

        /** the comparing function */
        bool operator()(const std::pair<SUMOReal, SUMOReal> &range) {
            return mySpeed >= range.first && mySpeed < range.second;
        }

    private:
        SUMOReal mySpeed;
    };

private:
    /** a container type for edge priority ranges */
    typedef std::vector<std::pair<SUMOReal, SUMOReal> > RangeCont;

    /** A container type for the resulting junction types (cross matrix) */
    typedef std::vector<std::string> StringCont;

    /** a map of chars to ints */
    typedef std::map<char, SumoXMLNodeType> CharToVal;

    /** a container for edge priority ranges */
    RangeCont   myRanges;

    /** A container for the resulting junction types (cross matrix)
    The informations are stored as chars:
    't': Traffic Light Junction
    'r': Right-before-Left Junction
    'p': Priority Junction
    'x': no Junction */
    StringCont  myValues;

    /** a map of chars to their NBNode-representation */
    CharToVal   myMap;

};


#endif

/****************************************************************************/

