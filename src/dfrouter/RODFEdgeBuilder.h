/****************************************************************************/
/// @file    RODFEdgeBuilder.h
/// @author  Daniel Krajzewicz
/// @author  Eric Nicolay
/// @author  Michael Behrisch
/// @date    Thu, 16.03.2006
/// @version $Id: RODFEdgeBuilder.h 11671 2012-01-07 20:14:30Z behrisch $
///
// Interface for building instances of dfrouter-edges
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
#ifndef RODFEdgeBuilder_h
#define RODFEdgeBuilder_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <router/ROAbstractEdgeBuilder.h>


// ===========================================================================
// class declarations
// ===========================================================================
class ROEdge;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class RODFEdgeBuilder
 * @brief Interface for building instances of dfrouter-edges
 *
 * This ROAbstractEdgeBuilder implementation builds edges for the duarouter
 *  (instances of RODFEdge).
 *
 * @see RODFEdge
 */
class RODFEdgeBuilder : public ROAbstractEdgeBuilder {
public:
    /// @brief Constructor
    RODFEdgeBuilder() ;


    /// @brief Destructor
    ~RODFEdgeBuilder() ;


    /// @name Methods to be implemented, inherited from ROAbstractEdgeBuilder
    /// @{

    /** @brief Builds an edge with the given name
     *
     * This implementation builds a RODFEdge.
     *
     * @param[in] name The name of the edge
     * @param[in] from The node the edge begins at
     * @param[in] to The node the edge ends at
     * @param[in] slope The slope of that edge
     * @return A proper instance of the named edge
     * @see RODFEdge
     */
    ROEdge* buildEdge(const std::string& name, RONode* from, RONode* to,
                      SUMOReal slope) ;
    /// @}


};


#endif

/****************************************************************************/

