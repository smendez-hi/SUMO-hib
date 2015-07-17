/****************************************************************************/
/// @file    NLSucceedingLaneBuilder.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @date    Mon, 22 Oct 2001
/// @version $Id: NLSucceedingLaneBuilder.h 11671 2012-01-07 20:14:30Z behrisch $
///
// Temporary storage for a lanes succeeding lanes while parsing them
// This class is only used when parsing legacy networks using tags succ/succlane
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
#ifndef NLSucceedingLaneBuilder_h
#define NLSucceedingLaneBuilder_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <microsim/MSLinkCont.h>


// ===========================================================================
// class declarations
// ===========================================================================
class MSJunction;
class NLJunctionControlBuilder;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class NLSucceedingLaneBuilder
 * @brief Temporary storage for a lanes succeeding lanes while parsing them
 * @todo Consider moving this functionality to another class
 */
class NLSucceedingLaneBuilder {
public:
    /** @brief Constructor
     *
     * @param[in] jb The junction control builder to obtain referenced tls from
     */
    NLSucceedingLaneBuilder(NLJunctionControlBuilder& jb) ;


    /// @brief Destructor
    ~NLSucceedingLaneBuilder() ;


    /** @brief Begins the computation of a container holding the succeeding lanes of a lane
     *
     * @param[in] laneId The id of the lane from which connections will be built
     * @todo Why is the ID kept, not already the lane itself?
     */
    void openSuccLane(const std::string& laneId) ;


    /** @brief Adds a succeeding lane
     *
     * If either the current, or the succeeding lane is not known, an InvalidArgument
     *  is thrown (with the proper message). Also if a tls-logic is referenced, but
     *  not known to the junction control builder, an InvalidArgument is thrown.
     *
     * @param[in] laneId ID of the lane to add a connection to (from the last opened lane)
     * @param[in] viaID The junction-internal lane to use to reach the destination lane
     * @param[in] pass Theoretically: the maximum filling rate at the internal link; unused currently
     * @param[in] dir The abstract direction of the link
     * @param[in] state The abstract state of the link
     * @param[in] tlid (optional) ID of the tls that controls the link
     * @param[in] linkNo (optional) index of the link within the controlling tls
     * @todo Recheck usage of "pass"
     * @exception InvalidArgument If one of the referenced structures was not found or is invalid
     * @see LinkDirection
     * @see LinkState
     * @see MSLink
     */
    void addSuccLane(const std::string& laneId,
#ifdef HAVE_INTERNAL_LANES
                     const std::string& viaID, SUMOReal pass,
#endif
                     LinkDirection dir, LinkState state,
                     const std::string& tlid = "", unsigned int linkNo = 0) throw(InvalidArgument);


    /** @brief Ends the computation of a container holding the succeeding lanes of a lane
     *
     * The current lane is determined and the parsed connections are added to it.
     * If the lane is not known, an InvalidArgument is thrown.
     *
     * @exception InvalidArgument If the current lane is not known
     */
    void closeSuccLane() throw(InvalidArgument);


    /** @brief Returns the name of the lane the succeeding lanes are added to
     * @return The ID of the currently opened lane
     */
    const std::string& getCurrentLaneName() const ;


private:
    /// @brief ID of the lane the succeeding lanes are added to
    /// @todo Why is not the lane itself saved?
    std::string myCurrentLane;

    /// @brief The list of connections
    /// @todo Is it really necessary to have this as a pointer - the link container is rebuilt anyway
    MSLinkCont* mySuccLanes;

    /// @brief The junction control builder to obtain referenced tls from
    NLJunctionControlBuilder& myJunctionControlBuilder;


private:
    /** @brief invalid copy constructor */
    NLSucceedingLaneBuilder(const NLSucceedingLaneBuilder& s);

    /** @brief invalid assignment operator */
    NLSucceedingLaneBuilder& operator=(const NLSucceedingLaneBuilder& s);


};


#endif

/****************************************************************************/

