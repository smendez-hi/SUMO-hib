/****************************************************************************/
/// @file    NLSucceedingLaneBuilder.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Mon, 22 Oct 2001
/// @version $Id: NLSucceedingLaneBuilder.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// Temporary storage for a lanes succeeding lanes while parsing them
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

#include <string>
#include <map>
#include <vector>
#include <iterator>
#include <microsim/MSLane.h>
#include <microsim/MSInternalLane.h>
#include <microsim/MSLink.h>
#include <microsim/MSLinkCont.h>
#include <microsim/MSGlobals.h>
#include <microsim/traffic_lights/MSTrafficLightLogic.h>
#include "NLBuilder.h"
#include "NLSucceedingLaneBuilder.h"
#include "NLJunctionControlBuilder.h"
#include <utils/options/OptionsCont.h>
#include <utils/common/UtilExceptions.h>
#include <utils/geom/GeomHelper.h>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
NLSucceedingLaneBuilder::NLSucceedingLaneBuilder(NLJunctionControlBuilder& jb)
    : myJunctionControlBuilder(jb) {
    mySuccLanes = new MSLinkCont();
    mySuccLanes->reserve(10);
}


NLSucceedingLaneBuilder::~NLSucceedingLaneBuilder() {
    delete mySuccLanes;
}


void
NLSucceedingLaneBuilder::openSuccLane(const std::string& laneId) {
    myCurrentLane = laneId;
}


void
NLSucceedingLaneBuilder::addSuccLane(const std::string& laneId,
#ifdef HAVE_INTERNAL_LANES
                                     const std::string& viaID,
                                     SUMOReal pass,
#endif
                                     LinkDirection dir,
                                     LinkState state,
                                     const std::string& tlid, unsigned int linkNo) throw(InvalidArgument) {
    // check whether the link is a dead link
    if (laneId == "SUMO_NO_DESTINATION") {
        // build the dead link and add it to the container
#ifdef HAVE_INTERNAL_LANES
        MSLink* link = new MSLink(0, 0, LINKDIR_NODIR, LINKSTATE_DEADEND, 0.);
#else
        MSLink* link = new MSLink(0, LINKDIR_NODIR, LINKSTATE_DEADEND, 0.);
#endif
        mySuccLanes->push_back(link);
        if (tlid != "") {
            MSTLLogicControl::TLSLogicVariants& logics = myJunctionControlBuilder.getTLLogic(tlid);
            MSLane* current = MSLane::dictionary(myCurrentLane);
            if (current == 0) {
                throw InvalidArgument("An unknown lane ('" + myCurrentLane + "') should be assigned to a tl-logic.");
            }
            logics.addLink(link, current, linkNo);
        }
        return;
    }

    // get the lane the link belongs to
    MSLane* lane = MSLane::dictionary(laneId);
    if (lane == 0) {
        throw InvalidArgument("An unknown lane ('" + laneId + "') should be set as a follower for lane '" + myCurrentLane + "'.");
    }
#ifdef HAVE_INTERNAL_LANES
    MSLane* via = 0;
    if (viaID != "" && MSGlobals::gUsingInternalLanes) {
        via = MSLane::dictionary(viaID);
        if (via == 0) {
            throw InvalidArgument("An unknown lane ('" + viaID + "') should be set as a via-lane for lane '" + myCurrentLane + "'.");
        }
    }
    if (pass >= 0) {
        static_cast<MSInternalLane*>(lane)->setPassPosition(pass);
    }
#endif
    MSLane* orig = MSLane::dictionary(myCurrentLane);
    if (orig == 0) {
        return;
    }


    // build the link
    SUMOReal length = orig != 0 && lane != 0
                      ? orig->getShape()[-1].distanceTo(lane->getShape()[0])
                      : 0;
#ifdef HAVE_INTERNAL_LANES
    if (via != 0) {
        length = via->getLength();
    }
    MSLink* link = new MSLink(lane, via, dir, state, length);
#else
    MSLink* link = new MSLink(lane, dir, state, length);
#endif

    MSLane* clane = MSLane::dictionary(myCurrentLane);
    if (clane != 0) {
#ifdef HAVE_INTERNAL_LANES
        if (via != 0) {
            via->addIncomingLane(clane, link);
        } else {
            lane->addIncomingLane(clane, link);
        }
#else
        lane->addIncomingLane(clane, link);
#endif
        lane->addApproachingLane(clane);
    }
    // if a traffic light is responsible for it, inform the traffic light
    // check whether this link is controlled by a traffic light
    if (tlid != "") {
        MSTLLogicControl::TLSLogicVariants& logics = myJunctionControlBuilder.getTLLogic(tlid);
        MSLane* current = MSLane::dictionary(myCurrentLane);
        if (current == 0) {
            throw InvalidArgument("An unknown lane ('" + myCurrentLane + "') should be assigned to a tl-logic.");
        }
        logics.addLink(link, current, linkNo);
    }
    // add the link to the container
    mySuccLanes->push_back(link);
}


void
NLSucceedingLaneBuilder::closeSuccLane() throw(InvalidArgument) {
    MSLane* current = MSLane::dictionary(myCurrentLane);
    if (current == 0) {
        throw InvalidArgument("Trying to close connections of an unknown lane ('" + myCurrentLane + "').");
    }
    MSLinkCont* cont = new MSLinkCont();
    cont->reserve(mySuccLanes->size());
    copy(mySuccLanes->begin(), mySuccLanes->end(), back_inserter(*cont));
    current->initialize(cont);
    mySuccLanes->clear();
}


const std::string&
NLSucceedingLaneBuilder::getCurrentLaneName() const {
    return myCurrentLane;
}



/****************************************************************************/

