/****************************************************************************/
/// @file    MSLane.cpp
/// @author  Christian Roessel
/// @author  Jakob Erdmann
/// @author  Daniel Krajzewicz
/// @author  Tino Morenz
/// @author  Axel Wegener
/// @author  Michael Behrisch
/// @author  Christoph Sommer
/// @date    Mon, 05 Mar 2001
/// @version $Id: MSLane.cpp 11701 2012-01-10 22:23:40Z behrisch $
///
// Representation of a lane in the micro simulation
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

#include <utils/common/UtilExceptions.h>
#include <utils/common/StdDefs.h>
#include "MSVehicle.h"
#include "MSVehicleType.h"
#include "MSEdge.h"
#include "MSEdgeControl.h"
#include "MSJunction.h"
#include "MSLogicJunction.h"
#include "MSLink.h"
#include "MSLane.h"
#include "MSVehicleTransfer.h"
#include "MSGlobals.h"
#include "MSVehicleControl.h"
#include <cmath>
#include <bitset>
#include <iostream>
#include <cassert>
#include <functional>
#include <algorithm>
#include <iterator>
#include <exception>
#include <climits>
#include <set>
#include <utils/common/MsgHandler.h>
#include <utils/common/ToString.h>
#include <utils/options/OptionsCont.h>
#include <utils/common/HelpersHarmonoise.h>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS

// ===========================================================================
// static member definitions
// ===========================================================================
MSLane::DictType MSLane::myDict;

// ===========================================================================
// member method definitions
// ===========================================================================
MSLane::MSLane
 (const std::string& id, SUMOReal maxSpeed, SUMOReal length, MSEdge* const edge,
  unsigned int numericalID, const PositionVector& shape, SUMOReal width,
  const SUMOVehicleClasses& allowed, const SUMOVehicleClasses& disallowed,
  SUMOReal slope)
 :Named(id),
  myShape(shape), myNumericalID(numericalID), myVehicles(), myLength(length),
  myWidth(width), myEdge(edge), myMaxSpeed(maxSpeed), myAllowedClasses(allowed),
  myNotAllowedClasses(disallowed), myLogicalPredecessorLane(0),
  myVehicleLengthSum(0), myInlappingVehicleEnd(10000), myInlappingVehicle(0),
  mySlope(slope)
{
  if(OptionsCont::getOptions().getInt("buildVerbosity") > 1)
    printf("----> MSLane::MSLane(...)\n");
}

MSLane::~MSLane() {
    for (MSLinkCont::iterator i = myLinks.begin(); i != myLinks.end(); ++i) {
        delete *i;
    }
}

void
MSLane::initialize(MSLinkCont* links) {
    myLinks = *links;
    delete links;
}

void
MSLane::addLink(MSLink* link) {
    myLinks.push_back(link);
}

// ------ interaction with MSMoveReminder ------
void
MSLane::addMoveReminder(MSMoveReminder* rem) {
    myMoveReminders.push_back(rem);
    for (VehCont::iterator veh = myVehicles.begin(); veh != myVehicles.end(); ++veh) {
        (*veh)->addReminder(rem);
    }
}

SUMOReal MSLane::getMySlope() const
{
    return mySlope;
}

void MSLane::setSlope(SUMOReal myNewSlope)
{
    mySlope = myNewSlope;
}

// ------ Vehicle emission ------
void
MSLane::incorporateVehicle(MSVehicle* veh, SUMOReal pos, SUMOReal speed, const MSLane::VehCont::iterator& at, MSMoveReminder::Notification notification) {
    bool wasInactive = myVehicles.size() == 0;
    veh->enterLaneAtInsertion(this, pos, speed, notification);
    if (at == myVehicles.end()) {
        // vehicle will be the first on the lane
        myVehicles.push_back(veh);
    } else {
        myVehicles.insert(at, veh);
    }
    myVehicleLengthSum += veh->getVehicleType().getLengthWithGap();
    if (wasInactive) {
        MSNet::getInstance()->getEdgeControl().gotActive(this);
    }
}


bool
MSLane::pWagGenericInsertion(MSVehicle& veh, SUMOReal mspeed, SUMOReal maxPos, SUMOReal minPos) {
    SUMOReal xIn = maxPos;
    SUMOReal vIn = mspeed;
    SUMOReal leaderDecel;
    if (myVehicles.size() != 0) {
        MSVehicle* leader = myVehicles.front();
        xIn = leader->getPositionOnLane() - leader->getVehicleType().getLengthWithGap();
        vIn = leader->getSpeed();
        leaderDecel = leader->getCarFollowModel().getMaxDecel();
    } else {
        veh.getBestLanes(true, this);
        SUMOReal brakeGap = veh.getCarFollowModel().brakeGap(mspeed);
        std::pair<MSVehicle* const, SUMOReal> leader = getLeaderOnConsecutive(brakeGap, 0, mspeed, veh, veh.getBestLanesContinuation(this));
        if (leader.first != 0) {
            xIn = getLength() + leader.second;
            vIn = leader.first->getSpeed();
            leaderDecel = leader.first->getCarFollowModel().getMaxDecel();
        } else {
            incorporateVehicle(&veh, maxPos, mspeed, myVehicles.end());
            return true;
        }
    }
    const SUMOReal vHlp = 0.5 * (vIn + mspeed);
    SUMOReal x2 = xIn;// have seen leader length already - skCar::lCar;
    SUMOReal x1 = x2 - 100.0;
    SUMOReal x = 0;
    for (int i = 0; i <= 10; i++) {
        x = 0.5 * (x1 + x2);
        SUMOReal vSafe = veh.getCarFollowModel().followSpeed(&veh, vHlp, xIn - x, vIn, leaderDecel);
        if (vSafe < vHlp) {
            x2 = x;
        } else {
            x1 = x;
        }
    }
    if (x < minPos) {
        return false;
    } else if (x > maxPos) {
        x = maxPos;
    }
    incorporateVehicle(&veh, x, vHlp, myVehicles.begin());
    return true;
}


bool
MSLane::pWagSimpleInsertion(MSVehicle& veh, SUMOReal mspeed, SUMOReal maxPos, SUMOReal minPos) {
    SUMOReal xIn = maxPos;
    SUMOReal vIn = mspeed;
    if (myVehicles.size() != 0) {
        MSVehicle* leader = myVehicles.front();
        xIn = leader->getPositionOnLane() - leader->getVehicleType().getLengthWithGap();
        vIn = leader->getSpeed();
    } else {
        veh.getBestLanes(true, this);
        SUMOReal brakeGap = veh.getCarFollowModel().brakeGap(mspeed);
        std::pair<MSVehicle* const, SUMOReal> leader = getLeaderOnConsecutive(brakeGap, 0, mspeed, veh, veh.getBestLanesContinuation(this));
        if (leader.first != 0) {
            xIn = getLength() + leader.second;
            vIn = leader.first->getSpeed();
        } else {
            incorporateVehicle(&veh, maxPos, mspeed, myVehicles.end());
            return true;
        }
    }
    const SUMOReal vHlp = 0.5 * (mspeed + vIn);
    xIn = xIn - vHlp * veh.getCarFollowModel().getHeadwayTime();
    if (xIn < minPos) {
        return false;
    } else if (xIn > maxPos) {
        xIn = maxPos;
    }
    incorporateVehicle(&veh, xIn, vHlp, myVehicles.begin());
    return true;
}


bool
MSLane::maxSpeedGapInsertion(MSVehicle& veh, SUMOReal mspeed) {
    if (myVehicles.size() == 0) {
        return isInsertionSuccess(&veh, mspeed, myLength / 2, true);
    }
    // go through the lane, look for free positions (starting after the last vehicle)
    MSLane::VehCont::iterator predIt = myVehicles.begin();
    SUMOReal maxSpeed = 0;
    SUMOReal maxPos = 0;
    MSLane::VehCont::iterator maxIt = myVehicles.begin();
    while (predIt != myVehicles.end()) {
        // get leader (may be zero) and follower
        const MSVehicle* leader = predIt != myVehicles.end() - 1 ? *(predIt + 1) : getPartialOccupator();
        const MSVehicle* follower = *predIt;
        SUMOReal leaderRearPos = getLength();
        SUMOReal leaderSpeed = mspeed;
        if (leader != 0) {
            leaderRearPos = leader->getPositionOnLane() - leader->getVehicleType().getLengthWithGap();
            if (leader == getPartialOccupator()) {
                leaderRearPos = getPartialOccupatorEnd();
            }
            leaderSpeed = leader->getSpeed();
        }
        const SUMOReal gap = leaderRearPos - follower->getPositionOnLane();
        const SUMOReal fSpeed = follower->getSpeed();
        const SUMOReal currentMaxSpeed = (gap + leaderSpeed * leaderSpeed / veh.getCarFollowModel().getMaxDecel() - fSpeed * fSpeed / follower->getCarFollowModel().getMaxDecel()) / veh.getCarFollowModel().getHeadwayTime() - fSpeed;
        if (MIN2(currentMaxSpeed, mspeed) > maxSpeed) {
            maxSpeed = currentMaxSpeed;
            maxPos = (leaderRearPos + follower->getPositionOnLane() + veh.getVehicleType().getLengthWithGap()) / 2;
            maxIt = predIt+1;
        }
        ++predIt;
    }
    if (maxSpeed > 0) {
        incorporateVehicle(&veh, maxPos, maxSpeed, maxIt);
        return true;
    }
    return false;
}


bool
MSLane::freeInsertion(MSVehicle& veh, SUMOReal mspeed,
                      MSMoveReminder::Notification notification) {
    bool adaptableSpeed = true;
    if (myVehicles.size() == 0) {
        if (isInsertionSuccess(&veh, mspeed, 0, adaptableSpeed, notification)) {
            return true;
        }
    } else {
        // check whether the vehicle can be put behind the last one if there is such
        MSVehicle* leader = myVehicles.back();
        SUMOReal leaderPos = leader->getPositionOnLane() - leader->getVehicleType().getLengthWithGap();
        SUMOReal speed = mspeed;
        if (adaptableSpeed) {
            speed = leader->getSpeed();
        }
        SUMOReal frontGapNeeded = veh.getCarFollowModel().getSecureGap(speed, leader->getSpeed(), leader->getCarFollowModel().getMaxDecel());
        if (leaderPos - frontGapNeeded >= 0) {
            SUMOReal tspeed = MIN2(veh.getCarFollowModel().followSpeed(&veh, mspeed, frontGapNeeded, leader->getSpeed(), leader->getCarFollowModel().getMaxDecel()), mspeed);
            // check whether we can insert our vehicle behind the last vehicle on the lane
            if (isInsertionSuccess(&veh, tspeed, 0, adaptableSpeed, notification)) {
                return true;
            }
        }
    }
    // go through the lane, look for free positions (starting after the last vehicle)
    MSLane::VehCont::iterator predIt = myVehicles.begin();
    while (predIt != myVehicles.end()) {
        // get leader (may be zero) and follower
        const MSVehicle* leader = predIt != myVehicles.end() - 1 ? *(predIt + 1) : getPartialOccupator();
        const MSVehicle* follower = *predIt;

        // patch speed if allowed
        SUMOReal speed = mspeed;
        if (adaptableSpeed && leader != 0) {
            speed = MIN2(leader->getSpeed(), mspeed);
        }

        // compute the space needed to not collide with leader
        SUMOReal frontMax = getLength();
        if (leader != 0) {
            SUMOReal leaderRearPos = leader->getPositionOnLane() - leader->getVehicleType().getLengthWithGap();
            if (leader == getPartialOccupator()) {
                leaderRearPos = getPartialOccupatorEnd();
            }
            SUMOReal frontGapNeeded = veh.getCarFollowModel().getSecureGap(speed, leader->getSpeed(), leader->getCarFollowModel().getMaxDecel());
            frontMax = leaderRearPos - frontGapNeeded;
        }
        // compute the space needed to not let the follower collide
        const SUMOReal followPos = follower->getPositionOnLane();
        const SUMOReal backGapNeeded = follower->getCarFollowModel().getSecureGap(follower->getSpeed(), veh.getSpeed(), veh.getCarFollowModel().getMaxDecel());
        const SUMOReal backMin = followPos + backGapNeeded + veh.getVehicleType().getLengthWithGap();

        // check whether there is enough room (given some extra space for rounding errors)
        if (frontMax > 0 && backMin + POSITION_EPS < frontMax) {
            // try to insert vehicle (should be always ok)
            if (isInsertionSuccess(&veh, speed, backMin + POSITION_EPS, adaptableSpeed, notification)) {
                return true;
            }
        }
        ++predIt;
    }
    // first check at lane's begin
    return false;
}


bool
MSLane::insertVehicle(MSVehicle& veh) {
    SUMOReal pos = 0;
    SUMOReal speed = 0;
    bool patchSpeed = true; // whether the speed shall be adapted to infrastructure/traffic in front

    // determine the speed
    const SUMOVehicleParameter& pars = veh.getParameter();
    switch (pars.departSpeedProcedure) {
        case DEPART_SPEED_GIVEN:
            speed = pars.departSpeed;
            patchSpeed = false;
            break;
        case DEPART_SPEED_RANDOM:
            speed = RandHelper::rand(MIN2(veh.getMaxSpeed(), getMaxSpeed()));
            patchSpeed = true; // !!!(?)
            break;
        case DEPART_SPEED_MAX:
            speed = MIN2(veh.getMaxSpeed(), getMaxSpeed());
            patchSpeed = true; // !!!(?)
            break;
        case DEPART_SPEED_DEFAULT:
        default:
            // speed = 0 was set before
            patchSpeed = false; // !!!(?)
            break;
    }

    // determine the position
    switch (pars.departPosProcedure) {
        case DEPART_POS_GIVEN:
			pos = pars.departPos;
            if (pos < 0.) {
				pos += myLength;
            }
            break;
        case DEPART_POS_RANDOM:
            pos = RandHelper::rand(getLength());
            break;
        case DEPART_POS_RANDOM_FREE: {
            for (unsigned int i = 0; i < 10; i++) {
                // we will try some random positions ...
                pos = RandHelper::rand(getLength());
                if (isInsertionSuccess(&veh, speed, pos, patchSpeed)) {
                    return true;
                }
            }
            // ... and if that doesn't work, we put the vehicle to the free position
            return freeInsertion(veh, speed);
        }
        break;
        case DEPART_POS_FREE:
            return freeInsertion(veh, speed);
        case DEPART_POS_PWAG_SIMPLE:
            return pWagSimpleInsertion(veh, speed, getLength(), 0.0);
        case DEPART_POS_PWAG_GENERIC:
            return pWagGenericInsertion(veh, speed, getLength(), 0.0);
        case DEPART_POS_MAX_SPEED_GAP:
            return maxSpeedGapInsertion(veh, speed);
        case DEPART_POS_BASE:
        case DEPART_POS_DEFAULT:
        default:
            pos = MIN2(static_cast<SUMOReal>(veh.getVehicleType().getLengthWithGap() + POSITION_EPS), myLength);
            break;
    }
    // try to insert
    return isInsertionSuccess(&veh, speed, pos, patchSpeed);
}


bool
MSLane::isInsertionSuccess(MSVehicle* aVehicle,
                           SUMOReal speed, SUMOReal pos, bool patchSpeed,
                           MSMoveReminder::Notification notification) {
    if (pos < 0 || pos > myLength) {
        // we may not start there
        WRITE_ERROR("Vehicle '" + aVehicle->getID() + "' will not be able to depart at the given position!");
        // !!! we probably should do something else...
        return false;
    }
    aVehicle->getBestLanes(true, this);
    const MSCFModel& cfModel = aVehicle->getCarFollowModel();
    const std::vector<MSLane*> &bestLaneConts = aVehicle->getBestLanesContinuation(this);
    std::vector<MSLane*>::const_iterator ri = bestLaneConts.begin();
    SUMOReal seen = getLength() - pos;
    SUMOReal dist = cfModel.brakeGap(speed);
    const MSRoute& r = aVehicle->getRoute();
    MSRouteIterator ce = r.begin();
    MSLane* currentLane = this;
    MSLane* nextLane = this;
    SUMOTime arrivalTime = MSNet::getInstance()->getCurrentTimeStep() + TIME2STEPS(seen / speed);
    while (seen < dist && ri != bestLaneConts.end()) {
        // get the next link used...
        MSLinkCont::const_iterator link = currentLane->succLinkSec(*aVehicle, 1, *currentLane, bestLaneConts);
        // ...and the next used lane (including internal)
        if (!currentLane->isLinkEnd(link) && (*link)->opened(arrivalTime, speed, aVehicle->getVehicleType().getLengthWithGap()) && (*link)->getState() != LINKSTATE_TL_RED) { // red may have priority?
#ifdef HAVE_INTERNAL_LANES
            bool nextInternal = false;
            nextLane = (*link)->getViaLane();
            if (nextLane == 0) {
                nextLane = (*link)->getLane();
            } else {
                nextInternal = true;
            }
#else
            nextLane = (*link)->getLane();
#endif
        } else {
            break;
        }
        // check how next lane effects the journey
        if (nextLane != 0) {
            arrivalTime += TIME2STEPS(nextLane->getLength() / speed);
            SUMOReal gap = 0;
            MSVehicle* leader = currentLane->getPartialOccupator();
            if (leader != 0) {
                gap = seen + currentLane->getPartialOccupatorEnd() - currentLane->getLength();
            } else {
                // check leader on next lane
                leader = nextLane->getLastVehicle();
                if (leader != 0) {
                    gap = seen + leader->getPositionOnLane() - leader->getVehicleType().getLengthWithGap();
                }
            }
            if (leader != 0) {
                if (gap < 0) {
                    return false;
                }
                const SUMOReal nspeed = cfModel.followSpeed(aVehicle, speed, gap, leader->getSpeed(), leader->getCarFollowModel().getMaxDecel());
                if (nspeed < speed) {
                    if (patchSpeed) {
                        speed = MIN2(nspeed, speed);
                        dist = cfModel.brakeGap(speed);
                    } else {
                        // we may not drive with the given velocity - we crash into the leader
                        return false;
                    }
                }
            }
            // check next lane's maximum velocity
            const SUMOReal nspeed = nextLane->getMaxSpeed();
            if (nspeed < speed) {
                // patch speed if needed
                if (patchSpeed) {
                    speed = MIN2(cfModel.freeSpeed(aVehicle, speed, seen, nspeed), speed);
                    dist = cfModel.brakeGap(speed);
                } else {
                    // we may not drive with the given velocity - we would be too fast on the next lane
                    return false;
                }
            }
            // check traffic on next junctions
            const SUMOTime arrivalTime = MSNet::getInstance()->getCurrentTimeStep() + TIME2STEPS(seen / speed);
#ifdef HAVE_INTERNAL_LANES
            const SUMOTime leaveTime = (*link)->getViaLane() == 0 ? arrivalTime + TIME2STEPS((*link)->getLength() * speed) : arrivalTime + TIME2STEPS((*link)->getViaLane()->getLength() * speed);
#else
            const SUMOTime leaveTime = arrivalTime + TIME2STEPS((*link)->getLength() * speed);
#endif
            if ((*link)->hasApproachingFoe(arrivalTime, leaveTime)) {
                SUMOReal nspeed = cfModel.followSpeed(aVehicle, speed, seen, 0, 0);
                if (nspeed < speed) {
                    if (patchSpeed) {
                        speed = MIN2(nspeed, speed);
                        dist = cfModel.brakeGap(speed);
                    } else {
                        // we may not drive with the given velocity - we crash into the leader
                        return false;
                    }
                }
            } else {
                // we can only drive to the end of the current lane...
                SUMOReal nspeed = cfModel.followSpeed(aVehicle, speed, seen, 0, 0);
                if (nspeed < speed) {
                    if (patchSpeed) {
                        speed = MIN2(nspeed, speed);
                        dist = cfModel.brakeGap(speed);
                    } else {
                        // we may not drive with the given velocity - we crash into the leader
                        return false;
                    }
                }
            }
            seen += nextLane->getLength();
            ++ce;
            ++ri;
            currentLane = nextLane;
        }
    }
    if (seen < dist) {
        SUMOReal nspeed = cfModel.followSpeed(aVehicle, speed, seen, 0, 0);
        if (nspeed < speed) {
            if (patchSpeed) {
                speed = MIN2(nspeed, speed);
                dist = cfModel.brakeGap(speed);
            } else {
                // we may not drive with the given velocity - we crash into the leader
                WRITE_ERROR("Vehicle '" + aVehicle->getID() + "' will not be able to depart using given velocity!");
                // !!! we probably should do something else...
                return false;
            }
        }
    }

    // get the pointer to the vehicle next in front of the given position
    MSLane::VehCont::iterator predIt = find_if(myVehicles.begin(), myVehicles.end(), bind2nd(VehPosition(), pos));
    if (predIt != myVehicles.end()) {
        // ok, there is one (a leader)
        MSVehicle* leader = *predIt;
        SUMOReal frontGapNeeded = cfModel.getSecureGap(speed, leader->getSpeed(), leader->getCarFollowModel().getMaxDecel());
        SUMOReal gap = MSVehicle::gap(leader->getPositionOnLane(), leader->getVehicleType().getLengthWithGap(), pos);
        if (gap < frontGapNeeded) {
            // too close to the leader on this lane
            return false;
        }
    }

    // check back vehicle
    if (predIt != myVehicles.begin()) {
        // there is direct follower on this lane
        MSVehicle* follower = *(predIt - 1);
        SUMOReal backGapNeeded = follower->getCarFollowModel().getSecureGap(follower->getSpeed(), aVehicle->getSpeed(), cfModel.getMaxDecel());
        SUMOReal gap = MSVehicle::gap(pos, aVehicle->getVehicleType().getLengthWithGap(), follower->getPositionOnLane());
        if (gap < backGapNeeded) {
            // too close to the follower on this lane
            return false;
        }
    } else {
        // check approaching vehicle (consecutive follower)
        SUMOReal lspeed = getMaxSpeed();
        // in order to look back, we'd need the minimum braking ability of vehicles in the net...
        //  we'll assume it to be 4m/s^2
        //   !!!revisit
        SUMOReal dist = lspeed * lspeed / (2.*4.) + SPEED2DIST(lspeed);
        std::pair<const MSVehicle* const, SUMOReal> approaching = getFollowerOnConsecutive(dist, 0, speed, pos - aVehicle->getVehicleType().getLengthWithGap(), 4.5);
        if (approaching.first != 0) {
            const MSVehicle* const follower = approaching.first;
            SUMOReal backGapNeeded = follower->getCarFollowModel().getSecureGap(follower->getSpeed(), aVehicle->getSpeed(), cfModel.getMaxDecel());
            SUMOReal gap = approaching.second - pos - aVehicle->getVehicleType().getLengthWithGap();
            if (gap < backGapNeeded) {
                // too close to the consecutive follower
                return false;
            }
        }
        // check for in-lapping vehicle
        MSVehicle* leader = getPartialOccupator();
        if (leader != 0) {
            SUMOReal frontGapNeeded = cfModel.getSecureGap(speed, leader->getSpeed(), leader->getCarFollowModel().getMaxDecel());
            SUMOReal gap = getPartialOccupatorEnd() - pos;
            if (gap <= frontGapNeeded) {
                // too close to the leader on this lane
                return false;
            }
        }
    }

    // may got negative while adaptation
    if (speed < 0) {
        return false;
    }
    // enter
    incorporateVehicle(aVehicle, pos, speed, predIt, notification);
    return true;
}


void
MSLane::forceVehicleInsertion(MSVehicle* veh, SUMOReal pos) {
    incorporateVehicle(veh, pos, veh->getSpeed(), find_if(myVehicles.begin(), myVehicles.end(), bind2nd(VehPosition(), pos)));
}


// ------ Handling vehicles lapping into lanes ------
SUMOReal
MSLane::setPartialOccupation(MSVehicle* v, SUMOReal leftVehicleLength) {
    myInlappingVehicle = v;
    if (leftVehicleLength > myLength) {
        myInlappingVehicleEnd = 0;
    } else {
        myInlappingVehicleEnd = myLength - leftVehicleLength;
    }
    return myLength;
}


void
MSLane::resetPartialOccupation(MSVehicle* v) {
    if (v == myInlappingVehicle) {
        myInlappingVehicleEnd = 10000;
    }
    myInlappingVehicle = 0;
}


std::pair<MSVehicle*, SUMOReal>
MSLane::getLastVehicleInformation() const {
    if (myVehicles.size() != 0) {
        // the last vehicle is the one in scheduled by this lane
        MSVehicle* last = *myVehicles.begin();
        const SUMOReal pos = last->getPositionOnLane() - last->getVehicleType().getLengthWithGap();
        return std::make_pair(last, pos);
    }
    if (myInlappingVehicle != 0) {
        // the last one is a vehicle extending into this lane
        return std::make_pair(myInlappingVehicle, myInlappingVehicleEnd);
    }
    return std::make_pair<MSVehicle*, SUMOReal>(0, 0);
}


// ------  ------
bool
MSLane::moveCritical(SUMOTime t) {
    myLeftVehLength = myVehicleLengthSum;
    assert(myVehicles.size() != 0);
    std::vector<MSVehicle*> collisions;
    VehCont::iterator lastBeforeEnd = myVehicles.end() - 1;
    VehCont::iterator veh;
    // Move all next vehicles beside the first
    for (veh = myVehicles.begin(); veh != lastBeforeEnd;) {
        myLeftVehLength -= (*veh)->getVehicleType().getLengthWithGap();
        VehCont::const_iterator pred(veh + 1);
        if ((*veh)->moveRegardingCritical(t, this, *pred, 0, myLeftVehLength)) {
            collisions.push_back(*veh);
        }
        ++veh;
    }
    myLeftVehLength -= (*veh)->getVehicleType().getLengthWithGap();
    if ((*veh)->moveRegardingCritical(t, this, 0, 0, myLeftVehLength)) {
        collisions.push_back(*veh);
    }
    
    // (UXIO) Failing here
    //std::cout<<(*veh)->getPositionOnLane()<<" <?= "<<myLength<<std::endl;
    // (UXIO) (Make a little patch)
    //(*veh)->getPositionOnLane() = myLength; // AND PLEASE FIXME
    
    // (smendez) Getting error in this line. Don't know how to fix it
    assert((*veh)->getPositionOnLane() <= (myLength + 5.0)); // PLEASE FIXME
    assert((*veh)->getLane() == this);
    // deal with collisions
    for (std::vector<MSVehicle*>::iterator i = collisions.begin(); i != collisions.end(); ++i) {
        WRITE_WARNING("Teleporting vehicle '" + (*i)->getID() + "'; collision, lane='" + getID() + "', time=" + time2string(MSNet::getInstance()->getCurrentTimeStep()) + ".");
        myVehicleLengthSum -= (*i)->getVehicleType().getLengthWithGap();
        myVehicles.erase(find(myVehicles.begin(), myVehicles.end(), *i));
        MSVehicleTransfer::getInstance()->addVeh(t, *i);
    }
    return myVehicles.size() == 0;
}


void
MSLane::detectCollisions(SUMOTime timestep) {
    if (myVehicles.size() < 2) {
        return;
    }

    VehCont::iterator lastVeh = myVehicles.end() - 1;
    for (VehCont::iterator veh = myVehicles.begin(); veh != lastVeh;) {
        VehCont::iterator pred = veh + 1;
        SUMOReal gap = (*pred)->getPositionOnLane() - (*pred)->getVehicleType().getLengthWithGap() - (*veh)->getPositionOnLane();
        if (gap < 0) {
            MSVehicle* vehV = *veh;
            WRITE_WARNING("Teleporting vehicle '" + vehV->getID() + "'; collision, lane='" + getID() + "', time=" + time2string(MSNet::getInstance()->getCurrentTimeStep()) + ".");
            myVehicleLengthSum -= vehV->getVehicleType().getLengthWithGap();
            MSVehicleTransfer::getInstance()->addVeh(timestep, vehV);
            veh = myVehicles.erase(veh); // remove current vehicle
            lastVeh = myVehicles.end() - 1;
            if (veh == myVehicles.end()) {
                break;
            }
        } else {
            ++veh;
        }
    }
}


SUMOReal
getMaxSpeedRegardingNextLanes(MSVehicle& veh, SUMOReal speed, SUMOReal pos) {
    MSRouteIterator next = veh.getRoute().begin();
    const MSCFModel& cfModel = veh.getCarFollowModel();
    MSLane* currentLane = (*next)->getLanes()[0];
    SUMOReal seen = currentLane->getLength() - pos;
    SUMOReal dist = SPEED2DIST(speed) + cfModel.brakeGap(speed);
    SUMOReal tspeed = speed;
    while (seen < dist && next != veh.getRoute().end() - 1) {
        ++next;
        MSLane* nextLane = (*next)->getLanes()[0];
        tspeed = MIN2(cfModel.freeSpeed(&veh, tspeed, seen, nextLane->getMaxSpeed()), nextLane->getMaxSpeed());
        dist = SPEED2DIST(tspeed) + cfModel.brakeGap(tspeed);
        seen += nextLane->getMaxSpeed();
    }
    return tspeed;
}


bool MSLane::setCritical(SUMOTime t, std::vector<MSLane*> &into)
{
  // move critical vehicles
  for(VehCont::iterator i = myVehicles.begin(); i != myVehicles.end();) {
    MSVehicle* veh = *i;
    bool moved = veh->moveChecked();
    MSLane* target = veh->getLane();
    SUMOReal length = veh->getVehicleType().getLengthWithGap();
    if(veh->ends()) {
      // vehicle has reached its arrival position
      veh->onRemovalFromNet(MSMoveReminder::NOTIFICATION_ARRIVED);
      MSNet::getInstance()->getVehicleControl().scheduleVehicleRemoval(veh);
    }else if (target != 0 && moved) {
      if (target->getEdge().isVaporizing()) {
        // vehicle has reached a vaporizing edge
        veh->onRemovalFromNet(MSMoveReminder::NOTIFICATION_VAPORIZED);
        MSNet::getInstance()->getVehicleControl().scheduleVehicleRemoval(veh);
      }else {
        // vehicle has entered a new lane
        target->myVehBuffer.push_back(veh);
        SUMOReal pspeed = veh->getSpeed();
        SUMOReal oldPos = veh->getPositionOnLane() - SPEED2DIST(veh->getSpeed());
        veh->workOnMoveReminders(oldPos, veh->getPositionOnLane(), pspeed);
        into.push_back(target);
      }
    }else if (veh->isParking()) {
      // vehicle started to park
      veh->leaveLane(MSMoveReminder::NOTIFICATION_JUNCTION);
      MSVehicleTransfer::getInstance()->addVeh(t, veh);
    }else if (veh->getPositionOnLane() > getLength()) {
      // for any reasons the vehicle is beyond its lane... error
      WRITE_WARNING("Teleporting vehicle '" + veh->getID() + "'; beyond lane (2), targetLane='" + getID() + "', time=" + time2string(MSNet::getInstance()->getCurrentTimeStep()) + ".");
      MSVehicleTransfer::getInstance()->addVeh(t, veh);
    }else {
      ++i;
      continue;
    }
    myVehicleLengthSum -= length;
    i = myVehicles.erase(i);
  }
  if (myVehicles.size() > 0) {
    if (MSGlobals::gTimeToGridlock > 0
          && !(*(myVehicles.end() - 1))->isStopped()
          && (*(myVehicles.end() - 1))->getWaitingTime() > MSGlobals::gTimeToGridlock) {
      MSVehicle* veh = *(myVehicles.end() - 1);
      myVehicleLengthSum -= veh->getVehicleType().getLengthWithGap();
      myVehicles.erase(myVehicles.end() - 1);
      WRITE_WARNING("Teleporting vehicle '" + veh->getID() + "'; waited too long, lane='" + getID() + "', time=" + time2string(MSNet::getInstance()->getCurrentTimeStep()) + ".");
      MSVehicleTransfer::getInstance()->addVeh(t, veh);
    }
  }
  return myVehicles.size() == 0;
}


bool
MSLane::dictionary(std::string id, MSLane* ptr) {
    DictType::iterator it = myDict.find(id);
    if (it == myDict.end()) {
        // id not in myDict.
        myDict.insert(DictType::value_type(id, ptr));
        return true;
    }
    return false;
}


MSLane*
MSLane::dictionary(std::string id) {
    DictType::iterator it = myDict.find(id);
    if (it == myDict.end()) {
        // id not in myDict.
        return 0;
    }
    return it->second;
}


void
MSLane::clear() {
    for (DictType::iterator i = myDict.begin(); i != myDict.end(); ++i) {
        delete(*i).second;
    }
    myDict.clear();
}


void
MSLane::insertIDs(std::vector<std::string> &into) {
    for (DictType::iterator i = myDict.begin(); i != myDict.end(); ++i) {
        into.push_back((*i).first);
    }
}


bool
MSLane::appropriate(const MSVehicle* veh) {
    if (myEdge->getPurpose() == MSEdge::EDGEFUNCTION_INTERNAL) {
        return true;
    }
    MSLinkCont::const_iterator link = succLinkSec(*veh, 1, *this, veh->getBestLanesContinuation());
    return (link != myLinks.end());
}


bool
MSLane::integrateNewVehicle(SUMOTime) {
    bool wasInactive = myVehicles.size() == 0;
    sort(myVehBuffer.begin(), myVehBuffer.end(), vehicle_position_sorter());
    for (std::vector<MSVehicle*>::const_iterator i = myVehBuffer.begin(); i != myVehBuffer.end(); ++i) {
        MSVehicle* veh = *i;
        myVehicles.push_front(veh);
        myVehicleLengthSum += veh->getVehicleType().getLengthWithGap();
    }
    myVehBuffer.clear();
    return wasInactive && myVehicles.size() != 0;
}


bool
MSLane::isLinkEnd(MSLinkCont::const_iterator& i) const {
    return i == myLinks.end();
}


bool
MSLane::isLinkEnd(MSLinkCont::iterator& i) {
    return i == myLinks.end();
}


MSVehicle*
MSLane::getLastVehicle() const {
    if (myVehicles.size() == 0) {
        return 0;
    }
    return *myVehicles.begin();
}


const MSVehicle*
MSLane::getFirstVehicle() const {
    if (myVehicles.size() == 0) {
        return 0;
    }
    return *(myVehicles.end() - 1);
}


MSLinkCont::const_iterator
MSLane::succLinkSec(const SUMOVehicle& veh, unsigned int nRouteSuccs,
                    const MSLane& succLinkSource, const std::vector<MSLane*> &conts) const {
    const MSEdge* nRouteEdge = veh.succEdge(nRouteSuccs);
    // check whether the vehicle tried to look beyond its route
    if (nRouteEdge == 0) {
        // return end (no succeeding link) if so
        return succLinkSource.myLinks.end();
    }
    // a link may be used if
    //  1) there is a destination lane ((*link)->getLane()!=0)
    //  2) the destination lane belongs to the next edge in route ((*link)->getLane()->myEdge == nRouteEdge)
    //  3) the destination lane allows the vehicle's class ((*link)->getLane()->allowsVehicleClass(veh.getVehicleClass()))

    // at first, we'll assume we have the continuations of our route in "conts" (built in "getBestLanes")
    //  "conts" stores the best continuations of our current lane
    MSLinkCont::const_iterator link;
    if (nRouteSuccs > 0 && conts.size() >= nRouteSuccs && nRouteSuccs > 0) {
        // we go through the links in our list and return the matching one
        for (link = succLinkSource.myLinks.begin(); link != succLinkSource.myLinks.end() ; ++link) {
            if ((*link)->getLane() != 0 && (*link)->getLane()->myEdge == nRouteEdge && (*link)->getLane()->allowsVehicleClass(veh.getVehicleType().getVehicleClass())) {
                // we should use the link if it connects us to the best lane
                if ((*link)->getLane() == conts[nRouteSuccs - 1]) {
                    return link;
                }
            }
        }
    }

    // ok, we were not able to use the conts for any reason
    //  we will now collect allowed links, at first
    // collect allowed links
    std::vector<MSLinkCont::const_iterator> valid;
    for (link = succLinkSource.myLinks.begin(); link != succLinkSource.myLinks.end() ; ++link) {
        if ((*link)->getLane() != 0 && (*link)->getLane()->myEdge == nRouteEdge && (*link)->getLane()->allowsVehicleClass(veh.getVehicleType().getVehicleClass())) {
            valid.push_back(link);
        }
    }
    // if no valid link was found...
    if (valid.size() == 0) {
        // ... return end (no succeeding link)
        return succLinkSource.myLinks.end();
    }
    // if there is only one valid link, let's use it...
    if (valid.size() == 1) {
        return *(valid.begin());
    }
    // if the next edge is the route end, then we may return an arbitary link
    // also, if there is no allowed lane on the edge following the current one (recheck?)
    const MSEdge* nRouteEdge2 = veh.succEdge(nRouteSuccs + 1);
    const std::vector<MSLane*> *next_allowed = nRouteEdge->allowedLanes(*nRouteEdge2, veh.getVehicleType().getVehicleClass());
    if (nRouteEdge2 == 0 || next_allowed == 0) {
        return *(valid.begin());
    }
    // now let's determine which link is the best
    // in fact, we do not know it, here...
    for (std::vector<MSLinkCont::const_iterator>::iterator i = valid.begin(); i != valid.end(); ++i) {
        if (find(next_allowed->begin(), next_allowed->end(), (**i)->getLane()) != next_allowed->end()) {
            return *i;
        }
    }
    return *(valid.begin());
}



const MSLinkCont&
MSLane::getLinkCont() const {
    return myLinks;
}


void
MSLane::swapAfterLaneChange(SUMOTime) {
    myVehicles = myTmpVehicles;
    myTmpVehicles.clear();
}

GUILaneWrapper* MSLane::buildLaneWrapper(unsigned int)
{
    if(OptionsCont::getOptions().getInt("buildVerbosity") > 1)
      printf("----> GUILaneWrapper* MSLane::buildLaneWrapper(...)\n");    
    throw "Only within the gui-version";
}

MSVehicle* MSLane::removeVehicle(MSVehicle* remVehicle) {
    for (MSLane::VehCont::iterator it = myVehicles.begin(); it < myVehicles.end(); it++) {
        if (remVehicle->getID() == (*it)->getID()) {
            remVehicle->leaveLane(MSMoveReminder::NOTIFICATION_ARRIVED);
            myVehicles.erase(it);
            myVehicleLengthSum -= remVehicle->getVehicleType().getLengthWithGap();
            break;
        }
    }
    return remVehicle;
}

MSLane* MSLane::getLeftLane() const {
    return myEdge->leftLane(this);
}

MSLane* MSLane::getRightLane() const {
    return myEdge->rightLane(this);
}

bool MSLane::allowsVehicleClass(SUMOVehicleClass vclass) const {
    if (vclass == SVC_UNKNOWN) {
        return true;
    }
    if (myAllowedClasses.size() == 0 && myNotAllowedClasses.size() == 0) {
        return true;
    }
    if (find(myAllowedClasses.begin(), myAllowedClasses.end(), vclass) != myAllowedClasses.end()) {
        return true;
    }
    if (myAllowedClasses.size() != 0) {
        return false;
    }
    if (find(myNotAllowedClasses.begin(), myNotAllowedClasses.end(), vclass) != myNotAllowedClasses.end()) {
        return false;
    }
    return true;
}


void
MSLane::addIncomingLane(MSLane* lane, MSLink* viaLink) {
    IncomingLaneInfo ili;
    ili.lane = lane;
    ili.viaLink = viaLink;
    ili.length = lane->getLength();
    myIncomingLanes.push_back(ili);
}


void
MSLane::addApproachingLane(MSLane* lane) {
    MSEdge* approachingEdge = &lane->getEdge();
    if (myApproachingLanes.find(approachingEdge) == myApproachingLanes.end()) {
        myApproachingLanes[approachingEdge] = std::vector<MSLane*>();
    }
    myApproachingLanes[approachingEdge].push_back(lane);
}


bool
MSLane::isApproachedFrom(MSEdge* const edge) {
    return myApproachingLanes.find(edge) != myApproachingLanes.end();
}


bool
MSLane::isApproachedFrom(MSEdge* const edge, MSLane* const lane) {
    std::map<MSEdge*, std::vector<MSLane*> >::const_iterator i = myApproachingLanes.find(edge);
    if (i == myApproachingLanes.end()) {
        return false;
    }
    const std::vector<MSLane*> &lanes = (*i).second;
    return find(lanes.begin(), lanes.end(), lane) != lanes.end();
}


class by_second_sorter {
public:
    inline int operator()(const std::pair<const MSVehicle* , SUMOReal> &p1, const std::pair<const MSVehicle* , SUMOReal> &p2) const {
        return p1.second < p2.second;
    }
};

std::pair<MSVehicle* const, SUMOReal>
MSLane::getFollowerOnConsecutive(SUMOReal dist, SUMOReal seen, SUMOReal leaderSpeed,
                                 SUMOReal backOffset, SUMOReal predMaxDecel) const {
    // ok, a vehicle has not noticed the lane about itself;
    //  iterate as long as necessary to search for an approaching one
    std::set<MSLane*> visited;
    std::vector<std::pair<MSVehicle*, SUMOReal> > possible;
    std::vector<MSLane::IncomingLaneInfo> newFound;
    std::vector<MSLane::IncomingLaneInfo> toExamine = myIncomingLanes;
    while (toExamine.size() != 0) {
        for (std::vector<MSLane::IncomingLaneInfo>::iterator i = toExamine.begin(); i != toExamine.end(); ++i) {
            /*
            if ((*i).viaLink->getState()==LINKSTATE_TL_RED) {
                continue;
            }
            */
            MSLane* next = (*i).lane;
            if (next->getFirstVehicle() != 0) {
                MSVehicle* v = (MSVehicle*) next->getFirstVehicle();
                SUMOReal agap = (*i).length - v->getPositionOnLane() + backOffset;
                if (agap <= v->getCarFollowModel().getSecureGap(v->getCarFollowModel().maxNextSpeed(v->getSpeed()), leaderSpeed, predMaxDecel)) {
                    possible.push_back(std::make_pair(v, (*i).length - v->getPositionOnLane() + seen));
                }
            } else {
                if ((*i).length + seen < dist) {
                    const std::vector<MSLane::IncomingLaneInfo> &followers = next->getIncomingLanes();
                    for (std::vector<MSLane::IncomingLaneInfo>::const_iterator j = followers.begin(); j != followers.end(); ++j) {
                        if (visited.find((*j).lane) == visited.end()) {
                            visited.insert((*j).lane);
                            MSLane::IncomingLaneInfo ili;
                            ili.lane = (*j).lane;
                            ili.length = (*j).length + (*i).length;
                            ili.viaLink = (*j).viaLink;
                            newFound.push_back(ili);
                        }
                    }
                }
            }
        }
        toExamine.clear();
        swap(newFound, toExamine);
    }
    if (possible.size() == 0) {
        return std::pair<MSVehicle * const, SUMOReal>(static_cast<MSVehicle*>(0), -1);
    }
    sort(possible.begin(), possible.end(), by_second_sorter());
    return *(possible.begin());
}


std::pair<MSVehicle* const, SUMOReal>
MSLane::getLeaderOnConsecutive(SUMOReal dist, SUMOReal seen, SUMOReal speed, const MSVehicle& veh,
                               const std::vector<MSLane*> &bestLaneConts) const {
    if (seen > dist) {
        return std::pair<MSVehicle * const, SUMOReal>(static_cast<MSVehicle*>(0), -1);
    }
    unsigned int view = 1;
    // loop over following lanes
    const MSLane* targetLane = this;
    MSVehicle* leader = targetLane->getPartialOccupator();
    if (leader != 0) {
        return std::pair<MSVehicle * const, SUMOReal>(leader, seen - targetLane->getPartialOccupatorEnd());
    }
    const MSLane* nextLane = targetLane;
    SUMOTime arrivalTime = MSNet::getInstance()->getCurrentTimeStep() + TIME2STEPS(seen / speed);
    while (true) {
        // get the next link used
        MSLinkCont::const_iterator link = targetLane->succLinkSec(veh, view, *nextLane, bestLaneConts);
        if (nextLane->isLinkEnd(link) || !(*link)->opened(arrivalTime, speed, veh.getVehicleType().getLengthWithGap()) || (*link)->getState() == LINKSTATE_TL_RED) {
            return std::pair<MSVehicle * const, SUMOReal>(static_cast<MSVehicle*>(0), -1);
        }
#ifdef HAVE_INTERNAL_LANES
        bool nextInternal = false;
        nextLane = (*link)->getViaLane();
        if (nextLane == 0) {
            nextLane = (*link)->getLane();
        } else {
            nextInternal = true;
        }
#else
        nextLane = (*link)->getLane();
#endif
        if (nextLane == 0) {
            return std::pair<MSVehicle * const, SUMOReal>(static_cast<MSVehicle*>(0), -1);
        }
        arrivalTime += TIME2STEPS(nextLane->getLength() / speed);
        MSVehicle* leader = nextLane->getLastVehicle();
        if (leader != 0) {
            return std::pair<MSVehicle * const, SUMOReal>(leader, seen + leader->getPositionOnLane() - leader->getVehicleType().getLengthWithGap());
        } else {
            leader = nextLane->getPartialOccupator();
            if (leader != 0) {
                return std::pair<MSVehicle * const, SUMOReal>(leader, seen + nextLane->getPartialOccupatorEnd());
            }
        }
        if (nextLane->getMaxSpeed() < speed) {
            dist = veh.getCarFollowModel().brakeGap(nextLane->getMaxSpeed());
        }
        seen += nextLane->getLength();
        if (seen > dist) {
            return std::pair<MSVehicle * const, SUMOReal>(static_cast<MSVehicle*>(0), -1);
        }
#ifdef HAVE_INTERNAL_LANES
        if (!nextInternal) {
            view++;
        }
#else
        view++;
#endif
    }
}


MSLane*
MSLane::getLogicalPredecessorLane() const {
    if (myLogicalPredecessorLane != 0) {
        return myLogicalPredecessorLane;
    }
    if (myLogicalPredecessorLane == 0) {
        std::vector<MSEdge*> pred = myEdge->getIncomingEdges();
        // get only those edges which connect to this lane
        for (std::vector<MSEdge*>::iterator i = pred.begin(); i != pred.end();) {
            std::vector<IncomingLaneInfo>::const_iterator j = find_if(myIncomingLanes.begin(), myIncomingLanes.end(), edge_finder(*i));
            if (j == myIncomingLanes.end()) {
                i = pred.erase(i);
            } else {
                ++i;
            }
        }
        // get the edge with the most connections to this lane's edge
        if (pred.size() != 0) {
            std::sort(pred.begin(), pred.end(), by_connections_to_sorter(&getEdge()));
            MSEdge* best = *pred.begin();
            std::vector<IncomingLaneInfo>::const_iterator j = find_if(myIncomingLanes.begin(), myIncomingLanes.end(), edge_finder(best));
            myLogicalPredecessorLane = (*j).lane;
        }
    }
    return myLogicalPredecessorLane;
}


void
MSLane::leftByLaneChange(MSVehicle* v) {
    myVehicleLengthSum -= v->getVehicleType().getLengthWithGap();
}


void
MSLane::enteredByLaneChange(MSVehicle* v) {
    myVehicleLengthSum += v->getVehicleType().getLengthWithGap();
}


// ------------ Current state retrieval
SUMOReal
MSLane::getOccupancy() const {
    return myVehicleLengthSum / myLength;
}


SUMOReal
MSLane::getVehLenSum() const {
    return myVehicleLengthSum;
}


SUMOReal
MSLane::getMeanSpeed() const {
    if (myVehicles.size() == 0) {
        return myMaxSpeed;
    }
    SUMOReal v = 0;
    const MSLane::VehCont& vehs = getVehiclesSecure();
    for (VehCont::const_iterator i = vehs.begin(); i != vehs.end(); ++i) {
        v += (*i)->getSpeed();
    }
    SUMOReal ret = v / (SUMOReal) myVehicles.size();
    releaseVehicles();
    return ret;
}


SUMOReal
MSLane::getHBEFA_CO2Emissions() const {
    SUMOReal ret = 0;
    const MSLane::VehCont& vehs = getVehiclesSecure();
    for (MSLane::VehCont::const_iterator i = vehs.begin(); i != vehs.end(); ++i) {
        ret += (*i)->getHBEFA_CO2Emissions();
    }
    releaseVehicles();
    return ret;
}


SUMOReal
MSLane::getHBEFA_COEmissions() const {
    SUMOReal ret = 0;
    const MSLane::VehCont& vehs = getVehiclesSecure();
    for (MSLane::VehCont::const_iterator i = vehs.begin(); i != vehs.end(); ++i) {
        ret += (*i)->getHBEFA_COEmissions();
    }
    releaseVehicles();
    return ret;
}


SUMOReal
MSLane::getHBEFA_PMxEmissions() const {
    SUMOReal ret = 0;
    const MSLane::VehCont& vehs = getVehiclesSecure();
    for (MSLane::VehCont::const_iterator i = vehs.begin(); i != vehs.end(); ++i) {
        ret += (*i)->getHBEFA_PMxEmissions();
    }
    releaseVehicles();
    return ret;
}


SUMOReal
MSLane::getHBEFA_NOxEmissions() const {
    SUMOReal ret = 0;
    const MSLane::VehCont& vehs = getVehiclesSecure();
    for (MSLane::VehCont::const_iterator i = vehs.begin(); i != vehs.end(); ++i) {
        ret += (*i)->getHBEFA_NOxEmissions();
    }
    releaseVehicles();
    return ret;
}


SUMOReal
MSLane::getHBEFA_HCEmissions() const {
    SUMOReal ret = 0;
    const MSLane::VehCont& vehs = getVehiclesSecure();
    for (MSLane::VehCont::const_iterator i = vehs.begin(); i != vehs.end(); ++i) {
        ret += (*i)->getHBEFA_HCEmissions();
    }
    releaseVehicles();
    return ret;
}

SUMOReal MSLane::getHBEFA_FuelConsumption() const {
    SUMOReal ret = 0;
    const MSLane::VehCont& vehs = getVehiclesSecure();
    for (MSLane::VehCont::const_iterator i = vehs.begin(); i != vehs.end(); ++i) {
        ret += (*i)->getHBEFA_FuelConsumption();
    }
    releaseVehicles();
    return ret;
}

SUMOReal MSLane::getHarmonoise_NoiseEmissions() const {
    SUMOReal ret = 0;
    const MSLane::VehCont& vehs = getVehiclesSecure();
    if (vehs.size() == 0) {
        releaseVehicles();
        return 0;
    }
    for (MSLane::VehCont::const_iterator i = vehs.begin(); i != vehs.end(); ++i) {
        SUMOReal sv = (*i)->getHarmonoise_NoiseEmissions();
        ret += (SUMOReal) pow(10., (sv / 10.));
    }
    releaseVehicles();
    return HelpersHarmonoise::sum(ret);
}

MSJunction * MSLane::getFromJunction() const { return myEdge->getFromJunction(); }

void MSLane::setFromJunction(MSJunction *arg) { myEdge->setFromJunction(arg); }

MSJunction * MSLane::getToJunction() const { return myEdge->getToJunction(); }

void MSLane::setToJunction(MSJunction *arg) { myEdge->setToJunction(arg); }

void MSLane::setNameToJunction(std::string arg) { myEdge->setNameOfMyToJunction(arg); }