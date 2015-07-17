/****************************************************************************/
/// @file    MSVehicle.cpp
/// @author  Christian Roessel
/// @author  Jakob Erdmann
/// @author  Bjoern Hendriks
/// @author  Daniel Krajzewicz
/// @author  Thimor Bohn
/// @author  Friedemann Wesner
/// @author  Laura Bieker
/// @author  Clemens Honomichl
/// @author  Michael Behrisch
/// @author  Axel Wegener
/// @author  Christoph Sommer
/// @date    Mon, 05 Mar 2001
/// @version $Id: MSVehicle.cpp 11745 2012-01-20 07:47:11Z dkrajzew $
///
// Representation of a vehicle in the micro simulation
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
#include <stdlib.h>
#include "additionalConfig.h"
#include "MSLane.h"
#include "MSVehicle.h"
#include "MSEdge.h"
#include "MSVehicleType.h"
#include "MSNet.h"
#include "MSRoute.h"
#include "MSLinkCont.h"
#include <utils/common/StringUtils.h>
#include <utils/common/StdDefs.h>
#include <utils/common/SUMOTime.h>
#include <utils/geom/GeoConvHelper.h>
#include <microsim/MSVehicleControl.h>
#include <microsim/MSGlobals.h>
#include "microsim/devices/MSDevice_FEV.h"
#include <iostream>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <time.h>
#include <cstring>
#include <algorithm>
#include <map>
#include "MSMoveReminder.h"
#include <utils/options/OptionsCont.h>
#include "MSLCM_DK2004.h"
#include <utils/common/ToString.h>
#include <utils/common/FileHelpers.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/iodevices/BinaryInputDevice.h>
#include "trigger/MSBusStop.h"
#include <utils/common/DijkstraRouterTT.h>
#include "MSPerson.h"
#include "MSPersonControl.h"
#include <utils/common/RandHelper.h>
#include "devices/MSDevice_Person.h"
#include "MSEdgeWeightsStorage.h"
#include <utils/common/HelpersHBEFA.h>
#include <utils/common/HelpersHarmonoise.h>
#include "scenload/MyHandler2.h"
#include "scenload/VehicleToTrack.h"

#ifdef _MESSAGES
#include "MSMessageEmitter.h"
#endif

#ifdef HAVE_MESOSIM
#include <mesosim/MESegment.h>
#include <mesosim/MELoop.h>
#include "MSGlobals.h"
#endif

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS
//#define DEBUG_VEHICLE_GUI_SELECTION 1
#ifdef DEBUG_VEHICLE_GUI_SELECTION
#include <utils/gui/div/GUIGlobalSelection.h>
#include <guisim/GUIVehicle.h>
#include <guisim/GUILane.h>
#endif

#define BUS_STOP_OFFSET 0.5

static Position myLastAdasPosition(0,0);
static bool first=true;

// ===========================================================================
// static value definitions
// ===========================================================================
std::vector<MSLane*> MSVehicle::myEmptyLaneVector;

// ===========================================================================
// method definitions
// ===========================================================================
/* -------------------------------------------------------------------------
 * methods of MSVehicle::State
 * ----------------------------------------------------------------------- */
MSVehicle::State::State(const State& state) {
	myPos = state.myPos;
	mySpeed = state.mySpeed;
	myTimeEnterLane = state.myTimeEnterLane;

}

MSVehicle::State&
MSVehicle::State::operator=(const State& state) {
	myPos = state.myPos;
	mySpeed = state.mySpeed;
	myTimeEnterLane = state.myTimeEnterLane;
	return *this;
}

bool MSVehicle::State::operator!=(const State& state) {
	return (myPos != state.myPos || mySpeed != state.mySpeed
			|| myTimeEnterLane != state.myTimeEnterLane);
}

SUMOReal MSVehicle::State::pos() const {
	return myPos;
}

MSVehicle::State::State(SUMOReal pos, SUMOReal speed, SUMOTime lane) :
		myPos(pos), mySpeed(speed), myTimeEnterLane(lane) {
}

/* -------------------------------------------------------------------------
 * methods of MSVehicle::Influencer
 * ----------------------------------------------------------------------- */
#ifndef NO_TRACI
MSVehicle::Influencer::Influencer() :
		mySpeedAdaptationStarted(true), myConsiderSafeVelocity(true), myConsiderMaxAcceleration(
				true), myConsiderMaxDeceleration(true) {
}

MSVehicle::Influencer::~Influencer(){}

void MSVehicle::Influencer::setSpeedTimeLine(
		const std::vector<std::pair<SUMOTime, SUMOReal> > &speedTimeLine) {
	mySpeedAdaptationStarted = true;
	mySpeedTimeLine = speedTimeLine;
}

void MSVehicle::Influencer::setLaneTimeLine
 (const std::vector<std::pair<SUMOTime, unsigned int> > &laneTimeLine)
{
  myLaneTimeLine = laneTimeLine;
}

SUMOReal MSVehicle::Influencer::influenceSpeed(SUMOTime currentTime,
		SUMOReal speed, SUMOReal vSafe, SUMOReal vMin, SUMOReal vMax) {
	// keep original speed
	myOriginalSpeed = speed;
	// remove leading commands which are no longer valid
	while (mySpeedTimeLine.size() == 1
			|| (mySpeedTimeLine.size() > 1
					&& currentTime > mySpeedTimeLine[1].first)) {
		mySpeedTimeLine.erase(mySpeedTimeLine.begin());
	}
	// do nothing if the time line does not apply for the current time
	if (mySpeedTimeLine.size() < 2 || currentTime < mySpeedTimeLine[0].first) {
		return speed;
	}
	// compute and set new speed
	if (!mySpeedAdaptationStarted) {
		mySpeedTimeLine[0].second = speed;
		mySpeedAdaptationStarted = true;
	}
	currentTime += DELTA_T;
	const SUMOReal td =
			STEPS2TIME(currentTime - mySpeedTimeLine[0].first)
					/ STEPS2TIME(mySpeedTimeLine[1].first + DELTA_T - mySpeedTimeLine[0].first);
	speed = mySpeedTimeLine[0].second
			- (mySpeedTimeLine[0].second - mySpeedTimeLine[1].second) * td;
	if (myConsiderSafeVelocity) {
		speed = MIN2(speed, vSafe);
	}
	if (myConsiderMaxAcceleration) {
		speed = MIN2(speed, vMax);
	}
	if (myConsiderMaxDeceleration) {
		speed = MAX2(speed, vMin);
	}
	return speed;
}

MSVehicle::ChangeRequest MSVehicle::Influencer::checkForLaneChanges(
		SUMOTime currentTime, const MSEdge& currentEdge,
		unsigned int currentLaneIndex) {
	// remove leading commands which are no longer valid
	while (myLaneTimeLine.size() == 1
			|| (myLaneTimeLine.size() > 1
					&& currentTime > myLaneTimeLine[1].first)) {
		myLaneTimeLine.erase(myLaneTimeLine.begin());
	}
	// do nothing if the time line does not apply for the current time
	if (myLaneTimeLine.size() < 2 || currentTime < myLaneTimeLine[0].first) {
		return REQUEST_NONE;
	}
	unsigned int destinationLaneIndex = myLaneTimeLine[1].second;
	if ((unsigned int) currentEdge.getLanes().size() <= destinationLaneIndex) {
		return REQUEST_NONE;
	}
	if (currentLaneIndex > destinationLaneIndex) {
		return REQUEST_RIGHT;
	} else if (currentLaneIndex < destinationLaneIndex) {
		return REQUEST_LEFT;
	} else {
		return REQUEST_HOLD;
	}
}

/*
void MSVehicle::Influencer::setConsiderSafeVelocity(bool value) {
  myConsiderSafeVelocity = value;
}
*/

static time_t now;
static time_t adasZroTimeTNow=0;
static time_t adasOneTimeTNow=0;
static time_t adasZroLastAdasUpdateTimeT=0;
static time_t adasOneLastAdasUpdateTimeT=0;
/*
static SUMOTime adasZroLastAdasUpdateTimeStep(string2time(0));
static SUMOTime adasOneLastAdasUpdateTimeStep(string2time(0));
*/


void MSVehicle::Influencer::setConsiderMaxAcceleration(bool value) {
  myConsiderMaxAcceleration = value;
}

void MSVehicle::Influencer::setConsiderMaxDeceleration(bool value) {
  myConsiderMaxDeceleration = value;
}
#endif

/* -------------------------------------------------------------------------
 * MSVehicle-methods
 * ----------------------------------------------------------------------- */
MSVehicle::~MSVehicle() {
	delete myLaneChangeModel;
	// other
	delete myEdgeWeights;
	for (std::vector<MSLane*>::iterator i = myFurtherLanes.begin();
			i != myFurtherLanes.end(); ++i) {
		(*i)->resetPartialOccupation(this);
	}
	for (DriveItemVector::iterator i = myLFLinkLanes.begin();
			i != myLFLinkLanes.end(); ++i) {
		if ((*i).myLink != 0) {
			(*i).myLink->removeApproaching(this);
		}
	}
	myFurtherLanes.clear();
	//
	if (myType->amVehicleSpecific()) {
		delete myType;
	}
#ifndef NO_TRACI
	delete myInfluencer;
#endif
}

MSVehicle::MSVehicle
 (SUMOVehicleParameter* pars,const MSRoute* route,const MSVehicleType* type,
  int /*vehicleIndex*//*,bool ecoGemRouter*/)
 :MSBaseVehicle(pars,route,type/*, ecoGemRouter*/),
  myLastLaneChangeOffset(0),myWaitingTime(0),
  myState(0,0,MSNet::getInstance()->getCurrentTimeStep()), 
  myOldState(0,0,MSNet::getInstance()->getCurrentTimeStep()),
  myOlderState(0,0,MSNet::getInstance()->getCurrentTimeStep()),
  myLane(0),myLastBestLanesEdge(0),myPersonDevice(0),myPreDawdleAcceleration(0),
  mySignals(0),myAmOnNet(false),myAmRegisteredAsWaitingForPerson(false),
  myEdgeWeights(0),myHaveToWaitOnNextLink(false),amAdasChecked(false),
  amAdas(false),myCurrentTraveledDistance(0.),lastAdasUpdateTimeT(false)
#ifndef NO_TRACI
  ,myInfluencer(0)
#endif
  ,hasChangeEdge(false),myLastEdgeId("null")
{
  lastAdasUpdateTimeT=new time_t();
  OptionsCont &oc = OptionsCont::getOptions();
    if(oc.getBuildVerbosity() > 1 ||
       oc.getSimulationVerbosity() > 1)
    {
      std::cout<<"MSVehicle::MSVehicle(...)\n"<<std::endl;
      std::cout << "  pars->id=" << pars->id << "\n"
                << "  pars->color=" << pars->color << std::endl;
    }
	for (std::vector<SUMOVehicleParameter::Stop>::iterator i =
			pars->stops.begin(); i != pars->stops.end(); ++i) {
		if (!addStop(*i)) {
			throw ProcessError(
					"Stop for vehicle '" + pars->id + "' on lane '" + i->lane
							+ "' is not downstream the current route.");
		}
	}
	for (std::vector<SUMOVehicleParameter::Stop>::const_iterator i =
			route->getStops().begin(); i != route->getStops().end(); ++i) {
		if (!addStop(*i)) {
			throw ProcessError(
					"Stop for vehicle '" + pars->id + "' on lane '" + i->lane
							+ "' is not downstream the current route.");
		}
	}
	const MSLane* const depLane = (*myCurrEdge)->getDepartLane(*this);
	if (depLane == 0) {
		throw ProcessError(
				"Invalid departlane definition for vehicle '" + pars->id
						+ "'.");
	}
	if (pars->departSpeedProcedure == DEPART_SPEED_GIVEN
			&& pars->departSpeed > depLane->getMaxSpeed()) {
		throw ProcessError(
				"Departure speed for vehicle '" + pars->id
						+ "' is too high for the departure lane '"
						+ depLane->getID() + "'.");
	}
	if (pars->departSpeedProcedure == DEPART_SPEED_GIVEN
			&& pars->departSpeed > type->getMaxSpeed()) {
		throw ProcessError(
				"Departure speed for vehicle '" + pars->id
						+ "' is too high for the vehicle type '" + type->getID()
						+ "'.");
	}
#ifdef _MESSAGES
  myLCMsgEmitter = MSNet::getInstance()->getMsgEmitter("lanechange");
  myBMsgEmitter = MSNet::getInstance()->getMsgEmitter("break");
  myHBMsgEmitter = MSNet::getInstance()->getMsgEmitter("heartbeat");
#endif
  myLaneChangeModel = new MSLCM_DK2004(*this);
  myCFVariables = type->getCarFollowModel().createVehicleVariables();  
  /* scenload catching */
  if(oc.getBuildVerbosity() > 1 ||
       (oc.getSimulationVerbosity() > 1))
    std::cout<<"attempting to catch vehicles to track"<<std::endl;
  if(VehicleToTrack::getVehiclesToTrackWithCharacteristicsList() &&
     VehicleToTrack::getVehiclesToTrackWithCharacteristicsList()->size() > 0)
  {
    if(oc.getBuildVerbosity() > 1 ||
       (oc.getSimulationVerbosity() > 1))
      std::cout<<"catched vehicles to track"<<std::endl;
    /*
    for(int i=0; i<VehicleToTrack::getVehiclesToTrackWithCharacteristicsList().size(); i++)
    {
      if(VehicleToTrack::getVehiclesToTrackWithCharacteristicsList()->at(i)->getIdentifier() == getID())
      {
      */
        /* TODO PROCESS */
        /*this->getMyType()->getCooperLosses();*/
        ;
      /*
      }
    }
    */
  }
  else
  {
    if(oc.getBuildVerbosity() > 1 ||
       (oc.getSimulationVerbosity() > 1))
      std::cout<<"no vehicles to track catched"<<std::endl;
  }
}

void MSVehicle::onRemovalFromNet(const MSMoveReminder::Notification reason)
{
	workOnMoveReminders(myState.myPos - SPEED2DIST(myState.mySpeed),
			myState.myPos, myState.mySpeed);
	for (DriveItemVector::iterator i = myLFLinkLanes.begin();
			i != myLFLinkLanes.end(); ++i) {
		if ((*i).myLink != 0) {
			(*i).myLink->removeApproaching(this);
		}
	}
	leaveLane(reason);
}

// ------------ interaction with the route
bool MSVehicle::ends() const {
	return myCurrEdge == myRoute->end() - 1
			&& myState.myPos > myArrivalPos - POSITION_EPS;
}

bool MSVehicle::replaceRoute(const MSRoute* newRoute, bool onInit) {
	const MSEdgeVector& edges = newRoute->getEdges();
	// assert the vehicle may continue (must not be "teleported" or whatever to another position)
	if (!onInit && !newRoute->contains(*myCurrEdge)) {
		return false;
	}

	// rebuild in-vehicle route information
	if (onInit) {
		myCurrEdge = newRoute->begin();
	} else {
		myCurrEdge = find(edges.begin(), edges.end(), *myCurrEdge);
	}
	// check whether the old route may be deleted (is not used by anyone else)
	newRoute->addReference();
	myRoute->release();
	// assign new route
	myRoute = newRoute;
	myLastBestLanesEdge = 0;
	// update arrival definition
	calculateArrivalPos();
	// save information that the vehicle was rerouted
	myNumberReroutes++;
	MSNet::getInstance()->informVehicleStateListener(this,
			MSNet::VEHICLE_STATE_NEWROUTE);
	// recheck stops
	for (std::list<Stop>::iterator iter = myStops.begin();
			iter != myStops.end();) {
		if (find(edges.begin(), edges.end(), &iter->lane->getEdge())
				== edges.end()) {
			iter = myStops.erase(iter);
		} else {
			iter->edge = find(edges.begin(), edges.end(),
					&iter->lane->getEdge());
			++iter;
		}
	}
	return true;
}

bool MSVehicle::willPass(const MSEdge* const edge) const {
	return find(myCurrEdge, myRoute->end(), edge) != myRoute->end();
}

MSEdgeWeightsStorage&
MSVehicle::getWeightsStorage() {
	if (myEdgeWeights == 0) {
		myEdgeWeights = new MSEdgeWeightsStorage();
	}
	return *myEdgeWeights;
}

// ------------ Interaction with move reminders
void MSVehicle::workOnMoveReminders(SUMOReal oldPos, SUMOReal newPos,
		SUMOReal newSpeed) {
	// This erasure-idiom works for all stl-sequence-containers
	// See Meyers: Effective STL, Item 9
	for (MoveReminderCont::iterator rem = myMoveReminders.begin();
			rem != myMoveReminders.end();) {
		if (!rem->first->notifyMove(*this, oldPos + rem->second,
				newPos + rem->second, newSpeed)) {
			rem = myMoveReminders.erase(rem);
		} else {
			++rem;
		}
	}
}

void MSVehicle::adaptLaneEntering2MoveReminder(const MSLane& enteredLane) {
	// save the old work reminders, patching the position information
	//  add the information about the new offset to the old lane reminders
	const SUMOReal oldLaneLength = myLane->getLength();
	for (MoveReminderCont::iterator rem = myMoveReminders.begin();
			rem != myMoveReminders.end(); ++rem) {
		rem->second += oldLaneLength;
	}
	for (std::vector<MSMoveReminder*>::const_iterator rem =
			enteredLane.getMoveReminders().begin();
			rem != enteredLane.getMoveReminders().end(); ++rem) {
		addReminder(*rem);
	}
}

// ------------ Other getter methods
Position MSVehicle::getPosition() const {
	if (myLane == 0) {
		return Position(-1000, -1000);
	}
	return myLane->getShape().positionAtLengthPosition(myState.pos());
}

/*
NMEA to decimal
latitude  - 0302.78469  - 03 + (02.78469/60) = 3.046412
longitude - 10141.82531 - 101 + 41.82531/60) = 101.6971
*/

SUMOReal decimalToNmea(SUMOReal degrees){
  int deg=(int)degrees;
  degrees=(degrees-deg)*60;
  int min=(int)degrees;
  degrees=degrees-min;
  return ((deg*100)+min+degrees);
}

SUMOReal MSVehicle::getAngle() const {
	Position p1 = myLane->getShape().positionAtLengthPosition(myState.pos());
	Position p2 =
			myFurtherLanes.size() > 0 ?
					myFurtherLanes.front()->getShape().positionAtLengthPosition(
							myFurtherLanes.front()->getPartialOccupatorEnd()) :
					myLane->getShape().positionAtLengthPosition(
							myState.pos() - myType->getLengthWithGap());
	if (p1 != p2) {
		return atan2(p1.x() - p2.x(), p2.y() - p1.y()) * 180. / PI;
	} else {
		return -myLane->getShape().rotationDegreeAtLengthPosition(
				getPositionOnLane());
	}
}

// ------------
bool MSVehicle::addStop(const SUMOVehicleParameter::Stop& stopPar,
		SUMOTime untilOffset) {
	Stop stop;
	stop.lane = MSLane::dictionary(stopPar.lane);
	stop.busstop = MSNet::getInstance()->getBusStop(stopPar.busstop);
	stop.startPos = stopPar.startPos;
	stop.endPos = stopPar.endPos;
	stop.duration = stopPar.duration;
	stop.until = stopPar.until;
	if (stop.until != -1) {
		stop.until += untilOffset;
	}
	stop.triggered = stopPar.triggered;
	stop.parking = stopPar.parking;
	stop.reached = false;
	if (stop.startPos < 0 || stop.endPos > stop.lane->getLength()) {
		return false;
	}
	stop.edge = find(myCurrEdge, myRoute->end(), &stop.lane->getEdge());
	MSRouteIterator prevStopEdge = myCurrEdge;
	SUMOReal prevStopPos = myState.myPos;
	// where to insert the stop
	std::list<Stop>::iterator iter = myStops.begin();
	if (stopPar.index == STOP_INDEX_END
			|| stopPar.index >= static_cast<int>(myStops.size())) {
		if (myStops.size() > 0) {
			prevStopEdge = myStops.back().edge;
			prevStopPos = myStops.back().endPos;
			iter = myStops.end();
			stop.edge = find(prevStopEdge, myRoute->end(),
					&stop.lane->getEdge());
		}
	} else {
		if (stopPar.index == STOP_INDEX_FIT) {
			while (iter != myStops.end()
					&& (iter->edge < stop.edge
							|| (iter->endPos < stop.endPos
									&& iter->edge == stop.edge))) {
				prevStopEdge = iter->edge;
				prevStopPos = iter->endPos;
				++iter;
			}
		} else {
			int index = stopPar.index;
			while (index > 0) {
				prevStopEdge = iter->edge;
				prevStopPos = iter->endPos;
				++iter;
				--index;
			}
			stop.edge = find(prevStopEdge, myRoute->end(),
					&stop.lane->getEdge());
		}
	}
	if (stop.edge == myRoute->end() || prevStopEdge > stop.edge
			|| prevStopEdge == stop.edge && prevStopPos > stop.endPos) {
		return false;
	}
	if (myCurrEdge == stop.edge
			&& myState.myPos
					> stop.endPos
							- getCarFollowModel().brakeGap(myState.mySpeed)) {
		return false;
	}
	myStops.insert(iter, stop);
	return true;
}

bool MSVehicle::isStopped() const {
	return !myStops.empty() && myStops.begin()->reached;
}

bool MSVehicle::isParking() const {
	return isStopped() && myStops.begin()->parking;
}

SUMOReal MSVehicle::processNextStop(SUMOReal currentVelocity) {
	if (myStops.empty()) {
		// no stops; pass
		return currentVelocity;
	}
	Stop& stop = myStops.front();
	if (stop.reached) {
		// ok, we have already reached the next stop
		// any waiting persons may board now
		bool boarded = MSNet::getInstance()->getPersonControl().boardAnyWaiting(
				&myLane->getEdge(), this);
		if (boarded) {
			// the triggering condition has been fulfilled. Maybe we want to wait a bit longer for additional riders (car pooling)
			stop.triggered = false;
			if (myAmRegisteredAsWaitingForPerson) {
				MSNet::getInstance()->getVehicleControl().unregisterOneWaitingForPerson();
				myAmRegisteredAsWaitingForPerson = false;
			}
		}
		if (stop.duration <= 0 && !stop.triggered) {
			// we have waited long enough and fulfilled any passenger-requirements
			if (stop.busstop != 0) {
				// inform bus stop about leaving it
				stop.busstop->leaveFrom(this);
			}
			// the current stop is no longer valid
			MSNet::getInstance()->getVehicleControl().removeWaiting(
					&myLane->getEdge(), this);
			myStops.pop_front();
			// do not count the stopping time towards gridlock time.
			// Other outputs use an independent counter and are not affected.
			myWaitingTime = 0;
			// maybe the next stop is on the same edge; let's rebuild best lanes
			getBestLanes(true);
			// continue as wished...
		} else {
			// we have to wait some more time
			if (stop.triggered && !myAmRegisteredAsWaitingForPerson) {
				// we can only register after waiting for one step. otherwise we might falsely signal a deadlock
				MSNet::getInstance()->getVehicleControl().registerOneWaitingForPerson();
				myAmRegisteredAsWaitingForPerson = true;
			}
			stop.duration -= DELTA_T;
			return 0;
		}
	} else {
		// is the next stop on the current lane?
		if (stop.edge == myCurrEdge) {
			// get the stopping position
			SUMOReal endPos = stop.endPos;
			bool busStopsMustHaveSpace = true;
			if (stop.busstop != 0) {
				// on bus stops, we have to wait for free place if they are in use...
				endPos = stop.busstop->getLastFreePos();
				if (endPos - 5. < stop.busstop->getBeginLanePosition()) { // !!! explicit offset
					busStopsMustHaveSpace = false;
				}
			}
			if (myState.pos() >= endPos - BUS_STOP_OFFSET
					&& busStopsMustHaveSpace) {
				// ok, we may stop (have reached the stop)
				stop.reached = true;
				MSNet::getInstance()->getVehicleControl().addWaiting(
						&myLane->getEdge(), this);
				// compute stopping time
				if (stop.until >= 0) {
					if (stop.duration == -1) {
						stop.duration = stop.until
								- MSNet::getInstance()->getCurrentTimeStep();
					} else {
						stop.duration =
								MAX2(stop.duration,
										stop.until
												- MSNet::getInstance()->getCurrentTimeStep());
					}
				}
				if (stop.busstop != 0) {
					// let the bus stop know the vehicle
					stop.busstop->enter(this, myState.pos(),
							myState.pos() - myType->getLengthWithGap());
				}
			}
			// decelerate
			// !!! should not v be 0 when we have reached the stop?
			return getCarFollowModel().stopSpeed(this, endPos - myState.pos());
		}
	}
	return currentVelocity;
}

bool MSVehicle::moveRegardingCritical(SUMOTime t, const MSLane* const lane,
		const MSVehicle* const pred, const MSVehicle* const neigh,
		SUMOReal lengthsInFront) {
#ifdef _MESSAGES
	if (myHBMsgEmitter != 0) {
		if (isOnRoad()) {
			SUMOReal timeStep = MSNet::getInstance()->getCurrentTimeStep();
			myHBMsgEmitter->writeHeartBeatEvent(myParameter->id, timeStep, myLane, myState.pos(), myState.speed(), getPosition().x(), getPosition().y());
		}
	}
#endif
#ifdef DEBUG_VEHICLE_GUI_SELECTION
	if (gSelected.isSelected(GLO_VEHICLE, static_cast<const GUIVehicle*>(this)->getGlID())) {
		int bla = 0;
	}
#endif
	// remove information about approaching links, will be reset later in this step
	for (DriveItemVector::iterator i = myLFLinkLanes.begin();
			i != myLFLinkLanes.end(); ++i) {
		if ((*i).myLink != 0) {
			(*i).myLink->removeApproaching(this);
		}
	}
	myLFLinkLanes.clear();
	//
	const MSCFModel& cfModel = getCarFollowModel();
	SUMOReal vBeg = MIN2(cfModel.maxNextSpeed(myState.mySpeed),
			lane->getMaxSpeed());
	// check whether the vehicle is not on an appropriate lane
	bool onAppropriateLane = myLane->appropriate(this);
	if (!onAppropriateLane) {
		// decelerate to lane end when yes
		vBeg = MIN2(
				cfModel.stopSpeed(this, myLane->getLength() - myState.myPos),
				myLane->getMaxSpeed());
	}
	if (myCurrEdge == myRoute->end() - 1) {
		if (myParameter->arrivalSpeedProcedure == ARRIVAL_SPEED_GIVEN) {
			vBeg = MIN2(
					cfModel.freeSpeed(this, getSpeed(),
							myArrivalPos - myState.myPos,
							myParameter->arrivalSpeed), myLane->getMaxSpeed());
		} else {
			vBeg = myLane->getMaxSpeed();
		}
	}
	// interaction with leader
	if (pred != 0) {
		// interaction with leader if one exists on same lane
		SUMOReal gap = gap2pred(*pred);
		if (MSGlobals::gCheck4Accidents && gap < 0) {
			// collision occured!
			return true;
		}
		vBeg = MIN2(vBeg,
				cfModel.followSpeed(this, getSpeed(), gap2pred(*pred),
						pred->getSpeed(),
						pred->getCarFollowModel().getMaxDecel()));
	} else {
		// (potential) interaction with a vehicle extending partially into this lane
		MSVehicle* predP = myLane->getPartialOccupator();
		if (predP != 0 && predP != this) {
			SUMOReal gap = myLane->getPartialOccupatorEnd() - myState.myPos;
			if (MSGlobals::gCheck4Accidents && gap < 0) {
				// collision occured!
				return true;
			}
			vBeg = MIN2(vBeg,
					cfModel.followSpeed(this, getSpeed(), gap,
							predP->getSpeed(),
							predP->getCarFollowModel().getMaxDecel()));
		}
	}
	// interaction with left-lane leader (do not overtake right)
	cfModel.leftVehicleVsafe(this, neigh, vBeg);
	// check whether the vehicle wants to stop somewhere
	if (!myStops.empty()
			&& &myStops.begin()->lane->getEdge() == &lane->getEdge()) {
		const Stop& stop = *myStops.begin();
		SUMOReal stopPos =
				stop.busstop == 0 ?
						stop.endPos :
						stop.busstop->getLastFreePos() - POSITION_EPS;
		SUMOReal seen = lane->getLength() - myState.pos();
		SUMOReal vsafeStop = cfModel.stopSpeed(this,
				seen - (lane->getLength() - stopPos));
		vBeg = MIN2(vBeg, vsafeStop);
	}
	vBeg = MAX2((SUMOReal) 0, vBeg);
#ifdef _DEBUG
	if (vBeg < cfModel.getSpeedAfterMaxDecel(myState.mySpeed)) {
		WRITE_WARNING("Vehicle '" + getID() + "' is decelerating too much (#1; is: " + toString(myState.mySpeed - vBeg) + ", may: " + toString(cfModel.getSpeedAfterMaxDecel(myState.mySpeed)) + ")");
	}
#endif
	if (!onAppropriateLane) {
		myLFLinkLanes.push_back(
				DriveProcessItem(0, vBeg, vBeg, false, 0, 0,
						myLane->getLength() - myState.myPos));
	} else {
		// check whether the driver wants to let someone in
		vsafeCriticalCont(t, vBeg);
		checkRewindLinkLanes(lengthsInFront);
	}
	return false;
}

bool MSVehicle::moveChecked()
{
  OptionsCont &oc = OptionsCont::getOptions();
  if(oc.getSimulationVerbosity() > 1)
  {
    std::cout << "----> bool MSVehicle::moveChecked()" << std::endl;
    if(oc.getSimulationVerbosity() > 2)
      std::cout << "  this->getID()=" << this->getID() << std::endl;
  }
#ifdef DEBUG_VEHICLE_GUI_SELECTION
	if (gSelected.isSelected(GLO_VEHICLE, static_cast<const GUIVehicle*>(this)->getGlID())) {
		int bla = 0;
	}
#endif
	// get vsafe
	SUMOReal vSafe = 0;
	myHaveToWaitOnNextLink = false;
	assert(myLFLinkLanes.size() != 0);
	DriveItemVector::iterator i;
	bool braking = false;
	bool lastWasGreenCont = false;
	//Added by Hi-Iberia
	MSLane* lastLane = myLane;

  if(oc.getSimulationVerbosity() > 2)
    std::cout << "[100] bool MSVehicle::moveChecked()" << std::endl;
  
	myState.setMyPosOnLane(myState.myPos);
	for (i = myLFLinkLanes.begin(); i != myLFLinkLanes.end(); ++i) {
		MSLink* link = (*i).myLink;
		// the vehicle must change the lane on one of the next lanes
		if (link != 0 && (*i).mySetRequest) {
			const LinkState ls = link->getState();
			// vehicles should brake when running onto a yellow light if the distance allows to halt in front
			const bool yellow = ls == LINKSTATE_TL_YELLOW_MAJOR
					|| ls == LINKSTATE_TL_YELLOW_MINOR;
			if (yellow
					&& (*i).myDistance
							> getCarFollowModel().getSpeedAfterMaxDecel(
									myState.mySpeed)) {
				vSafe = (*i).myVLinkWait;
				braking = true;
				lastWasGreenCont = false;
				link->removeApproaching(this);
				break;
			}
			//
			const bool opened = link->opened((*i).myArrivalTime,
					(*i).myArrivalSpeed, getVehicleType().getLengthWithGap());
			// vehicles should decelerate when approaching a minor link
			if (opened && !lastWasGreenCont && !link->havePriority()
					&& (*i).myDistance > getCarFollowModel().getMaxDecel()) {
				vSafe = (*i).myVLinkWait;
				braking = true;
				lastWasGreenCont = false;
				if (ls == LINKSTATE_EQUAL) {
					link->removeApproaching(this);
				}
				break; // could be revalidated
			}
			// have waited; may pass if opened...
			if (opened) {
				vSafe = (*i).myVLinkPass;
				lastWasGreenCont = link->isCont()
						&& (ls == LINKSTATE_TL_GREEN_MAJOR);
			} else {
				lastWasGreenCont = false;
				vSafe = (*i).myVLinkWait;
				braking = true;
				if (ls == LINKSTATE_EQUAL) {
					link->removeApproaching(this);
				}
				break;
			}
		} else {
			vSafe = (*i).myVLinkWait;
			braking = true;
			break;
		}
	}

  if(oc.getSimulationVerbosity() > 2)
    std::cout << "[200] bool MSVehicle::moveChecked()" << std::endl;  

	if (braking) {
		myHaveToWaitOnNextLink = true;
	}

	SUMOReal vNext = getCarFollowModel().moveHelper(this, vSafe);
  
  /*   Added by Hi-Iberia (emartin or doancea) Following the same approach
   * followed by the article "Electric Vehicle Simulator for energy Consumption
   * Studies in Electric Mobility Systems"
   */
  MSDevice_FEV *aux1 = NULL;
  for(std::vector < MSDevice* > ::const_iterator it = getDevices().begin();
      it != getDevices().end(); ++it)
  {
    aux1 = dynamic_cast < MSDevice_FEV* > (*it);
    if(aux1)
      break;
  }
  
  if(oc.getSimulationVerbosity() > 2)
    std::cout << "[300] bool MSVehicle::moveChecked()" << std::endl;    
  
  /*   Now aux1 should have a MSDevice_FEV, but that was only when all vehicles
   * had that device, which was an error, because electric and normal vehicles
   * should coexist in the simulation
   */
  SUMOReal maxSpeedByTorque;
  if(aux1)
  {
    // Electric vechicles
    maxSpeedByTorque = aux1->computeMaxSpeed(vNext);
    if(oc.getSimulationVerbosity() > 2)
      std::cout << "[310] bool MSVehicle::moveChecked()" << std::endl;    
    if(maxSpeedByTorque < 0)
    {
      if(oc.getSimulationVerbosity() > 2)
        std::cout << "[320] bool MSVehicle::moveChecked()" << std::endl;    
      maxSpeedByTorque = vNext;
    }
  }
  else
  {
    // Nonelectric vehicles
    /*
    std::cout << "  CRITICAL ERROR asking for the MSDevice_FEV of a non FEV veh"
                 "icle in file " << __FILE__ << ":" << __LINE__ << std::endl
              << "  Defaulting the maxSpeedByTorque variable value" << std::endl
              ;
    maxSpeedByTorque = 0;
    */
  }
  if(oc.getSimulationVerbosity() > 2)
    std::cout << "[330] bool MSVehicle::moveChecked()" << std::endl;    
  vNext = MAX2(vNext, (SUMOReal) 0.);    
  if(oc.getSimulationVerbosity() > 2)
    std::cout << "[400] bool MSVehicle::moveChecked()" << std::endl;      
#ifndef NO_TRACI
	if (myInfluencer != 0) {
		SUMOReal vMin = MAX2(SUMOReal(0),
				getVehicleType().getCarFollowModel().getSpeedAfterMaxDecel(
						myState.mySpeed));
		SUMOReal vMax = getVehicleType().getCarFollowModel().maxNextSpeed(
				myState.mySpeed);
		vNext = myInfluencer->influenceSpeed(
				MSNet::getInstance()->getCurrentTimeStep(), vNext, vSafe, vMin,
				vMax);
	}
#endif
	// visit waiting time
	if (vNext <= 0.1) {
		myWaitingTime += DELTA_T;
		braking = true;
	} else {
		myWaitingTime = 0;
	}
	if (myState.mySpeed < vNext) {
		braking = false;
	}
	if (braking) {
		switchOnSignal(VEH_SIGNAL_BRAKELIGHT);
	} else {
		switchOffSignal(VEH_SIGNAL_BRAKELIGHT);
	}
	// call reminders after vNext is set
	SUMOReal pos = myState.myPos;
  
  if(aux1)
    // Electric vehicle
    vNext = MIN3(vNext, getMaxSpeed(), maxSpeedByTorque);
  else
    // Nonelectric vehicle
    vNext = MIN2(vNext, getMaxSpeed());
  
  if(oc.getSimulationVerbosity() > 2)
    std::cout << "[400] bool MSVehicle::moveChecked()" << std::endl;    

#ifdef _MESSAGES
	if (myHBMsgEmitter != 0) {
		if (isOnRoad()) {
			SUMOReal timeStep = MSNet::getInstance()->getCurrentTimeStep();
			myHBMsgEmitter->writeHeartBeatEvent(myParameter->id, timeStep, myLane, myState.pos(), myState.speed(), getPosition().x(), getPosition().y());
		}
	}
	if (myBMsgEmitter != 0) {
		if (vNext < myState.mySpeed) {
			SUMOReal timeStep = MSNet::getInstance()->getCurrentTimeStep();
			myBMsgEmitter->writeBreakEvent(myParameter->id, timeStep, myLane, myState.pos(), myState.speed(), getPosition().x(), getPosition().y());
		}
	}
#endif
	// update position and speed
	myState.addToMyPos(SPEED2DIST(vNext));
	myState.mySpeed = vNext;
	std::vector<MSLane*> passedLanes;
	for (std::vector<MSLane*>::reverse_iterator i = myFurtherLanes.rbegin();
			i != myFurtherLanes.rend(); ++i) {
		passedLanes.push_back(*i);
	}
	if (passedLanes.size() == 0 || passedLanes.back() != myLane) {
		passedLanes.push_back(myLane);
	}

  if(oc.getSimulationVerbosity() > 2)
    std::cout << "[500] bool MSVehicle::moveChecked()" << std::endl;  

  bool moved = true;
  // move on lane(s)
  if (myState.myPos <= myLane->getLength()) {
		// we are staying at our lane
		//  there is no need to go over succeeding lanes
		workOnMoveReminders(pos, pos + SPEED2DIST(vNext), vNext);
		moved = false;
	} else {
		// we are moving at least to the next lane (maybe pass even more than one)
		if (myCurrEdge != myRoute->end() - 1) {
			//Added by HI-Iberia to track those vehicles that have changed their current edge.
			MSLane* approachedLane = myLane;
			// move the vehicle forward
			for (i = myLFLinkLanes.begin();
					i != myLFLinkLanes.end() && approachedLane != 0
							&& myState.myPos > approachedLane->getLength();
					++i) {
				leaveLane(MSMoveReminder::NOTIFICATION_JUNCTION);
				MSLink* link = (*i).myLink;
				// check whether the vehicle was allowed to enter lane
				//  otherwise it is decelerated and we do not need to test for it's
				//  approach on the following lanes when a lane changing is performed
				myState.myPos -= approachedLane->getLength();
				assert(myState.myPos > 0);
				// proceed to the next lane
				if (link != 0) {
#ifdef HAVE_INTERNAL_LANES
					approachedLane = link->getViaLane();
					if (approachedLane == 0) {
						approachedLane = link->getLane();
					}
#else
					approachedLane = link->getLane();
#endif
				} else {
					approachedLane = 0;
				}
				if (approachedLane != myLane && approachedLane != 0) {
					enterLaneAtMove(approachedLane);
					myLane = approachedLane;
					if (approachedLane->getEdge().isVaporizing()) {
						break;
					}
				}
        passedLanes.push_back(approachedLane);
      }
    }
  }

  if(oc.getSimulationVerbosity() > 2)
    std::cout << "[600] bool MSVehicle::moveChecked()" << std::endl;  

	for (std::vector<MSLane*>::iterator i = myFurtherLanes.begin();
			i != myFurtherLanes.end(); ++i) {
		(*i)->resetPartialOccupation(this);
	}
	myFurtherLanes.clear();
	if (!ends()) {
		if (myState.myPos - getVehicleType().getLengthWithGap() < 0
				&& passedLanes.size() > 0) {
			SUMOReal leftLength = getVehicleType().getLengthWithGap()
					- myState.myPos;
			std::vector<MSLane*>::reverse_iterator i = passedLanes.rbegin() + 1;
			while (leftLength > 0 && i != passedLanes.rend()) {
				myFurtherLanes.push_back(*i);
				leftLength -= (*i)->setPartialOccupation(this, leftLength);
				++i;
			}
		}
		setBlinkerInformation();
	}

  if(oc.getSimulationVerbosity() > 2)
    std::cout << "[700] bool MSVehicle::moveChecked()" << std::endl;
  
  /* Update myTraveledDistance */
  if(myCurrentTraveledDistance == 0.)
  {
    if(oc.getSimulationVerbosity() > 2)
      std::cout << "beginning the trip" << std::endl;
    /* Beginning the trip */
    myCurrentTraveledDistance = .000000001 /*getPositionOnLane()*/; /* FIXME IF DEADLINE ALLOWS IT BECAUSE THIS IS Q&D */
    myLastTraveledDistance = 0.;
  }
  else
  {
    /* Detect edge change */
    if(myLane->getEdge().getID().compare(myLastEdgeId) != 0)
    {
      if(oc.getSimulationVerbosity())
      {
        /*
        std::cout << "myLane->getLength()=" << myLane->getLength() << std::endl;
        ss
        std::cout << "myLane->getgetLength()=" << myLane->getLength() << std::endl;
        */
      }
      //myLastTraveledDistance += myLane->getLength();
      myLastTraveledDistance = myCurrentTraveledDistance;
      if(oc.getSimulationVerbosity() > 2)
      {
        std::cout << "myLastTraveledDistance=" << myLastTraveledDistance << std::endl;
        std::cout << "getPositionOnLane()=" << getPositionOnLane() << std::endl;
      }
      myCurrentTraveledDistance = myLastTraveledDistance + getPositionOnLane();
      if(oc.getSimulationVerbosity() > 2)
        std::cout << "myCurrentTraveledDistance=" << myCurrentTraveledDistance << std::endl;
    }
    else
    {
      /*
      if(oc.getSimulationVerbosity())
        std::cout << std::endl << myCurrentTraveledDistance << " " << getPositionOnLane() << " " << myLastEdgeId.length()
          << " " << getOldPositionOnLane() << " " << myLane->getLength() << std::endl;
        */
      myCurrentTraveledDistance += getPositionOnLane();
      myCurrentTraveledDistance -= getOldPositionOnLane();
      if(oc.getSimulationVerbosity() > 2)
        std::cout << "myCurrentTraveledDistance=" << myCurrentTraveledDistance << std::endl;
    }
  }
  //std::cout << "myCurrentTraveledDistance=" << myCurrentTraveledDistance << std::endl << std::endl;

	//std::cout << "\n MoveChecked A all " << getID()<<"\t" << time2string(MSNet::getInstance()->getCurrentTimeStep()) << "\t" << getMyLane()->getEdge().getID() << "\t" << getMyLastEdgeId()<<"\n";
	//std::cout << "\n MoveChecked B " << getID()<<"\t" << time2string(MSNet::getInstance()->getCurrentTimeStep()) << "\t" << getMyLane()->getEdge().getID() << "\t" << getMyLastEdgeId()<<"\n";
	//Code added by HI-Iberia
	//For synthetic data generation is required that each vehicles
	//dumps their speed and other parameters whenever each car reach a new edge
	//the next lane is approachedLane and myLane is my current lane
	// clear previously set information
  /*
  if(aux1)
  {
  */
    if(oc.getSimulationVerbosity() > 2)
      std::cout << "[701] bool MSVehicle::moveChecked()" << std::endl;
    /* Detect edge change */
    if(myLane->getEdge().getID().compare(myLastEdgeId) != 0)
    {
      /* edge change */
      if(oc.getSimulationVerbosity() > 2)
        std::cout << "[702] bool MSVehicle::moveChecked()" << std::endl;
      /* Seems to be because of ending trips */
      if(myLastEdgeId.compare("null") != 0)
      {
        if(oc.getSimulationVerbosity() > 2)
          std::cout << "[703] bool MSVehicle::moveChecked()" << std::endl;
        /*
        Same as if(this->isFev())
        */
        if((aux1)||
           ((strcmp(getID().c_str(),"fev0")==0)||
            (strcmp(getID().c_str(),"fev1")==0)))
        {
          writeFevSyntheticData(lastLane);
        }
        if(aux1)
        {
          if(oc.getSimulationVerbosity() > 2)
            std::cout << "[704] bool MSVehicle::moveChecked()" << std::endl;
          /* Send VDADASRP info to the meant web service */
          if(OptionsCont::getOptions().isSet("ARFTmode"))
          {
            if(oc.getSimulationVerbosity() > 2)
              std::cout<<"ARFTmode is set"<<std::endl;            
            goto tag1;            
            if(!amAdasChecked)
            {
              std::string str, dupString;
              char *strtok_r_tok1, *strtok_r_tok2, *vehicleString,
                *webServiceUrl, *adasRp;
              /* Check amAdas condition */
              str = OptionsCont::getOptions().getString("ARFTmode");
              /* Loop over vehicles */
              dupString = strdup(str.c_str());
              std::cout<<"dupString="<<dupString<<std::endl;
              vehicleString = strtok_r((char *)dupString.c_str(),",",&strtok_r_tok1);
              std::cout<<"vehicleString="<<vehicleString<<std::endl;
              /* while((vehicleString.c_str() != NULL)&&(!amAdasChecked)) --> .c_str() segfaults */
              //while((vehicleString!=NULL)&&(!amAdasChecked))
              while((vehicleString!=NULL))
              {
                std::string dupVehicleString;
                char *vehicleId;
                /* Still having vehicles to check if this is it */
                dupVehicleString = strdup(vehicleString);
                std::cout<<"dupVehicleString="<<dupVehicleString<<std::endl;
                vehicleId = strtok_r((char *)dupVehicleString.c_str(),"@",&strtok_r_tok2);
                std::cout<<"{ADAS vehicleId="<<vehicleId<<"}"<<std::endl;
                if( /* getID()==vehicleId */
                     strcmp(getID().c_str(),vehicleId/*.c_str()*/))
                {
                  /* Keep eating tokens about me (about this Vehicle) */
                  adasRp = strtok_r(NULL,"@",&strtok_r_tok2);
                  std::cout<<"{ADAS adasRp="<<adasRp<<"}"<<std::endl;
                  if(adasRp)
                  {
                    myAdasRp = adasRp;
                    std::cout<<"{ADAS myAdasRp=}"<<myAdasRp<<"}"<<std::endl;
                  }
                  else
                    /* input error */
                    ;
                  webServiceUrl = strtok_r(NULL,"@",&strtok_r_tok2);
                  if(webServiceUrl)
                  {
                    myWebServiceUrl = webServiceUrl;
                    myAdasRp=adasRp;
                    std::cout<<"{ADAS myWebServiceUrl="<<myWebServiceUrl<<"}"<<std::endl;
                  }
                  else
                    /* input error */
                    ;
                  /* Vehicle in list of ADASes */
                  if(oc.getSimulationVerbosity() > 2)
                    std::cout << "[802] SUMO detected a correct ADAS service request and set that into the vehicle"
                              << std::endl;
                  amAdas = true;
                  amAdasChecked = true;
                  std::string traceString="echo '";
                  traceString+=vehicleId;
                  traceString+=" is adas (";
                  traceString+=adasRp;
                  traceString+=")' >> COMMAND_TRACE";
                  system(traceString.c_str());
                }
                /* Ask for the next vehicle token */
                std::cout<<"vehicleString="<<vehicleString<<std::endl;
                vehicleString = strtok_r(NULL,",",&strtok_r_tok1);
                std::cout<<"vehicleString="<<vehicleString<<std::endl;
              }
              if(!amAdasChecked)
              {
                /*   Loop ended not finding this vehicle in the ADASes list:
                 * Make the negative check
                 */
                amAdas = false;
                amAdasChecked = true;
              }
            }            
            goto tag2;            
          tag1:
            //goto tag2;
            if(!amAdasChecked)
            {
              if      (strcmp(getID().c_str(),"fev0")==0){
                //std::cout<<"fev0"<<std::endl;
                amAdasChecked=true;
                amAdas=true;
                //myAdasRp=myWebServiceUrl="http://192.168.1.20:8792/VDAdas?wsdl"; // DEPRECATED
                //myAdasRp=myWebServiceUrl="http://93.62.202.221/VDAdas?wsdl"; // DEPRECATED
                
                // LOCAL ITS/BXL
                myWebServiceUrl="http://192.168.1.20:8792/VDAdas?wsdl";
                // REMOTE ITALY
                //myWebServiceUrl="http://93.62.202.221/VDAdas?wsdl";
                
              }else if(strcmp(getID().c_str(),"fev1")==0){
                //std::cout<<"fev1"<<std::endl;
                amAdasChecked=true;
                amAdas=true;
                //myAdasRp=myWebServiceUrl="http://192.168.1.10:8792/VDAdas?wsdl"; // DEPRECATED
                //myAdasRp=myWebServiceUrl="http://93.62.202.221/VDAdas?wsdl"; // DEPRECATED
                
                // LOCAL ITS/BXL
                myWebServiceUrl="http://192.168.1.10:8792/VDAdas?wsdl";
                // REMOTE GREECE
                //myWebServiceUrl="http://147.102.7.30:8792/VDAdas?wsdl";
                
              }else{
                //std::cout<<getID()<<std::endl;
                amAdasChecked=true;
                amAdas=false;
              }
            }
          tag2:
            if(amAdas==true)
            {
              if(oc.getSimulationVerbosity() > 2)
                std::cout<<"[803] amAdas"<<std::endl;
              
              //std::cout<<"check2"<<std::endl;
              adasWriteInfo();
              
            }
          }
        }
        else
        {
          writeNonFevSyntheticData(lastLane);
          //adasWriteGpsString(); // TODO TASK FIXME ALERT DANGER BYPASS THIS IN FINAL VERSION !!!
        }
      }
      if(oc.getSimulationVerbosity() > 2)
        std::cout << "[804] bool MSVehicle::moveChecked()" << std::endl;        
      myState.setEnterLane(MSNet::getInstance()->getCurrentTimeStep());
      if(oc.getSimulationVerbosity() > 2)
        std::cout << "[805] bool MSVehicle::moveChecked()" << std::endl;        
      myState.setMyEnterPosLane(myState.myPos);
      if(oc.getSimulationVerbosity() > 2)
        std::cout << "[806] bool MSVehicle::moveChecked()" << std::endl;        
      
      if(aux1)
      {
        
        if(oc.getSimulationVerbosity() > 2)
          std::cout << "[807] bool MSVehicle::moveChecked()" << std::endl;          
        myState.setMyLastChargeRemoved(aux1->getMyLastChargeRem());
      
      }
      else
      {
        if(oc.getSimulationVerbosity() > 2)
          std::cout << "  CRITICAL ERROR asking for the MSDevice_FEV of a non F"
            "EV vehicle in file " << __FILE__ << ":" << __LINE__ << std::endl;
      }
      
      /* remember aux1 may segfault */
    }
    else
    {
      /* No lane change */
      /* Also send the NMEA string */
      /* But only if ARFTmode is requested */
      if(OptionsCont::getOptions().isSet("ARFTmode"))
      {
        if(oc.getSimulationVerbosity() > 2)
          std::cout<<"ARFTmode is set"<<std::endl;
        /* The timing checkings are inside adasWriteGpsString() */
        if((strcmp(getID().c_str(),"fev0")==0)||
          (strcmp(getID().c_str(),"fev1")==0))
        {
          if(!amAdasChecked)
          {
            
            if      (strcmp(getID().c_str(),"fev0")==0){ // ECOGEM MODE
              //std::cout<<"fev0"<<std::endl;
              amAdasChecked=true;
              amAdas=true;
              //myAdasRp=myWebServiceUrl="http://192.168.1.20:8792/VDAdas?wsdl"; // DEPRECATED
              //myAdasRp=myWebServiceUrl="http://93.62.202.221/VDAdas?wsdl"; // DEPRECATED
              
              // LOCAL ITS/BXL
              myWebServiceUrl="http://192.168.1.20:8792/VDAdas?wsdl"; // FIXME
              // REMOTE ITALY
              //myWebServiceUrl="http://93.62.202.221/VDAdas?wsdl"; // FIXME
              
            }else if(strcmp(getID().c_str(),"fev1")==0){ // NON ECOGEM MODE
              //std::cout<<"fev1"<<std::endl;
              //amAdasChecked=true;
              amAdasChecked=false;
              amAdas=true;
              //myAdasRp=myWebServiceUrl="http://192.168.1.10:8792/VDAdas?wsdl"; // DEPRECATED
              //myAdasRp=myWebServiceUrl="http://147.102.7.30:8792/VDAdas?wsdl"; // DEPRECATED
              
              // LOCAL ITS/BXL
              myWebServiceUrl="http://192.168.1.10:8792/VDAdas?wsdl"; // FIXME
              // GREECE
              //myWebServiceUrl="http://147.102.7.30:8792/VDAdas?wsdl"; // FIXME
              
            }else{
              //std::cout<<getID()<<std::endl;
              amAdasChecked=true;
              amAdas=false;
            }
            
            ;
          }
          if(amAdas==true)
          {
            if(oc.getSimulationVerbosity() > 2)
              std::cout << "[803] amAdas"
                        << std::endl;
            /* Whitespaces for easy commenting */
            
            //std::cout<<"check1"<<std::endl;
            adasWriteInfo();
            
            ;
          }
        } /* vehicleId is one of fev0 or fev1 */
      } /* ARFTmode active */
    } /* no lane change */
    if(oc.getSimulationVerbosity() > 2)
      std::cout << "[808] bool MSVehicle::moveChecked()" << std::endl;          
    myLastEdgeId.assign(myLane->getEdge().getID());
  /*
  }
  */
  if(oc.getSimulationVerbosity() > 2)
    std::cout << "<---- bool MSVehicle::moveChecked()" << std::endl;    
  return moved;
}

SUMOReal MSVehicle::getCurrentTraveledDistance(){ return myCurrentTraveledDistance; }

SUMOReal MSVehicle::getLastTraveledDistance(){ return myLastTraveledDistance; }

/*   The web service based interface IVDAdas exposes a couple of methods:
 * WriteGPSString and WriteExtendedInfo. The first method will accept an array
 * of strings formatted according to the NMEA protocol (e.g. $GPGGA, $GPGLL,
 * $GPRMC), which is the standard for GPS; for our purposes, the only $GPRMC
 * string could suffice (see table in the appendix of this mail).
 *   The second method (that can be optionally called after the first one) will
 * accept all complementary data, such as weight, SOC, SOH, etc. If HIB and TEC
 * think this approach is fine, we can publish a dummy interface for testing
 * purposes in a relatively short time.
 *   Hope these changes can simplify our and your work. Best regards.
 *  
 * Appendix: structure of $GPRMC NMEA string
 *  
 * $GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*70
 *        1      2 3       4 5        6 7     8     9      10    11 12
 *  
 *       1   220516     Time Stamp
 *       2   A          validity - A-ok, V-invalid
 *       3   5133.82    current Latitude
 *       4   N          North/South
 *       5   00042.24   current Longitude
 *       6   W          East/West
 *       7   173.8      Speed in knots
 *       8   231.8      True course (heading)
 *       9   130694     Date Stamp
 *       10  004.2      Variation
 *       11  W          East/West
 *       12  *70        Checksum
 *  
 * Marco Gorini
 * Softeco Sismat S.p.A.
 * WTC Tower
 * Via De Marini 1
 * 16149 GENOVA
 * ITALY
 * Cell: +39-335-7484779
 * Phone: +39-010-60261
 * Fax: +39-010-6026350
 * e-mail: marco.gorini@softeco.it 
 * www.softeco.it
 */

/*
 *   RMC - NMEA has its own version of essential gps pvt (position, velocity, time) data. It is called RMC, The Recommended Minimum, which will look similar to:

$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A

Where:
     RMC          Recommended Minimum sentence C
     123519       Fix taken at 12:35:19 UTC
     A            Status A=active or V=Void.
     4807.038,N   Latitude 48 deg 07.038' N
     01131.000,E  Longitude 11 deg 31.000' E
     022.4        Speed over the ground in knots
     084.4        Track angle in degrees True
     230394       Date - 23rd of March 1994
     003.1,W      Magnetic Variation
     *6A          The checksum data, always begins with *

Note that, as of the 2.3 release of NMEA, there is a new field in the RMC sentence at the end just prior to the checksum. For more information on this field see here. 

www.gpsinformation.org/dale/nmea.htm

*/

/*

http://www.codepedia.com/1/The+GPRMC+Sentence
http://www.gpsinformation.org/dale/nmea.htm#RMC

*/

static int timeStampCounter = 0;

#define KNOTS_IN_A_METER_DIV_SECOND 1.9438612860586

SUMOReal /*MSVehicle::*/toKnots(SUMOReal speed/*InKmPerHour*/)
{
  const bool IN_METERS_DIV_SECOND = true;
  if(IN_METERS_DIV_SECOND)
    return KNOTS_IN_A_METER_DIV_SECOND * speed;
  else
    return KNOTS_IN_A_METER_DIV_SECOND * speed * 3.6;
}

/*
char *upperHex(byte b)
{
  char *toReturn = (char *)malloc(2);
  toReturn ("%x
}

char *lowerHex(byte b)
{
  char *toReturn = (char *)malloc(2);
}
*/

#ifdef byte
#undef byte
#endif
#define byte char

char *hex(byte b)
{
  char *toReturn = (char *)malloc(3);
  /*
  toReturn[0] = upperHex(b)[0];
  toReturn[1] = lowerHex(b)[0];
  toReturn[2] = '\0';
  */
  sprintf(toReturn,"%x",b);
  return toReturn;
}

/*

$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A

Where:
     RMC          Recommended Minimum sentence C
     123519       Fix taken at 12:35:19 UTC
     A            Status A=active or V=Void.
     4807.038,N   Latitude 48 deg 07.038' N
     01131.000,E  Longitude 11 deg 31.000' E
     022.4        Speed over the ground in knots
     084.4        Track angle in degrees True
     230394       Date - 23rd of March 1994
     003.1,W      Magnetic Variation
     *6A          The checksum data, always begins with *
     
*/

static time_t start;

static bool startTimeSetted = false;

static void startSetter()
{
  OptionsCont &oc = OptionsCont::getOptions();
  if(startTimeSetted)
  {
    std::cout<<"CRITICAL1 static void startSetter()"<<std::endl;
    exit(0xffff);
  }
  else
  {
    if(oc.getAppName().substr(0,8)=="sumo-gui")
      //start = 0; // TODO
      start = *MyHandler2::getInitialTime(); // What does the upper mean ? ---> FIXME IS BECAUSE LOADER DOES NOT LOAD GOOD. PLEASE SOLVE.
    else if(oc.getAppName().substr(0,4)=="sumo")
      start = *MyHandler2::getInitialTime();
    else
    {
      std::cout<<"CRITICAL2 static void startSetter()"<<std::endl;
      exit(0xffff);
    }
    startTimeSetted = true;
  }
}

time_t thenCalculatorTimetToString()
{
  // its fair patch
  return start + MSNet::getInstance()->getCurrentTimeStep() /
                 SECS_IN_CURENT_TIME_STEP;
}
        
std::string MSVehicle::adasFriendlyDateStamp(time_t then)
{
  std::string returning;
  char *buffer;  
  buffer = (char *)malloc(64);   
  strftime(buffer, 48, "%d%m%y", gmtime(&then));  
  returning = strdup(buffer);
  free(buffer);
  return returning;
}

std::string MSVehicle::adasFriendlyTimeStamp(time_t then)
{
  bool itsViennaFairPatch=false;
  if(itsViennaFairPatch)
    // hour +2
    ;
  std::string returning;
  char *buffer;  
  buffer = (char *)malloc(64);   
  //strftime(buffer, 48, "%3H%3M%3S.000", gmtime(&then)); // FIXME CORRECT MILLISECS
  //strftime(buffer, 48, "%2H%2M%2S.000", gmtime(&then)); // FIXME CORRECT MILLISECS
  strftime(buffer, 48, "%I%M%S.000", gmtime(&then)); // FIXME CORRECT MILLISECS
  //std::cout<<buffer<<std::endl;
  if(itsViennaFairPatch)
  {
    // hour +2
    if(buffer[1]=='8')
      buffer[1]=0;
    else if(buffer[1]=='9')
      buffer[1]=1;
    else
      buffer[1]+=2;
  }
  else
  {
    // hour +1
    if(buffer[1]=='9')
      buffer[1]=0;
    else
      buffer[1]+=1;
  }
  returning = strdup(buffer);
  free(buffer);
  return returning;
}

void MSVehicle::adasWriteInfo()
{
  adasWriteGpsString();
  adasWriteExtendedInfo(); // not yet available
}


/* g++ -DHAVE_CONFIG_H
 *     -I.
 *     -I../../src
 *     -I/home/uxio/ecogem/development/sumo-0.14.0-hib/./src
 *     -I/usr/local/include/fox-1.6
 *     -I/usr/include/fox-1.6
 *     -I/usr/local/include/gdal
 *     -I/usr/include/gdal
 *     -I/usr/local/include
 *     -I/usr/include
 *     -g
 *     -pthread
 *     -MT MSVehicle.o
 *     -MD
 *     -MP
 *     -MF .deps/MSVehicle.Tpo
 *     -c
 *     -o MSVehicle.o
 *     MSVehicle.cpp
 */

const int VDADAS_WAIT = 1;

void MSVehicle::adasWriteGpsString()
{
  std::string gprmcString, webServiceUrl, command, commandArguments;
  char *aux, *checksumString;
  SUMOReal latitude, longitude, oldLatitude, oldLongitude, angle, bearing,
    difDist;
  int i;
  //byte checksum;
  unsigned char checksum;
  /*   Take care that this time_t 'now' is used not for dateTime printing labor
   * but controlling adas update period (kinda one second)
   */
  //time_t *now = (time_t*)malloc(sizeof(time_t));
  //time_t now;
  OptionsCont &oc = OptionsCont::getOptions();
  if(oc.getSimulationVerbosity() > 1)
    std::cout << "----> void MSVehicle::adasWriteGpsString()" << std::endl;
  aux = (char *)malloc(64);
  checksumString=(char*)malloc(64);
  if(!startTimeSetted)
    startSetter();
  now=time(NULL);
  if      (strcmp(getID().c_str(),"fev0")==0){
    adasZroTimeTNow=time(NULL);
    now=adasZroTimeTNow;
  }else if(strcmp(getID().c_str(),"fev1")==0){
    adasOneTimeTNow=time(NULL);
    now=adasOneTimeTNow;
  }
  /* time(&now); */
  if(now == -1)
  {
    std::cout<<"Exiting on time(time_t *) unhandled error\n"<<std::endl;
    exit(-1);
  }
  //if(now-(getLastAdasUpdateTime()) < 4) /* TASK Verify this */
  
  if      (strcmp(getID().c_str(),"fev0")==0){
    if(now-adasZroLastAdasUpdateTimeT<VDADAS_WAIT)
      //return;
      ;
  }else if(strcmp(getID().c_str(),"fev1")==0){
    if(now-adasOneLastAdasUpdateTimeT<VDADAS_WAIT)
      //return;
      ;
  }
  
  /*
  if((strcmp(getID().c_str(),"fev0")==0)||(strcmp(getID().c_str(),"fev1")==0))
  {
    if(now-getLastAdasUpdateTimeT()<VDADAS_WAIT)
      //return;
      ;
  }
  */
  /* time is bigger than VDADAS_WAIT, must send NMEA string */
  if(oc.getSimulationVerbosity() > 2)
    std::cout << "[100] void MSVehicle::adasWriteGpsString()" << std::endl;
  /* RMC          Recommended Minimum sentence C */
  gprmcString = "";
  gprmcString += "\\$GPRMC,"; /* The double bar is for bash to not recognize $GPRMC as the GPRMC bash variable */
  /* Time stamp */
  /* 123519       Fix taken at 12:35:19 UTC */
  gprmcString += adasFriendlyTimeStamp(thenCalculatorTimetToString());
  /* A            Status A=active or V=Void. */
  gprmcString += ",A,";
  /* 4807.038,N   Latitude 48 deg 07.038' N */
  /* getMyLane() =?= getLane() */
  /* getMyLane()->getBegin() ???
   * getMyLane()->getEnd() ???
   * getMyLane()->getMiddle() ???
   */
  std::string dateCommand="";
  dateCommand+="date >> COMMAND_TRACE";
  system(dateCommand.c_str());
  std::string vehicleIdCommand="echo VEHID: "+getID()+" RUNNING OVER LINKID: "+getLane()->getEdge().getID()+" >> COMMAND_TRACE";
  system(vehicleIdCommand.c_str());
  std::string traceCommand="";
  //Position pos(getMyLane()->getShape().getBegin().x(),getMyLane()->getShape().getBegin().y());
  Position pos(getPosition());
  //Position pos(getMyPosOnLane());getPosition()
  char *aux1=(char*)malloc(64), *aux2=(char*)malloc(64);
  traceCommand="echo 'ADAS TIMESTAMP: "+adasFriendlyTimeStamp(thenCalculatorTimetToString())+"' >> COMMAND_TRACE";
  sprintf(aux1,"%f",pos.y());
  sprintf(aux2,"%f",pos.x());
  traceCommand="echo 'SUMO CARTESIAN COORDINATES: "+std::string(aux1)+","+std::string(aux2)+"' >> COMMAND_TRACE";
  system(traceCommand.c_str());
  GeoConvHelper::getFinal().cartesian2geo(pos);
  sprintf(aux1,"%f",pos.y());
  sprintf(aux2,"%f",pos.x());
  traceCommand="echo 'GEODESIC DECIMAL COORDINATES: "+std::string(aux1)+","+std::string(aux2)+"' >> COMMAND_TRACE";
  system(traceCommand.c_str());
  sprintf(aux1,"%f",decimalToNmea(pos.y()));
  sprintf(aux2,"%f",decimalToNmea(pos.x()));
  traceCommand="echo 'GEODESIC NMEA COORDINATES: "+std::string(aux1)+","+std::string(aux2)+"' >> COMMAND_TRACE";
  system(traceCommand.c_str());
  //latitude=decimalToNmea(GeoConvHelper::getFinal().cartesian2geo(getMyLane()->getShape().getBegin().x())/*+getMyState().getMyPosOnLane()*/);
  latitude=decimalToNmea(pos.y());
  sprintf(aux,"%8.4f",latitude);
  gprmcString += aux;
  gprmcString += ",N,";
  if(oc.getSimulationVerbosity() > 2)
    std::cout << "[250] void MSVehicle::adasWriteGpsString()" << std::endl;
  /* 01131.000,E  Longitude 11 deg 31.000' E */
  //longitude=decimalToNmea(GeoConvHelper::getFinal().cartesian2geo(getMyLane()->getShape().getBegin().y())/*+getMyState().getMyPosOnLane()*/);
  longitude=decimalToNmea(pos.x());
  sprintf(aux,"00%8.4f,",longitude);
  gprmcString += aux;
  gprmcString += longitude<0?"W,":"E,";
  
  /* 022.4        Speed over the ground in knots */
  // Original was
  sprintf(aux,"%.1f",toKnots(myState.getMySpeed()));
  // New try
  //std::cout<<OutputDevice::realString(pos.x(),15)<<","<<OutputDevice::realString(pos.y(),15)<<std::endl;
  pos=getPosition();
  //std::cout<<OutputDevice::realString(pos.x(),15)<<","<<OutputDevice::realString(pos.y(),15)<<std::endl;
  //std::cout<<OutputDevice::realString(myLastAdasPosition.x(),15)<<","<<OutputDevice::realString(myLastAdasPosition.y(),15)<<std::endl;
  //std::cout<<OutputDevice::realString(pos.distanceTo2D(myLastAdasPosition),15)<<std::endl;
  //std::cout<<OutputDevice::realString(pos.distanceTo2D(myLastAdasPosition)/1000.,15)<<std::endl;
  //std::cout<<OutputDevice::realString(toKnots(pos.distanceTo2D(myLastAdasPosition)/1000.),15)<<std::endl;
  /*
  if(first)
  {
    sprintf(aux,"%.1f",toKnots(0));
    first=false;
  }
  else
    //sprintf(aux,"%.1f",toKnots(pos.distanceTo2D(myLastAdasPosition)/1000.));
    sprintf(aux,"%.3f",toKnots(pos.distanceTo2D(myLastAdasPosition)/1000.));
    //sprintf(aux,"%.1f",toKnots(10));
  myLastAdasPosition=pos;
  */
  // Add it to gprmcString
  gprmcString += aux;
  
  /* heading / bearing / angle */
  /* 084.4        Track angle in degrees True */
  angle = getAngle();
  bearing = angle>0?180-+angle:180+-angle;
  if(oc.getSimulationVerbosity() > 2)
  {
    std::cout << "[400] void MSVehicle::adasWriteGpsString()" << std::endl;
    std::cout << " angle{"<<angle<<"}, bearing{"<<bearing<<"}"<< std::endl;
  }
  gprmcString += ",";
  sprintf(aux,"%.1f",bearing);
  gprmcString += aux;
  gprmcString += ",";
  if(oc.getSimulationVerbosity() > 2)
    std::cout << "[500] void MSVehicle::adasWriteGpsString()" << std::endl;
  /* Date stamp */
  /* 230394       Date - 23rd of March 1994 */  
  gprmcString += adasFriendlyDateStamp(thenCalculatorTimetToString());
  gprmcString += ",";
  /* 003.1,W      Magnetic Variation */  
  //oldLatitude=getMyLane()->getShape().getBegin().y()/*+getMyOldState().getMyPosOnLane()*/;
  //oldLongitude=getMyLane()->getShape().getBegin().x()/*+getMyOldState().getMyPosOnLane()*/;
  /*difDist = sqrt(pow(abs(abs(latitude)-abs(oldLatitude))  , 2) +
                 pow(abs(abs(longitude)-abs(oldLongitude)), 2));*/
  //sprintf(aux,"%f",difDist);
  //gprmcString += aux;
  gprmcString += ",";
  //gprmcString += (rand()/(RAND_MAX+0.) < 0.5) ? "W" : "E" ;
  gprmcString += ",";
  gprmcString += "A";
  gprmcString += "*";
  /* *6A          The checksum data, always begins with * */
  //checksum = (unsigned char)(gprmcString[1]);
  checksum = 0;
  //for(i = 2; i < gprmcString.length()-1; i++)
  for(i = 1; i < gprmcString.length()-1; i++)
  {
    checksum ^= (unsigned char)(gprmcString[i]);
  }
  if(checksum<16)
    sprintf(checksumString,"0%hx",checksum);
  else
    sprintf(checksumString,"%2hx",checksum);
  //gprmcString += checksumString;
  //webServiceUrl = myWebServiceUrl;
  //commandArguments  = "/WriteGPSString?" + gprmcString;
  //command = "curl '" + webServiceUrl + commandArguments +"' &";
  /*
  if(oc.getSimulationVerbosity())
    std::cout << "[700] void MSVehicle::adasWriteGpsString() - "
              << "Will now execute '" << command << "'" <<std::endl;
  */
  /*
   * Url for NMEA checksum validation
   * http://www.hhhh.org/wiml/proj/nmeaxor.html
   */
  command="python sudsVdadasTestConEntradaSinChecksum.py -i "+gprmcString;
  //command="python sudsVdadasTestConEntrada.py -i "+gprmcString+" &> foo";
  command+=" -u "+myWebServiceUrl;
  command+=" -I "+getID();
  command+=" -n ";
  command+=" -Z ";
  //command+=" &"; // DO NOT DECOMMENT THIS
  //std::cout<<command.c_str()<<std::endl;
  //std::cout<<command<<std::endl;
  //std::string dateCommand="echo '(";
  //dateCommand+=getID()+")'";
  traceCommand="echo \'"+command+"\' >> COMMAND_TRACE";
  std::string nmeaTraceCommand="echo "+gprmcString+" >> COMMAND_TRACE";
  // Separator for the COMMAND_TRACE file registers
  std::string clrfCommand="echo >> COMMAND_TRACE";
  std::string nmeaTraceInNmeaFileCommand="echo "+gprmcString+" >> NMEA_TRACE_all.nmea";
  std::string nmeaTraceInNmeaFileCommandFevZro="echo "+gprmcString+" >> NMEA_TRACE_fev0.nmea";
  std::string nmeaTraceInNmeaFileCommandFevOne="echo "+gprmcString+" >> NMEA_TRACE_fev1.nmea";
  //system(dateCommand.c_str());
  system(nmeaTraceCommand.c_str());
  system(nmeaTraceInNmeaFileCommand.c_str());
  if(strcmp("fev0",getID().c_str())==0)
    system(nmeaTraceInNmeaFileCommandFevZro.c_str());
  if(strcmp("fev1",getID().c_str())==0)
    system(nmeaTraceInNmeaFileCommandFevOne.c_str());
  system(traceCommand.c_str());
  system(clrfCommand.c_str());
  //std::cout<<"check3"<<std::endl;
  system(command.c_str()); // TODO TASK FIXME ALERT DANGER ACTIVATE THIS IN FINAL VERSION !!!
  //std::cout<<"check4"<<std::endl;
  time(&now/*=time(NULL)*/);
  if(now == -1)
  {
    std::cout<<"Exiting on time(time_t *) unhandled error\n"<<std::endl;
    exit(-1);
  }
  
  if     (strcmp(getID().c_str(),"fev0")==0)
    adasZroLastAdasUpdateTimeT=now;
  else if(strcmp(getID().c_str(),"fev1")==0)
    adasOneLastAdasUpdateTimeT=now;
  
  /*
  if((strcmp(getID().c_str(),"fev0")==0)||(strcmp(getID().c_str(),"fev1")))
  {
    setLastAdasUpdateTimeT(&now);
  }
  */
  if(oc.getSimulationVerbosity() > 2)
    std::cout << "<---- void MSVehicle::adasWriteGpsString()" << std::endl;
}

/*
  ExtendedInfo Softeco's signature:
   AuxStatus = None
   BatteryCapacity = None
   BatteryHealth = None
   BatteryLevel = None
   EnvHumidity = None
   EnvTemperature = None
   Ignition = None
   IsCharging = None
   Power = None
   VIN = None
   VehicleWeight = None
 */
void MSVehicle::adasWriteExtendedInfo()
{
  //return; // TODO FIXME QUITAR ESTE RETURN para testearlo
  OptionsCont &oc = OptionsCont::getOptions();
  if(oc.getSimulationVerbosity() > 1)
    std::cout << "<---> void MSVehicle::adasWriteExtendedInfo()" << std::endl;
  /* Variable for fetching FEV device status */
  MSDevice_FEV *aux1 = NULL;
  /* Fetching an output device for string formatting */
  OutputDevice& of = OutputDevice::getDeviceByOption("netstate-dump-fev");
  /* Fetching FEV device status */
  for(std::vector<MSDevice*>::const_iterator it = getDevices().begin();
   it != getDevices().end();
   ++it)
  {
    aux1=dynamic_cast<MSDevice_FEV*>(*it);
    if(aux1){
      break;
    }
  }
  std::string
    AuxStatus="0", // TODO
    BatteryCapacity=OutputDevice::realString(getMyType()->getCapacity()*getMyType()->getNumCells(),2),
    BatteryHealth=OutputDevice::realString(aux1->getMySoh(),6),
    BatteryLevel=OutputDevice::realString(1-(aux1->getMyCurrDoD()),6),
    EnvHumidity=OutputDevice::realString(getMyType()->getHumidity(),2),
    EnvTemperature=OutputDevice::realString(getMyType()->getCelsius(),2),
    Ignition="0", // TODO
    IsCharging="0", // TODO
    Power="0", // TODO
    VIN=getID(),
    VehicleWeight=OutputDevice::realString(getMyType()->getMass(),2);
  std::string osOrder="python sudsVdadasSendExtendedInfo.py ";
  osOrder+="-i "+AuxStatus+","+BatteryCapacity+","+BatteryHealth+","+
    BatteryLevel+","+EnvHumidity+","+EnvTemperature+","+Ignition+","+IsCharging+
    ","+Power+","+VIN+","+VehicleWeight+" ";
  osOrder+="-u "+myWebServiceUrl+" ";
  osOrder+="-I "+getID()+" ";
  osOrder+="-n ";
  osOrder+="-Z ";
  //osOrder+="&"; // DO NOT DECOMMENT THIS
  system(osOrder.c_str());
}

SUMOReal MSVehicle::getSpaceTillLastStanding(MSLane* l, bool& foundStopped) {
	SUMOReal lengths = 0;
	const MSLane::VehCont& vehs = l->getVehiclesSecure();
	for (MSLane::VehCont::const_iterator i = vehs.begin(); i != vehs.end();
			++i) {
		if ((*i)->getSpeed() < .1) {
			foundStopped = true;
			SUMOReal ret = (*i)->getPositionOnLane()
					- (*i)->getVehicleType().getLengthWithGap() - lengths;
			l->releaseVehicles();
			return ret;
		}
		lengths += (*i)->getVehicleType().getLengthWithGap();
	}
	l->releaseVehicles();
	return l->getLength() - lengths;
}

void MSVehicle::checkRewindLinkLanes(SUMOReal lengthsInFront) {
#ifdef DEBUG_VEHICLE_GUI_SELECTION
	if (gSelected.isSelected(GLO_VEHICLE, static_cast<const GUIVehicle*>(this)->getGlID())) {
		int bla = 0;
		if (MSNet::getInstance()->getCurrentTimeStep() == 152000) {
			bla = 0;
		}
	}
#endif
#ifdef HAVE_INTERNAL_LANES
	if (MSGlobals::gUsingInternalLanes) {
		int removalBegin = -1;
		bool hadVehicle = false;
		SUMOReal seenSpace = -lengthsInFront;

		std::vector<SUMOReal> availableSpace;
		std::vector<bool> hadVehicles;
		bool foundStopped = false;

		for (unsigned int i = 0; i < myLFLinkLanes.size(); ++i) {
			// skip unset links
			DriveProcessItem& item = myLFLinkLanes[i];
			if (item.myLink == 0 || foundStopped) {
				availableSpace.push_back(seenSpace);
				hadVehicles.push_back(hadVehicle);
				continue;
			}
			// get the next lane, determine whether it is an internal lane
			MSLane* approachedLane = item.myLink->getViaLane();
			if (approachedLane != 0) {
				if (item.myLink->isCrossing()
						&& item.myLink->willHaveBlockedFoe()) {
					seenSpace = seenSpace - approachedLane->getVehLenSum();
					hadVehicle |= approachedLane->getVehicleNumber() != 0;
				} else {
					seenSpace = seenSpace
							+ getSpaceTillLastStanding(approachedLane,
									foundStopped); // - approachedLane->getVehLenSum() + approachedLane->getLength();
					hadVehicle |= approachedLane->getVehicleNumber() != 0;
				}
				availableSpace.push_back(seenSpace);
				hadVehicles.push_back(hadVehicle);
				continue;
			}
			approachedLane = item.myLink->getLane();
			MSVehicle* last = approachedLane->getLastVehicle();
			if (last == 0) {
				last = approachedLane->getPartialOccupator();
				if (last != 0) {
					SUMOReal m = MAX2(seenSpace,
							seenSpace + approachedLane->getPartialOccupatorEnd()
									+ last->getCarFollowModel().brakeGap(
											last->getSpeed()));
					availableSpace.push_back(m);
					hadVehicle = true;
					seenSpace = seenSpace
							+ getSpaceTillLastStanding(approachedLane,
									foundStopped); // - approachedLane->getVehLenSum() + approachedLane->getLength();
					if (last->myHaveToWaitOnNextLink) {
						foundStopped = true;
					}
				} else {
//                    seenSpace = seenSpace - approachedLane->getVehLenSum() + approachedLane->getLength();
//                    availableSpace.push_back(seenSpace);
					availableSpace.push_back(
							seenSpace
									+ getSpaceTillLastStanding(approachedLane,
											foundStopped));
					if (!foundStopped) {
						seenSpace = seenSpace - approachedLane->getVehLenSum()
								+ approachedLane->getLength();
					} else {
						seenSpace = availableSpace.back();
					}
				}
			} else {
				if (last->signalSet(VEH_SIGNAL_BRAKELIGHT)) {
					SUMOReal lastBrakeGap = last->getCarFollowModel().brakeGap(
							approachedLane->getLastVehicle()->getSpeed());
					SUMOReal lastGap =
							last->getPositionOnLane()
									- last->getVehicleType().getLengthWithGap()
									+ lastBrakeGap
									- last->getSpeed()
											* last->getCarFollowModel().getHeadwayTime();
					SUMOReal m = MAX2(seenSpace, seenSpace + lastGap);
					availableSpace.push_back(m);
					seenSpace = seenSpace
							+ getSpaceTillLastStanding(approachedLane,
									foundStopped); // - approachedLane->getVehLenSum() + approachedLane->getLength();
				} else {
//                    seenSpace = seenSpace - approachedLane->getVehLenSum() + approachedLane->getLength();
//                    availableSpace.push_back(seenSpace);
					availableSpace.push_back(
							seenSpace
									+ getSpaceTillLastStanding(approachedLane,
											foundStopped));
					if (!foundStopped) {
						seenSpace = seenSpace - approachedLane->getVehLenSum()
								+ approachedLane->getLength();
					} else {
						seenSpace = availableSpace.back();
					}
				}
				if (last->myHaveToWaitOnNextLink) {
					foundStopped = true;
				}
				hadVehicle = true;
			}
			hadVehicles.push_back(hadVehicle);
		}
#ifdef DEBUG_VEHICLE_GUI_SELECTION
		if (gSelected.isSelected(GLO_VEHICLE, static_cast<const GUIVehicle*>(this)->getGlID())) {
			int bla = 0;
		}
#endif
		SUMOTime t = MSNet::getInstance()->getCurrentTimeStep();
		for (int i = (int) (myLFLinkLanes.size() - 1); i > 0; --i) {
			DriveProcessItem& item = myLFLinkLanes[i - 1];
			bool opened = item.myLink != 0
					&& item.myLink->opened(t, .1,
							getVehicleType().getLengthWithGap());
			bool check1 = item.myLink == 0 || item.myLink->isCont()
					|| !hadVehicles[i];
			bool allowsContinuation = check1 || opened;
			if (!opened && item.myLink != 0) {
				if (i > 1) {
					DriveProcessItem& item2 = myLFLinkLanes[i - 2];
					if (item2.myLink != 0 && item2.myLink->isCont()) {
						allowsContinuation = true;
					}
				}
			}
			if (allowsContinuation) {
				availableSpace[i - 1] = availableSpace[i];
			}
		}

		for (unsigned int i = 0;
				hadVehicle && i < myLFLinkLanes.size() && removalBegin < 0;
				++i) {
			// skip unset links
			DriveProcessItem& item = myLFLinkLanes[i];
			if (item.myLink == 0) {
				continue;
			}
			/*
			 SUMOReal impatienceCorrection = MAX2(SUMOReal(0), SUMOReal(SUMOReal(myWaitingTime)));
			 if (seenSpace<getVehicleType().getLengthWithGap()-impatienceCorrection/10.&&nextSeenNonInternal!=0) {
			 removalBegin = lastLinkToInternal;
			 }
			 */

			SUMOReal leftSpace = availableSpace[i]
					- getVehicleType().getLengthWithGap();
			if (leftSpace < 0 && item.myLink->willHaveBlockedFoe()) {
				SUMOReal impatienceCorrection = 0;
				/*
				 if(item.myLink->getState()==LINKSTATE_MINOR) {
				 impatienceCorrection = MAX2(SUMOReal(0), STEPS2TIME(myWaitingTime));
				 }
				 */
				if (leftSpace < -impatienceCorrection / 10.) {
					removalBegin = i;
				}
				//removalBegin = i;
			}
		}
		if (removalBegin != -1
				&& !(removalBegin == 0
						&& myLane->getEdge().getPurpose()
								== MSEdge::EDGEFUNCTION_INTERNAL)) {
			while (removalBegin < (int) (myLFLinkLanes.size())) {
				myLFLinkLanes[removalBegin].myVLinkPass =
						myLFLinkLanes[removalBegin].myVLinkWait;
				myLFLinkLanes[removalBegin].mySetRequest = false;
				++removalBegin;
			}
		}
	}
#endif
	for (DriveItemVector::iterator i = myLFLinkLanes.begin();
			i != myLFLinkLanes.end(); ++i) {
		if ((*i).myLink != 0) {
			(*i).myLink->setApproaching(this, (*i).myArrivalTime,
					(*i).myArrivalSpeed, (*i).mySetRequest);
		}
	}
}

void MSVehicle::vsafeCriticalCont(SUMOTime t, SUMOReal boundVSafe) {
#ifdef DEBUG_VEHICLE_GUI_SELECTION
	if (gSelected.isSelected(GLO_VEHICLE, static_cast<const GUIVehicle*>(this)->getGlID())) {
		int bla = 0;
	}
#endif
#ifndef NO_TRACI
	if (myInfluencer != 0) {
		SUMOReal vMin = MAX2(SUMOReal(0),
				getVehicleType().getCarFollowModel().getSpeedAfterMaxDecel(
						myState.mySpeed));
		SUMOReal vMax = getVehicleType().getCarFollowModel().maxNextSpeed(
				myState.mySpeed);
		boundVSafe = myInfluencer->influenceSpeed(
				MSNet::getInstance()->getCurrentTimeStep(), boundVSafe,
				boundVSafe, vMin, vMax);
	}
#endif
	const MSCFModel& cfModel = getCarFollowModel();
	// the vehicle may have just to look into the next lane
	//  compute this information and use it only once in the next loop
	SUMOReal seen = myLane->getLength() - myState.myPos;
	SUMOReal seenNonInternal = 0;
	//
	if (this != myLane->getFirstVehicle()
			&& seen - cfModel.brakeGap(myState.mySpeed) > 0
			&& seen - SPEED2DIST(boundVSafe) - ACCEL2DIST(cfModel.getMaxAccel())
					> 0) {
		// not "reaching critical"
		myLFLinkLanes.push_back(
				DriveProcessItem(0, boundVSafe, boundVSafe, false, 0, 0, seen));
		return;
	}

	MSLane* nextLane = myLane;
	// compute the way the vehicle would drive if it would use the current speed and then
	//  decelerate
	SUMOReal maxV = cfModel.maxNextSpeed(myState.mySpeed);
	SUMOReal dist = SPEED2DIST(maxV) + cfModel.brakeGap(maxV);
	SUMOReal vLinkPass = boundVSafe;
	SUMOReal vLinkWait = vLinkPass;
	const std::vector<MSLane*> &bestLaneConts = getBestLanesContinuation();
#ifdef HAVE_INTERNAL_LANES
	bool hadNonInternal = false;
#else
	bool hadNonInternal = true;
#endif

	unsigned int view = 1;
	// loop over following lanes
	while (true) {
		// process stops
		if (!myStops.empty()
				&& &myStops.begin()->lane->getEdge() == &nextLane->getEdge()) {
			SUMOReal vsafeStop = cfModel.stopSpeed(this,
					seen - (nextLane->getLength() - myStops.begin()->endPos));
			vLinkPass = MIN2(vLinkPass, vsafeStop);
			vLinkWait = MIN2(vLinkWait, vsafeStop);
		}

		// get the next link used
		MSLinkCont::const_iterator link = myLane->succLinkSec(*this, view,
				*nextLane, bestLaneConts);

		// check whether the lane is a dead end
		//  (should be valid only on further loop iterations
		if (nextLane->isLinkEnd(link)) {
			SUMOReal laneEndVSafe = cfModel.stopSpeed(this, seen);
			if (myCurrEdge + view == myRoute->end()) {
				if (myParameter->arrivalSpeedProcedure == ARRIVAL_SPEED_GIVEN) {
					laneEndVSafe = cfModel.freeSpeed(this, getSpeed(), seen,
							myParameter->arrivalSpeed);
				} else {
					laneEndVSafe = vLinkPass;
				}
			}
			// the vehicle will not drive further
			assert(
					MIN2(vLinkPass, laneEndVSafe) >= cfModel.getSpeedAfterMaxDecel(myState.mySpeed));
			myLFLinkLanes.push_back(
					DriveProcessItem(0, MIN2(vLinkPass, laneEndVSafe),
							MIN2(vLinkPass, laneEndVSafe), false, 0, 0, seen));
			return;
		}
		// the link was passed
		vLinkWait = vLinkPass;

		// get the following lane
#ifdef HAVE_INTERNAL_LANES
		nextLane = (*link)->getViaLane();
		if (nextLane == 0) {
			nextLane = (*link)->getLane();
			hadNonInternal = true;
			view++;
		}
#else
		nextLane = (*link)->getLane();
		view++;
#endif

		// compute the velocity to use when the link is not blocked by other vehicles
		//  the vehicle shall be not faster when reaching the next lane than allowed
		SUMOReal vmaxNextLane = MAX2(
				cfModel.freeSpeed(this, getSpeed(), seen,
						nextLane->getMaxSpeed()), nextLane->getMaxSpeed());

		// the vehicle shall keep a secure distance to its predecessor
		//  (or approach the lane end if the predeccessor is too near)
		SUMOReal vsafePredNextLane = 100000;
		std::pair<MSVehicle*, SUMOReal> lastOnNext =
				nextLane->getLastVehicleInformation();
		if (lastOnNext.first != 0) {
			if (seen + lastOnNext.second >= 0) {
				vsafePredNextLane = cfModel.followSpeed(this, getSpeed(),
						seen + lastOnNext.second, lastOnNext.first->getSpeed(),
						lastOnNext.first->getCarFollowModel().getMaxDecel());
			} else {
				vsafePredNextLane = cfModel.stopSpeed(this, seen);
			}
		}
#ifdef DEBUG_VEHICLE_GUI_SELECTION
		if (gSelected.isSelected(GLO_VEHICLE, static_cast<const GUIVehicle*>(this)->getGlID())) {
			int bla = 0;
		}
#endif
		// compute the velocity to use when the link may be used
		vLinkPass = MIN3(vLinkPass, vmaxNextLane, vsafePredNextLane);

		// if the link may not be used (is blocked by another vehicle) then let the
		//  vehicle decelerate until the end of the street
		vLinkWait = MIN3(vLinkPass, vLinkWait, cfModel.stopSpeed(this, seen));
#ifdef _DEBUG
		if (vLinkWait < cfModel.getSpeedAfterMaxDecel(myState.mySpeed)) {
			WRITE_WARNING("Vehicle '" + getID() + "' is decelerating too much (#2; is: " + toString(myState.mySpeed - vLinkWait) + ", may: " + toString(cfModel.getSpeedAfterMaxDecel(myState.mySpeed)) + ")");
		}
#endif

		// behaviour in front of not priorised intersections (waiting for priorised foe vehicles)
		bool setRequest = false;
		// process stops
		if (!myStops.empty()
				&& &myStops.begin()->lane->getEdge() == &nextLane->getEdge()) {
			const Stop& stop = *myStops.begin();
			const SUMOReal vsafeStop =
					stop.busstop == 0 ?
							cfModel.stopSpeed(this, seen + stop.endPos) :
							cfModel.stopSpeed(this,
									seen
											+ stop.busstop->getLastFreePos() - POSITION_EPS);
			vLinkPass = MIN2(vLinkPass, vsafeStop);
			vLinkWait = MIN2(vLinkWait, vsafeStop);
		}
		// check whether we approach the final edge
		if (myCurrEdge + view == myRoute->end()) {
			if (myParameter->arrivalSpeedProcedure == ARRIVAL_SPEED_GIVEN) {
				const SUMOReal vsafe = cfModel.freeSpeed(this, getSpeed(),
						seen + myArrivalPos, myParameter->arrivalSpeed);
				vLinkPass = MIN2(vLinkPass, vsafe);
				vLinkWait = MIN2(vLinkWait, vsafe);
			}
		}

		setRequest |= ((*link)->getState() != LINKSTATE_TL_RED
				&& (vLinkPass > 0 && dist - seen > 0));
		bool yellow = (*link)->getState() == LINKSTATE_TL_YELLOW_MAJOR
				|| (*link)->getState() == LINKSTATE_TL_YELLOW_MINOR;
		bool red = (*link)->getState() == LINKSTATE_TL_RED;
		if ((yellow || red)
				&& seen
						> cfModel.brakeGap(myState.mySpeed)
								- SPEED2DIST(myState.mySpeed)
										* cfModel.getHeadwayTime()) { // !!! we should reuse brakeGap with no reaction time...
			vLinkPass = vLinkWait;
			setRequest = false;
			assert(vLinkWait >= cfModel.getSpeedAfterMaxDecel(myState.mySpeed));
			myLFLinkLanes.push_back(
					DriveProcessItem(*link, vLinkWait, vLinkWait, false,
							t + TIME2STEPS(seen / vLinkPass), vLinkPass, seen));
		}
		// the next condition matches the previously one used for determining the difference
		//  between critical/non-critical vehicles. Though, one should assume that a vehicle
		//  should want to move over an intersection even though it could brake before it!?
		setRequest &= dist - seen > 0;
#ifdef _DEBUG
		if (MIN2(vLinkPass, vLinkWait) < cfModel.getSpeedAfterMaxDecel(myState.mySpeed)) {
			WRITE_WARNING("Vehicle '" + getID() + "' is decelerating too much (#3; is: " + toString(myState.mySpeed - MIN2(vLinkPass, vLinkWait)) + ", may: " + toString(cfModel.getSpeedAfterMaxDecel(myState.mySpeed)) + ")");
		}
#endif
		myLFLinkLanes.push_back(
				DriveProcessItem(*link, vLinkPass, vLinkWait, setRequest,
						t + TIME2STEPS(seen / vLinkPass), vLinkPass, seen));
		seen += nextLane->getLength();
		seenNonInternal +=
				nextLane->getEdge().getPurpose()
						== MSEdge::EDGEFUNCTION_INTERNAL ?
						0 : nextLane->getLength();
		if ((vLinkPass <= 0 || seen > dist) && hadNonInternal
				&& seenNonInternal > 50) {
			return;
		}
	}
}

void MSVehicle::activateReminders(const MSMoveReminder::Notification reason) {
	for (MoveReminderCont::iterator rem = myMoveReminders.begin();
			rem != myMoveReminders.end();) {
		if (rem->first->getLane() != 0 && rem->first->getLane() != getLane()) {
			++rem;
		} else {
			if (rem->first->notifyEnter(*this, reason)) {
				++rem;
			} else {
				rem = myMoveReminders.erase(rem);
			}
		}
	}
}

bool MSVehicle::enterLaneAtMove(MSLane* enteredLane, bool onTeleporting) {

	myAmOnNet = true;
	// vaporizing edge?
	/*
	 if (enteredLane->getEdge().isVaporizing()) {
	 // yep, let's do the vaporization...
	 myLane = enteredLane;
	 return true;
	 }
	 */

	if (!onTeleporting) {

		// move mover reminder one lane further
		adaptLaneEntering2MoveReminder(*enteredLane);
		// set the entered lane as the current lane
		myLane = enteredLane;
	}

	// internal edges are not a part of the route...
	if (enteredLane->getEdge().getPurpose() != MSEdge::EDGEFUNCTION_INTERNAL) {
		assert(&enteredLane->getEdge() == *(myCurrEdge + 1));
		++myCurrEdge;
	}
	if (!onTeleporting) {
		// may be optimized: compute only, if the current or the next have more than one lane...!!!
		getBestLanes(true);
		activateReminders(MSMoveReminder::NOTIFICATION_JUNCTION);
#ifndef NO_TRACI
		if (myInfluencer != 0) {
			myLaneChangeModel->requestLaneChange(
					myInfluencer->checkForLaneChanges(
							MSNet::getInstance()->getCurrentTimeStep(),
							**myCurrEdge, getLaneIndex()));
		}
#endif
	}
	return ends();
}

void MSVehicle::enterLaneAtLaneChange(MSLane* enteredLane) {
	myAmOnNet = true;

#ifdef _MESSAGES
	if (myLCMsgEmitter != 0) {
		SUMOReal timeStep = MSNet::getInstance()->getCurrentTimeStep();
		myLCMsgEmitter->writeLaneChangeEvent(myParameter->id, timeStep, myLane, myState.pos(), myState.speed(), enteredLane, getPosition().x(), getPosition().y());
	}
#endif
	myLane = enteredLane;
	// switch to and activate the new lane's reminders
	// keep OldLaneReminders
	for (std::vector<MSMoveReminder*>::const_iterator rem =
			enteredLane->getMoveReminders().begin();
			rem != enteredLane->getMoveReminders().end(); ++rem) {
		addReminder(*rem);
	}
	activateReminders(MSMoveReminder::NOTIFICATION_LANE_CHANGE);
	SUMOReal leftLength = myState.myPos - getVehicleType().getLengthWithGap();
	if (leftLength < 0) {
		// we have to rebuild "further lanes"
		const MSRoute& route = getRoute();
		MSRouteIterator i = myCurrEdge;
		MSLane* lane = myLane;
		while (i != route.begin() && leftLength > 0) {
			const MSEdge* const prev = *(--i);
			const std::vector<MSLane::IncomingLaneInfo> &incomingLanes =
					lane->getIncomingLanes();
			for (std::vector<MSLane::IncomingLaneInfo>::const_iterator j =
					incomingLanes.begin(); j != incomingLanes.end(); ++j) {
				if (&(*j).lane->getEdge() == prev) {
#ifdef HAVE_INTERNAL_LANES
					(*j).lane->setPartialOccupation(this, leftLength);
#else
					leftLength -= (*j).length;
					(*j).lane->setPartialOccupation(this, leftLength);
#endif
					leftLength -= (*j).lane->getLength();
					break;
				}
			}
		}
		myState.setEnterLane(MSNet::getInstance()->getCurrentTimeStep());
		myState.setMyPosOnLane(myState.myPos);
	}
#ifndef NO_TRACI
	// check if further changes are necessary
	if (myInfluencer != 0) {
		myLaneChangeModel->requestLaneChange(
				myInfluencer->checkForLaneChanges(
						MSNet::getInstance()->getCurrentTimeStep(),
						**myCurrEdge, getLaneIndex()));
	}
#endif
}

void MSVehicle::enterLaneAtInsertion
 (MSLane* enteredLane, SUMOReal pos, SUMOReal speed,
  MSMoveReminder::Notification notification)
{
  myOlderState = myOldState;
  myOldState = myState;
  myState = State(pos, speed, MSNet::getInstance()->getCurrentTimeStep());
  assert(myState.myPos >= 0);
  assert(myState.mySpeed >= 0);
  //assert(myState.myPosOnLane >= 0);
  myState.setMyPosOnLane(myState.myPos);
  myWaitingTime = 0;
  myLane = enteredLane;
  // set and activate the new lane's reminders
  for(
  std::vector < MSMoveReminder * > :: const_iterator rem = 
    enteredLane->getMoveReminders().begin();
  rem != enteredLane->getMoveReminders().end(); ++rem)
  {
    addReminder(*rem);
  }
  activateReminders(notification);
  std::string msg;
  if(MSGlobals::gCheckRoutes && !hasValidRoute(msg))
    throw ProcessError("Vehicle '" + getID() + "' has no valid route. " + msg);
  myAmOnNet = true;
  // build the list of lanes the vehicle is lapping into
  SUMOReal leftLength = myType->getLengthWithGap() - pos;
  MSLane* clane = enteredLane;
  while(leftLength > 0)
  {
    clane = clane->getLogicalPredecessorLane();
    if(clane == 0)
    {
      break;
    }
    myFurtherLanes.push_back(clane);
    leftLength -= (clane)->setPartialOccupation(this, leftLength);
  }
}

void MSVehicle::leaveLane(const MSMoveReminder::Notification reason) {
	for (MoveReminderCont::iterator rem = myMoveReminders.begin();
			rem != myMoveReminders.end();) {
		if (rem->first->notifyLeave(*this, myState.myPos + rem->second,
				reason)) {
			++rem;
		} else {
			rem = myMoveReminders.erase(rem);
		}
	}
	if (reason != MSMoveReminder::NOTIFICATION_JUNCTION) {
		for (std::vector<MSLane*>::iterator i = myFurtherLanes.begin();
				i != myFurtherLanes.end(); ++i) {
			(*i)->resetPartialOccupation(this);
		}
		myFurtherLanes.clear();
	}
	if (reason >= MSMoveReminder::NOTIFICATION_TELEPORT) {
		myAmOnNet = false;
	}

}

MSAbstractLaneChangeModel&
MSVehicle::getLaneChangeModel() {
	return *myLaneChangeModel;
}

const MSAbstractLaneChangeModel&
MSVehicle::getLaneChangeModel() const {
	return *myLaneChangeModel;
}

const std::vector<MSVehicle::LaneQ> &
MSVehicle::getBestLanes(bool forceRebuild, MSLane* startLane) const {
#ifdef DEBUG_VEHICLE_GUI_SELECTION
	if (gSelected.isSelected(GLO_VEHICLE, static_cast<const GUIVehicle*>(this)->getGlID())) {
		int bla = 0;
		myLastBestLanesEdge = 0;
	}
#endif

	if (startLane == 0) {
		startLane = myLane;
	}
	// update occupancy and current lane index, only, if the vehicle has not moved to a new lane
	if (myLastBestLanesEdge == &startLane->getEdge() && !forceRebuild) {
		std::vector<LaneQ> &lanes = *myBestLanes.begin();
		std::vector<LaneQ>::iterator i;
		for (i = lanes.begin(); i != lanes.end(); ++i) {
			SUMOReal nextOccupation = 0;
			for (std::vector<MSLane*>::const_iterator j =
					(*i).bestContinuations.begin() + 1;
					j != (*i).bestContinuations.end(); ++j) {
				nextOccupation += (*j)->getVehLenSum();
			}
			(*i).nextOccupation = nextOccupation;
			if ((*i).lane == startLane) {
				myCurrentLaneInBestLanes = i;
			}
		}
		return *myBestLanes.begin();
	}
	// start rebuilding
	myLastBestLanesEdge = &startLane->getEdge();
	myBestLanes.clear();

	// get information about the next stop
	const MSEdge* nextStopEdge = 0;
	const MSLane* nextStopLane = 0;
	SUMOReal nextStopPos = 0;
	if (!myStops.empty()) {
		const Stop& nextStop = myStops.front();
		nextStopLane = nextStop.lane;
		nextStopEdge = &nextStopLane->getEdge();
		nextStopPos = nextStop.startPos;
	}
	if (myParameter->arrivalLaneProcedure == ARRIVAL_LANE_GIVEN
			&& nextStopEdge == 0) {
		nextStopEdge = *(myRoute->end() - 1);
		nextStopLane = nextStopEdge->getLanes()[myParameter->arrivalLane];
		nextStopPos = myArrivalPos;
	}

	// go forward along the next lanes;
	int seen = 0;
	SUMOReal seenLength = 0;
	bool progress = true;
	for (MSRouteIterator ce = myCurrEdge; progress;) {
		std::vector<LaneQ> currentLanes;
		const std::vector<MSLane*> *allowed = 0;
		const MSEdge* nextEdge = 0;
		if (ce != myRoute->end() && ce + 1 != myRoute->end()) {
			nextEdge = *(ce + 1);
			allowed = (*ce)->allowedLanes(*nextEdge, myType->getVehicleClass());
		}
		const std::vector<MSLane*> &lanes = (*ce)->getLanes();
		for (std::vector<MSLane*>::const_iterator i = lanes.begin();
				i != lanes.end(); ++i) {
			LaneQ q;
			MSLane* cl = *i;
			q.lane = cl;
			q.bestContinuations.push_back(cl);
			q.bestLaneOffset = 0;
			q.length = cl->getLength();
			q.allowsContinuation = allowed == 0
					|| find(allowed->begin(), allowed->end(), cl)
							!= allowed->end();
			currentLanes.push_back(q);
		}
		//
		if (nextStopEdge == *ce) {
			progress = false;
			for (std::vector<LaneQ>::iterator q = currentLanes.begin();
					q != currentLanes.end(); ++q) {
				if (nextStopLane != (*q).lane) {
					(*q).allowsContinuation = false;
					(*q).length = nextStopPos;
				}
			}
		}

		myBestLanes.push_back(currentLanes);
		++seen;
		seenLength += currentLanes[0].lane->getLength();
		++ce;
		progress &= (seen <= 4 || seenLength < 3000);
		progress &= seen <= 8;
		progress &= ce != myRoute->end();
		/*
		 if(progress) {
		 progress &= (currentLanes.size()!=1||(*ce)->getLanes().size()!=1);
		 }
		 */
	}

	// we are examining the last lane explicitly
	if (myBestLanes.size() != 0) {
		SUMOReal bestLength = -1;
		int bestThisIndex = 0;
		int index = 0;
		std::vector<LaneQ> &last = myBestLanes.back();
		for (std::vector<LaneQ>::iterator j = last.begin(); j != last.end();
				++j, ++index) {
			if ((*j).length > bestLength) {
				bestLength = (*j).length;
				bestThisIndex = index;
			}
		}
		index = 0;
		for (std::vector<LaneQ>::iterator j = last.begin(); j != last.end();
				++j, ++index) {
			if ((*j).length < bestLength) {
				(*j).bestLaneOffset = bestThisIndex - index;
			}
		}
	}

	// go backward through the lanes
	// track back best lane and compute the best prior lane(s)
	for (std::vector<std::vector<LaneQ> >::reverse_iterator i =
			myBestLanes.rbegin() + 1; i != myBestLanes.rend(); ++i) {
		std::vector<LaneQ> &nextLanes = (*(i - 1));
		std::vector<LaneQ> &clanes = (*i);
		MSEdge& cE = clanes[0].lane->getEdge();
		int index = 0;
		SUMOReal bestConnectedLength = -1;
		SUMOReal bestLength = -1;
		for (std::vector<LaneQ>::iterator j = nextLanes.begin();
				j != nextLanes.end(); ++j, ++index) {
			if ((*j).lane->isApproachedFrom(&cE)
					&& bestConnectedLength < (*j).length) {
				bestConnectedLength = (*j).length;
			}
			if (bestLength < (*j).length) {
				bestLength = (*j).length;
			}
		}
		if (bestConnectedLength > 0) {
			int bestThisIndex = 0;
			index = 0;
			for (std::vector<LaneQ>::iterator j = clanes.begin();
					j != clanes.end(); ++j, ++index) {
				LaneQ bestConnectedNext;
				bestConnectedNext.length = -1;
				if ((*j).allowsContinuation) {
					for (std::vector<LaneQ>::const_iterator m =
							nextLanes.begin(); m != nextLanes.end(); ++m) {
						if ((*m).lane->isApproachedFrom(&cE, (*j).lane)) {
							if (bestConnectedNext.length < (*m).length
									|| (bestConnectedNext.length == (*m).length
											&& abs(
													bestConnectedNext.bestLaneOffset)
													> abs((*m).bestLaneOffset))) {
								bestConnectedNext = *m;
							}
						}
					}
					if (bestConnectedNext.length == bestConnectedLength
							&& abs(bestConnectedNext.bestLaneOffset) < 2) {
						(*j).length += bestLength;
					} else {
						(*j).length += bestConnectedNext.length;
					}
				}
				if (clanes[bestThisIndex].length < (*j).length
						|| (clanes[bestThisIndex].length == (*j).length
								&& abs(
										abs(
												clanes[bestThisIndex].bestLaneOffset
														> (*j).bestLaneOffset)))) {
					bestThisIndex = index;
				}
				copy(bestConnectedNext.bestContinuations.begin(),
						bestConnectedNext.bestContinuations.end(),
						back_inserter((*j).bestContinuations));
			}

			index = 0;
			for (std::vector<LaneQ>::iterator j = clanes.begin();
					j != clanes.end(); ++j, ++index) {
				if ((*j).length < clanes[bestThisIndex].length
						|| ((*j).length == clanes[bestThisIndex].length
								&& abs((*j).bestLaneOffset)
										< abs(
												clanes[bestThisIndex].bestLaneOffset))) {
					(*j).bestLaneOffset = bestThisIndex - index;
				} else {
					(*j).bestLaneOffset = 0;
				}
			}

		} else {

			int bestThisIndex = 0;
			int bestNextIndex = 0;
			int bestDistToNeeded = (int) clanes.size();
			index = 0;
			for (std::vector<LaneQ>::iterator j = clanes.begin();
					j != clanes.end(); ++j, ++index) {
				if ((*j).allowsContinuation) {
					int nextIndex = 0;
					for (std::vector<LaneQ>::const_iterator m =
							nextLanes.begin(); m != nextLanes.end();
							++m, ++nextIndex) {
						if ((*m).lane->isApproachedFrom(&cE, (*j).lane)) {
							if (bestDistToNeeded > abs((*m).bestLaneOffset)) {
								bestDistToNeeded = abs((*m).bestLaneOffset);
								bestThisIndex = index;
								bestNextIndex = nextIndex;
							}
						}
					}
				}
			}
			clanes[bestThisIndex].length += nextLanes[bestNextIndex].length;
			copy(nextLanes[bestNextIndex].bestContinuations.begin(),
					nextLanes[bestNextIndex].bestContinuations.end(),
					back_inserter(clanes[bestThisIndex].bestContinuations));
			index = 0;
			for (std::vector<LaneQ>::iterator j = clanes.begin();
					j != clanes.end(); ++j, ++index) {
				if ((*j).length < clanes[bestThisIndex].length
						|| ((*j).length == clanes[bestThisIndex].length
								&& abs((*j).bestLaneOffset)
										< abs(
												clanes[bestThisIndex].bestLaneOffset))) {
					(*j).bestLaneOffset = bestThisIndex - index;
				} else {
					(*j).bestLaneOffset = 0;
				}
			}

		}

	}

	// update occupancy and current lane index
	std::vector<LaneQ> &currLanes = *myBestLanes.begin();
	std::vector<LaneQ>::iterator i;
	for (i = currLanes.begin(); i != currLanes.end(); ++i) {
		SUMOReal nextOccupation = 0;
		for (std::vector<MSLane*>::const_iterator j =
				(*i).bestContinuations.begin() + 1;
				j != (*i).bestContinuations.end(); ++j) {
			nextOccupation += (*j)->getVehLenSum();
		}
		(*i).nextOccupation = nextOccupation;
		if ((*i).lane == startLane) {
			myCurrentLaneInBestLanes = i;
		}
	}
	return *myBestLanes.begin();
}

const std::vector<MSLane*> &
MSVehicle::getBestLanesContinuation() const {
	if (myBestLanes.empty() || myBestLanes[0].empty()
			|| myLane->getEdge().getPurpose()
					== MSEdge::EDGEFUNCTION_INTERNAL) {
		return myEmptyLaneVector;
	}
	return (*myCurrentLaneInBestLanes).bestContinuations;
}

const std::vector<MSLane*> &
MSVehicle::getBestLanesContinuation(const MSLane* const l) const {
	for (std::vector<std::vector<LaneQ> >::const_iterator i =
			myBestLanes.begin(); i != myBestLanes.end(); ++i) {
		if ((*i).size() != 0 && (*i)[0].lane == l) {
			return (*i)[0].bestContinuations;
		}
	}
	return myEmptyLaneVector;
}

SUMOReal MSVehicle::getDistanceToPosition(SUMOReal destPos,
		const MSEdge* destEdge) {
#ifdef DEBUG_VEHICLE_GUI_SELECTION
	SUMOReal distance = 1000000.;
#else
	SUMOReal distance = std::numeric_limits<SUMOReal>::max();
#endif
	if (isOnRoad() && destEdge != NULL) {
		if (&myLane->getEdge() == *myCurrEdge) {
			// vehicle is on a normal edge
			distance = myRoute->getDistanceBetween(getPositionOnLane(), destPos,
					*myCurrEdge, destEdge);
		} else {
			// vehicle is on inner junction edge
			distance = myLane->getLength() - getPositionOnLane();
			distance += myRoute->getDistanceBetween(0, destPos,
					*(myCurrEdge + 1), destEdge);
		}
	}
	return distance;
}

SUMOReal MSVehicle::getHBEFA_CO2Emissions() const {
	return HelpersHBEFA::computeCO2(myType->getEmissionClass(), myState.speed(),
			myPreDawdleAcceleration);
}

SUMOReal MSVehicle::getHBEFA_COEmissions() const {
	return HelpersHBEFA::computeCO(myType->getEmissionClass(), myState.speed(),
			myPreDawdleAcceleration);
}

SUMOReal MSVehicle::getHBEFA_HCEmissions() const {
	return HelpersHBEFA::computeHC(myType->getEmissionClass(), myState.speed(),
			myPreDawdleAcceleration);
}

SUMOReal MSVehicle::getHBEFA_NOxEmissions() const {
	return HelpersHBEFA::computeNOx(myType->getEmissionClass(), myState.speed(),
			myPreDawdleAcceleration);
}

SUMOReal MSVehicle::getHBEFA_PMxEmissions() const {
	return HelpersHBEFA::computePMx(myType->getEmissionClass(), myState.speed(),
			myPreDawdleAcceleration);
}

SUMOReal MSVehicle::getHBEFA_FuelConsumption() const {
	return HelpersHBEFA::computeFuel(myType->getEmissionClass(),
			myState.speed(), myPreDawdleAcceleration);
}

SUMOReal MSVehicle::getHarmonoise_NoiseEmissions() const {
	return HelpersHarmonoise::computeNoise(myType->getEmissionClass(),
			myState.speed(), myPreDawdleAcceleration);
}

void MSVehicle::addPerson(MSPerson* person) {
	if (myPersonDevice == 0) {
		myPersonDevice = MSDevice_Person::buildVehicleDevices(*this, myDevices);
		myMoveReminders.push_back(std::make_pair(myPersonDevice, 0.));
	}
	myPersonDevice->addPerson(person);
	if (myStops.size() > 0 && myStops.front().reached
			&& myStops.front().triggered) {
		myStops.front().duration = 0;
	}
}

void MSVehicle::setBlinkerInformation() {
	switchOffSignal(VEH_SIGNAL_BLINKER_RIGHT | VEH_SIGNAL_BLINKER_LEFT);
	int state = getLaneChangeModel().getOwnState();
	if ((state & LCA_LEFT) != 0) {
		switchOnSignal(VEH_SIGNAL_BLINKER_LEFT);
	} else if ((state & LCA_RIGHT) != 0) {
		switchOnSignal(VEH_SIGNAL_BLINKER_RIGHT);
	} else {
		const MSLane* lane = getLane();
		MSLinkCont::const_iterator link = lane->succLinkSec(*this, 1, *lane,
				getBestLanesContinuation());
		if (link != lane->getLinkCont().end()
				&& lane->getLength() - getPositionOnLane()
						< lane->getMaxSpeed() * (SUMOReal) 7.) {
			switch ((*link)->getDirection()) {
			case LINKDIR_TURN:
			case LINKDIR_LEFT:
			case LINKDIR_PARTLEFT:
				switchOnSignal(VEH_SIGNAL_BLINKER_LEFT);
				break;
			case LINKDIR_RIGHT:
			case LINKDIR_PARTRIGHT:
				switchOnSignal(VEH_SIGNAL_BLINKER_RIGHT);
				break;
			default:
				break;
			}
		}
	}
}

/*   The C++ linker mandates not to be declared in header if not shared by both
 * the file and its header file
 */
static SUMOReal aggredatedEnergyCosts;

//Added by HI-Iberia (emartin or doancea)
void MSVehicle::writeFevSyntheticData(MSLane*& lastLane)
{
  OptionsCont &oc = OptionsCont::getOptions();
  if(oc.getSimulationVerbosity()>1)
    std::cout<<"----> MSVehicle::writeFevSyntheticData(...)"<<std::endl;
#ifdef IGNORE_INTERNALS_IN_SYNTHETIC_DATA
  if(lastLane->getEdge().getID().at(0)!=':')
  {
#endif
    MSDevice_FEV *aux1 = NULL;
    OutputDevice& of = OutputDevice::getDeviceByOption("netstate-dump-fev");   

    for(std::vector<MSDevice*>::const_iterator it=getDevices().begin();
     it != getDevices().end();
     ++it)
    {
      aux1=dynamic_cast<MSDevice_FEV*>(*it);
      if(aux1)
        break;
    }    
    /* This can cause problems if not all vehicles had the MSDevice_FEV */
    if(isOnRoad())
    {
        //SUMOTime time_on_lane = MSNet::getInstance()->getCurrentTimeStep()-veh.myState.enterLane;
        SUMOTime time_on_lane
          = MSNet::getInstance()->getCurrentTimeStep()
          - getMyState().getEnterLane();
        if(oc.getSimulationVerbosity()>2)
          std::cout<<" time_on_lane = "<<time_on_lane<<std::endl;        
        double time_ = time_on_lane;
        if(oc.getSimulationVerbosity()>2)
          std::cout<<" time_ = "<<time_<<std::endl;
        if(time_ != 0)
          time_ /= 1000; //time_step_to_seconds;
        if(oc.getSimulationVerbosity()>2)
          std::cout<<" time_ = "<<time_<<std::endl;
        of.openTag("Register");
        double a = myState.getMyPosOnLane();
        double b = myState.getMyEnterPosLane();
        if(a == b)
        {
          /*   The vehicle has changed from the current edge in one time step,
           * then we compute avg_spped until the end of the current lane
           */
          a = lastLane->getLength();
        }
        if(oc.getSimulationVerbosity()>2)
          std::cout<<" a-b (distance) = "<<a-b<<std::endl;
        double avg_speed = (a - b) / time_;
        if(oc.getSimulationVerbosity()>2)
          std::cout<<" avg_speed = "<<avg_speed<<std::endl;
        avg_speed *= 3.6; // convert m/s to Km/h
        //avg_speed *= 10; // FIXME patch for adjusting
        if(oc.getSimulationVerbosity()>2)
        {
          std::cout<<" strcmp(getID().c_str(),\"fev0\")="<<strcmp(getID().c_str(),"fev0")<<std::endl;
          std::cout<<" strcmp(getID().c_str(),\"normVeh004\")="<<strcmp(getID().c_str(),"normVeh004")<<std::endl;
          std::cout<<" {MSVehicle}getID()="<<getID()<<std::endl;
          std::cout<<" avg_speed = "<<avg_speed<<std::endl;
        }
        double ampheres_spent_segment = aux1->getMyCurrChargeRem()
          - myState.getMyLastChargeRemoved();
        /* Segfault may has happened above */

        // Watts = Amps x Volts
        double mean_charge_removed_by_edge = ampheres_spent_segment
          * aux1->getMyCurrE();
        /* Segfault may has happened above */

        //Code//MyHandler2 handler;
        //Code//
        //Code//std::vector<VehicleToTrack> result = handler.vehicles;
        //Code//
        //Code//handler.showVehiclesToTrack(result);

        //std::cout << " Goes here..!" << std::endl;

        // Driver type: Cautious
        if (false) {
            mean_charge_removed_by_edge *=1;
        }

        // Driver type: Normal
        if (false) {
            mean_charge_removed_by_edge *= 1.1;
        }

        // Driver type: Aggressive
        if (false) {
            mean_charge_removed_by_edge *= 1.3;
        }

        //Adds aggregated energy costs.
        aggredatedEnergyCosts += mean_charge_removed_by_edge;
        nonAggregatedEnergyCosts += mean_charge_removed_by_edge;

        //Makes addition of the energy costs.
        
        of
          <<" AuxStatus=\"0\"" // TODO
          /* AverageSpeed should be exported as km/h */
          <<" AverageSpeed=\"" << avg_speed * 3.6 << "\""
          <<" BatteryCapacity=\"" << getMyType()->getCapacity() * getMyType()->getNumCells() << "\""
          <<" BatterySOC=\""<< OutputDevice::realString(1 - (aux1->getMyCurrDoD()), 6)<< "\""
          <<" BatterySOH=\""<< OutputDevice::realString(aux1->getMySoh(), 6) << "\""
          <<" Direction=\""<<(lastLane->getID().at(0)!='-'?1:-1)<<"\"" // VERIFYME
          //<<" LaneId=\"" << lastLane->getID() << "\""
        ;
        std::string link_id;
        if(lastLane->getEdge().getID().at(0)=='-')
          link_id = lastLane->getEdge().getID().substr(1);
        else
          link_id = lastLane->getEdge().getID();
        of
          <<" LinkId=\"" << link_id << "\""
          <<" NECF=\"" << mean_charge_removed_by_edge << "\""
          //<< " NumCell=\"" << veh.getMyType()->getNumCells() <<"\""
          <<" RelHumidity=\"" << getMyType()->getHumidity() << "\""
          <<" Temperature=\"" << getMyType()->getCelsius() << "\""
        ;
        char *buffer = (char*)malloc(64);
        //struct tm *tm_ = gmtime (
        time_t start;
        if(oc.getAppName().substr(0,8)=="sumo-gui")
          //start = 0; // TODO
          start = *MyHandler2::getInitialTime(); // What does the upper mean ? ---> FIXME IS BECAUSE LOADER DOES NOT LOAD GOOD. PLEASE SOLVE.
        else if(oc.getAppName().substr(0,4)=="sumo")
          start = *MyHandler2::getInitialTime();
        else
        {
          std::cout<<"CRITICAL printing netstate dump file"<<std::endl;
          exit(0xffff);
        }
        time_t then = start + MSNet::getInstance()->getCurrentTimeStep()/SECS_IN_CURENT_TIME_STEP;
        //strftime(buffer, 64, "%Y-%m-%dT%H:%M:%S", gmtime(MyHandler2::getInitialTime() + MSNet::getInstance()->getCurrentTimeStep()));
        strftime(buffer, 64, "%Y-%m-%dT%H:%M:%S", gmtime(&then));
        of
          <<" TimeStamp=\""<<buffer<<"\"" // TODO
        ;
        of
          <<" TravelTime=\""<< time2string(time_on_lane) << "\""
        ;
        time_t now;
        time(&now);
        strftime(buffer, 64, "%Y-%m-%dT%H:%M:%S", gmtime(&now));
        of
          <<" UploadTime=\""<<buffer<< "\"" // TODO
        ;
        of
          <<" VIN=\""<< getID() << "\""
          <<" Weight=\""<< getMyType()->getMass() << "\""
                    
          //<< " AuxMask=\"" << getMyType()->getAuxMask() << "\""
          //<< " LightMask=\"" << getMyType()->getLightMask() << "\""
          
          /*
          << " enter_time_on_lane=\"" << time2string(veh.getMyState().getEnterLane()) <<"\""
          << " time_on_lane =\"" << time2string(time_on_lane) << "\""
          << " enter_pos =\"" << veh.getMyState().getMyPosOnLane() <<"\""
          << " curr_pos =\"" << veh.getPositionOnLane() <<"\""
          << " instantaneous_speed=\"" << veh.getSpeed() <<"\""
          */
          
          /*
          << " EnergyCostByEdge =\""<< mean_charge_removed_by_edge << "\""
          << " AggregatedEnergyCost=\"" << aggredatedEnergyCosts << "\""
          << " NonAggregatedEnergyCost =\"" << nonAggregatedEnergyCosts << "\""
          */          
        ;
        of.closeTag(true);
        free(buffer);
    }
    // Write down the aggregated Energy costs.
    //Code//of.openTag("AggregatedEnergyCost");
    //Code//of << " EnergyCost=\"" << aggredatedEnergyCosts << "\"";
    //Code//of.closeTag(true);
#ifdef IGNORE_INTERNALS_IN_SYNTHETIC_DATA
  }
  else
  {
    /* Is internal and ignored */
    ;
  }
#endif
}

// Added by HI-Iberia (uprego)
void MSVehicle::writeNonFevSyntheticData(MSLane*& lastLane)
{
  OutputDevice& of = OutputDevice::getDeviceByOption("netstate-dump-fev");
#ifdef IGNORE_NONFEV_SYNTH_DATA
  if(false)
#else
  if(isOnRoad())
#endif
  {
    //SUMOTime time_on_lane = MSNet::getInstance()->getCurrentTimeStep()-veh.myState.enterLane;
    SUMOTime time_on_lane = MSNet::getInstance()->getCurrentTimeStep()
            - getMyState().getEnterLane();
    double time = time_on_lane;

    if (time != 0)
        time = time / 1000; //time_step_to_seconds;
    of.openTag("Register");
    double a = myState.getMyPosOnLane();
    double b = myState.getMyEnterPosLane();

    //The vehicle has changed from the current edge in one time step,
    //then we compute avg_spped until the end of the current lane

    if(a == b)
    {
      a = lastLane->getLength();
    }
    double avg_speed = (a - b) / time;
    avg_speed *= 3.6; // convert m/s to Km/h
    
    //return; // FIXME ALERT
   
    of << " timeStampUTC=\""
       << time2string(MSNet::getInstance()->getCurrentTimeStep())
       << "\"" << " LinkId=\"" << lastLane->getEdge().getID() << "\""
       << " LaneId=\"" << lastLane->getID() << "\"" << " VIN=\""
       << getID() << "\"" << " VehicleWeight=\""
       << getMyType()->getMass() << "\""
       << " AuxMask=\"" << getMyType()->getAuxMask() << "\""
       << " LightMask=\"" << getMyType()->getLightMask() << "\""
       << " EnvTemp=\"" << getMyType()->getCelsius() << "\""
       << " EnvHum=\"" << getMyType()->getHumidity() << "\""
       /*
       << " enter_time_on_lane=\"" << time2string(veh.getMyState().getEnterLane()) <<"\""
       << " time_on_lane =\"" << time2string(time_on_lane) << "\""
       << " enter_pos =\"" << veh.getMyState().getMyPosOnLane() <<"\""
       << " curr_pos =\"" << veh.getPositionOnLane() <<"\""
       << " instantaneous_speed=\"" << veh.getSpeed() <<"\""
       */
       << " Avg_speed =\"" << avg_speed << "\"" << " TravelTime =\""
       << time2string(time_on_lane) << "\""
       ;
    of.closeTag(true);
  }
}

/*

  (UXIO) Original implementation of this function by emartin and/or doancea
  
//Added by HI-Iberia
void MSVehicle::writeSyntheticData(MSLane*& lastLane) {
    MSDevice_FEV * aux1 = NULL;
    OutputDevice& of = OutputDevice::getDeviceByOption("netstate-dump-fev");

    //Holds the aggregated energy costs.
    static double aggredatedEnergyCosts;
    
    //Holds the energy cost by edge.
    //Code//double mean_charge_removed_by_edge;
    

    for (std::vector<MSDevice*>::const_iterator it = getDevices().begin();
            it != getDevices().end(); ++it) {
        aux1 = dynamic_cast<MSDevice_FEV*>(*it);
        if (aux1) {
            break;
        }
    }
    if (isOnRoad()) {
        //SUMOTime time_on_lane = MSNet::getInstance()->getCurrentTimeStep()-veh.myState.enterLane;
        SUMOTime time_on_lane = MSNet::getInstance()->getCurrentTimeStep()
                - getMyState().getEnterLane();
        double time = time_on_lane;

        if (time != 0)
            time = time / 1000; //time_step_to_seconds;
        of.openTag("Register");
        double a = myState.getMyPosOnLane();
        double b = myState.getMyEnterPosLane();

        //The vehicle has changed from the current edge in one time step,
        //then we compute avg_spped until the end of the current lane

        if (a == b) {
            a = lastLane->getLength();
        }
        double avg_speed = (a - b) / time;
        avg_speed *= 3.6; // convert m/s to Km/h
        double ampheres_spent_segment = aux1->getMyCurrChargeRem()
                - myState.getMyLastChargeRemoved();

        //Whatts = Amphs x Volts
        double mean_charge_removed_by_edge = ampheres_spent_segment
                * aux1->getMyCurrE();

        //Code//MyHandler2 handler;
        //Code//
        //Code//std::vector<VehicleToTrack> result = handler.vehicles;
        //Code//
        //Code//handler.showVehiclesToTrack(result);

        //std::cout << " Goes here..!" << std::endl;

        // Driver type: Cautious
        if (false) {
            mean_charge_removed_by_edge *=1;
        }

        // Driver type: Normal
        if (false) {
            mean_charge_removed_by_edge *= 1.1;
        }

        // Driver type: Aggressive
        if (false) {
            mean_charge_removed_by_edge *= 1.3;
        }

        //Adds aggregated energy costs.
        aggredatedEnergyCosts += mean_charge_removed_by_edge;

        //Makes addition of the energy costs.

        of << " timeStampUTC=\""
                << time2string(MSNet::getInstance()->getCurrentTimeStep())
                << "\"" << " LinkId=\"" << lastLane->getEdge().getID() << "\""
                << " LaneId=\"" << lastLane->getID() << "\"" << " VIN=\""
                << getID() << "\"" << " VehicleWeight=\""
                << getMyType()->getMass() << "\""
                //<< " NumCell=\"" << veh.getMyType()->getNumCells() <<"\""
                << " BatterySOH=\""
                << OutputDevice::realString(aux1->getMySoh(), 6) << "\""
                << " BatterySOC=\""
                << OutputDevice::realString(1.0 - (aux1->getMyCurrDoD()), 6)
                << "\"" << " BatteryCapacity=\"" << getMyType()->getCapacity()
                << "\"" << " AuxMask=\"" << getMyType()->getAuxMask() << "\""
                << " LightMask=\"" << getMyType()->getLightMask() << "\""
                << " EnvTemp=\"" << getMyType()->getCelsius() << "\""
                << " EnvHum=\"" << getMyType()->getHumidity() << "\""
                //      << " enter_time_on_lane=\"" << time2string(veh.getMyState().getEnterLane()) <<"\""
                //      << " time_on_lane =\"" << time2string(time_on_lane) << "\""
                //      << " enter_pos =\"" << veh.getMyState().getMyPosOnLane() <<"\""
                //      << " curr_pos =\"" << veh.getPositionOnLane() <<"\""
                //      << " instantaneous_speed=\"" << veh.getSpeed() <<"\""
                << " Avg_speed =\"" << avg_speed << "\"" << " TravelTime =\""
                << time2string(time_on_lane) << "\"" << " EnergyCostByEdge =\""
                << mean_charge_removed_by_edge << "\""
                << " AggregatedEnergyCost =\"" << aggredatedEnergyCosts << "\"";
        of.closeTag(true);
    }

    // Write down the aggregated Energy costs.
    //Code//of.openTag("AggregatedEnergyCost");
    //Code//of << " EnergyCost=\"" << aggredatedEnergyCosts << "\"";
    //Code//of.closeTag(true);
}
*/

void MSVehicle::replaceVehicleType(MSVehicleType* type) {
	if (myType->amVehicleSpecific()) {
		delete myType;
	}
	myType = type;
}

unsigned int MSVehicle::getLaneIndex() const {
	std::vector<MSLane*>::const_iterator laneP = std::find(
			(*myCurrEdge)->getLanes().begin(), (*myCurrEdge)->getLanes().end(),
			myLane);
	return (unsigned int) std::distance((*myCurrEdge)->getLanes().begin(),
			laneP);
}

#ifndef NO_TRACI
bool MSVehicle::addTraciStop(MSLane* lane, SUMOReal pos, SUMOReal /*radius*/,
		SUMOTime duration) {
	//if the stop exists update the duration
	for (std::list<Stop>::iterator iter = myStops.begin();
			iter != myStops.end(); iter++) {
		if (iter->lane == lane && fabs(iter->endPos - pos) < POSITION_EPS) {
			if (duration == 0 && !iter->reached) {
				myStops.erase(iter);
			} else {
				iter->duration = duration;
			}
			return true;
		}
	}

	SUMOVehicleParameter::Stop newStop;
	newStop.lane = lane->getID();
	newStop.busstop = MSNet::getInstance()->getBusStopID(lane, pos);
	newStop.startPos = pos - POSITION_EPS;
	newStop.endPos = pos;
	newStop.duration = duration;
	newStop.until = -1;
	newStop.triggered = false;
	newStop.parking = false;
	newStop.index = STOP_INDEX_END;
	return addStop(newStop);
}

MSVehicle::Influencer&
MSVehicle::getInfluencer() {
	if (myInfluencer == 0) {
		myInfluencer = new Influencer();
	}
	return *myInfluencer;
}

SUMOReal MSVehicle::getSpeedWithoutTraciInfluence() const {
	if (myInfluencer != 0) {
		return myInfluencer->getOriginalSpeed();
	}
	return myState.mySpeed;
}

SUMOTime MSVehicle::State::getEnterLane() const {
	return myTimeEnterLane;
}

double MSVehicle::State::getMyPosOnLane() const {
	return myPosOnLane;
}

double MSVehicle::State::getMyPos() const {
	return myPos;
}

SUMOReal MSVehicle::State::getMySpeed() const {
	return mySpeed;
}

void MSVehicle::State::setMyPosOnLane(SUMOReal aux) {
  //myOldPosOnLane = myPosOnLane;
	myPosOnLane = aux;
}

void MSVehicle::State::setEnterLane(SUMOTime enterLaneI) {
	myTimeEnterLane = enterLaneI;
}

void MSVehicle::State::setMyPos(double myPos) {
  this->myOldPos = this->myPos;
	this->myPos = myPos;
}

void MSVehicle::State::setMySpeed(double mySpeed) {
	this->mySpeed = mySpeed;
}

MSVehicle::State MSVehicle::getMyState() const { return myState; }

MSVehicle::State MSVehicle::getMyOldState() const { return myOldState; }

void MSVehicle::setMyState(State myState)
{
  this->myOlderState = this->myOldState;
  this->myOldState = this->myState;
  this->myState = myState;
}

MSLane *MSVehicle::getMyLane() const {
	return myLane;
}

bool MSVehicle::isHasChangeEdge() const {
	return hasChangeEdge;
}

void MSVehicle::setHasChangeEdge(bool hasChangeEdge) {
	this->hasChangeEdge = hasChangeEdge;
}

std::string MSVehicle::getMyLastEdgeId() const {
	return myLastEdgeId;
}

void MSVehicle::setMyLastEdgeId(std::string myLastEdgeId) {
	this->myLastEdgeId = myLastEdgeId;
}

double MSVehicle::State::getMyEnterPosLane() const {
	return myEnterPosLane;
}

void MSVehicle::State::setMyEnterPosLane(double myEnterPosLane) {
	this->myEnterPosLane = myEnterPosLane;
}

double MSVehicle::State::getMyLastChargeRemoved() const {
	return myLastChargeRemoved;
}

void MSVehicle::State::setMyLastChargeRemoved(double myLastChargeRemoved) {
	this->myLastChargeRemoved = myLastChargeRemoved;
}

void MSVehicle::State::addToMyPos(SUMOReal arg1)
{
  myOldPos = myPos;
  myPos += SPEED2DIST(arg1);
}

#endif
