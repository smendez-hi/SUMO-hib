/****************************************************************************/
/// @file    AGTrip.cpp
/// @author  Piotr Woznica
/// @author  Daniel Krajzewicz
/// @author  Walter Bamberger
/// @date    July 2010
/// @version $Id: AGTrip.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// Class containing all information of a given trip (car, bus)
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.sourceforge.net/
// Copyright (C) 2001-2012 DLR (http://www.dlr.de/) and contributors
// activitygen module
// Copyright 2010 TUM (Technische Universitaet Muenchen, http://www.tum.de/)
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

#include "AGTrip.h"


// ===========================================================================
// method definitions
// ===========================================================================
bool
AGTrip::operator <(AGTrip& trip) {
    if (getDay() < trip.getDay()) {
        return true;
    }
    if (getDay() == trip.getDay())
        if (getTime() < trip.getTime()) {
            return true;
        }
    return false;
}

void
AGTrip::print() {
    std::cout << "Trip: " << std::endl;
    std::cout << "\t-From= ";
    from.print();
    std::cout << "\t-To= ";
    to.print();
    std::cout << "\t-At= " << atTime << " -Day= " << day << std::endl;
    std::cout << "\t-Vehicle= " << vehicle << std::endl;
    std::cout << "\t-type= " << type << std::endl;
}

void
AGTrip::addLayOver(AGPosition by) {
    passBy.push_back(by);
}

void
AGTrip::addLayOver(AGTrip& trip) {
    std::list<AGPosition>::iterator it;
    for (it = trip.passBy.begin() ; it != trip.passBy.end() ; ++it) {
        passBy.push_back(*it);
    }
    passBy.push_back(trip.to);
}

void
AGTrip::addLayOverWithoutDestination(AGTrip& trip) {
    std::list<AGPosition>::iterator it;
    for (it = trip.passBy.begin() ; it != trip.passBy.end() ; ++it) {
        passBy.push_back(*it);
    }
}

std::list<AGPosition>*
AGTrip::getPassed() {
    return &passBy;
}

std::string
AGTrip::getType() {
    return type;
}

void
AGTrip::setType(std::string type) {
    this->type = type;
}

AGPosition
AGTrip::getDep() {
    return from;
}

AGPosition
AGTrip::getArr() {
    return to;
}

int
AGTrip::getTime() {
    return atTime;
}

int
AGTrip::getTimeTrip(SUMOReal secPerKm) {
    SUMOReal dist = 0;
    std::list<AGPosition> positions;
    positions.push_back(from);
    std::list<AGPosition>::iterator it;
    for (it = passBy.begin() ; it != passBy.end() ; ++it) {
        positions.push_back(*it);
    }
    positions.push_back(to);

    bool firstPass = true;
    AGPosition* temp;
    for (it = positions.begin() ; it != positions.end() ; ++it) {
        if (firstPass) {
            temp = &*it;
            continue;
        }
        dist += temp->distanceTo(*it);
        temp = &*it;
    }
    return (int)(secPerKm * (dist / 1000.0));
}

int
AGTrip::getArrTime(SUMOReal secPerKm) {
    int arrTime = atTime + getTimeTrip(secPerKm);
    return arrTime;
}

int
AGTrip::getRideBackArrTime(SUMOReal secPerKm) {
    int arrAtTime = getArrTime(secPerKm);
    int time = (int)(secPerKm * to.distanceTo(from) / 1000.0);
    int arrTime = arrAtTime + time;
    return arrTime;
}

void
AGTrip::setDepTime(int time) {
    atTime = time;
}

int
AGTrip::estimateDepTime(int arrTime, SUMOReal secPerKm) {
    int depTime = arrTime - getTimeTrip(secPerKm);
    return depTime;
}

std::string
AGTrip::getVehicleName() {
    return vehicle;
}

void
AGTrip::setVehicleName(std::string name) {
    vehicle = name;
}

void
AGTrip::setArr(AGPosition arrival) {
    to = *new AGPosition(arrival.getStreet(), arrival.getPosition());
}

void
AGTrip::setDep(AGPosition departure) {
    from = *new AGPosition(departure.getStreet(), departure.getPosition());
}

bool
AGTrip::isDaily() {
    if (day == 0) {
        return true;
    } else {
        return false;
    }
}

int
AGTrip::getDay() {
    return day;
}

void
AGTrip::setDay(int d) {
    day = d;
}

/****************************************************************************/
