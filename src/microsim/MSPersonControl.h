/****************************************************************************/
/// @file    MSPersonControl.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @date    Mon, 9 Jul 2001
/// @version $Id: MSPersonControl.h 11671 2012-01-07 20:14:30Z behrisch $
///
// Stores all persons in the net and handles their waiting for cars.
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
#ifndef MSPersonControl_h
#define MSPersonControl_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <vector>


// ===========================================================================
// class declarations
// ===========================================================================
class MSPerson;
class MSNet;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 *
 * @class MSPersonControl
 * The class is used to handle persons who are not using a transportation
 *  system but are walking or waiting. Both is processed by waiting
 *  for the arrival time / the time the waiting is over.
 */
class MSPersonControl {
public:

    typedef std::vector<MSPerson*> PersonVector;

    /// constructor
    MSPersonControl();

    /// destructor
    ~MSPersonControl();

    /// adds a single person, returns false iff an id clash occured
    bool add(const std::string& id, MSPerson* person);

    /// removes a single person
    void erase(MSPerson* person);

    /// sets the arrival time for a waiting or walking person
    void setArrival(SUMOTime time, MSPerson* person);

    /// checks whether any persons waiting or walking time is over
    void checkArrivedPersons(MSNet* net, const SUMOTime time);

    /// adds a person to the list of persons waiting for a vehicle on the specified edge
    void addWaiting(const MSEdge* edge, MSPerson* person) ;

    /** @brief board any applicable persons
     * Boards any people who wait on that edge for the given vehicle and removes them from myWaiting
     * @param[in] the edge on which the boarding should take place
     * @param[in] the vehicle which is taking on passengers
     * @return Whether any persons have been boarded
     */
    bool boardAnyWaiting(const MSEdge* edge, MSVehicle* vehicle) ;

    /// checks whether any person waits to finish her plan
    bool hasPersons() const ;

    /// checks whether any person is still engaged in walking / stopping
    bool hasPedestrians() const ;

    /// aborts the plan for any person that is still waiting for a ride
    void abortWaiting() ;

private:
    /// all persons by id
    std::map<std::string, MSPerson*> myPersons;

    /// the lists of walking / stopping persons
    std::map<SUMOTime, PersonVector> myArrivals;

    /// the lists of waiting persons
    std::map<const MSEdge*, PersonVector> myWaiting;

};


#endif

/****************************************************************************/
