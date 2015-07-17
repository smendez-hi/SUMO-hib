/****************************************************************************/
/// @file    RORDLoader_SUMOBase.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id: RORDLoader_SUMOBase.h 11671 2012-01-07 20:14:30Z behrisch $
///
// The base class for SUMO-native route handlers
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
#ifndef RORDLoader_SUMOBase_h
#define RORDLoader_SUMOBase_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include "ROTypedXMLRoutesLoader.h"
#include <utils/xml/SUMOXMLDefinitions.h>
#include <utils/common/RGBColor.h>
#include <utils/common/SUMOVehicleParameter.h>


// ===========================================================================
// class declarations
// ===========================================================================
class SUMOVTypeParameter;
class RORouteDef;
class MsgHandler;
class RORouteDef_Alternatives;
class RORouteDef_Complete;
class RORoute;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class RORDLoader_SUMOBase
 * @brief The base class for SUMO-native route handlers
 *
 * As both the SUMO-routes and the SUMO-route alternatives handlers share
 *  a set of processing functions, we've joined these functions within a
 *  single class (this one).
 */
class RORDLoader_SUMOBase :
    public ROTypedXMLRoutesLoader {
public:
    /// Constructor
    RORDLoader_SUMOBase(RONet& net,
                        SUMOTime begin, SUMOTime end,
                        const SUMOReal beta, const SUMOReal gawronA, const SUMOReal logitGamma,
                        const int maxRouteNumber, const bool tryRepair, const bool withTaz,
                        const bool keepRoutes, const bool skipRouteCalculation,
                        const std::string& file = "") ;

    /// Destructor
    ~RORDLoader_SUMOBase() ;


    /// @name inherited from ROAbstractRouteDefLoader
    //@{

    /** @brief Returns the time the current (last read) route starts at
     *
     * @return The least time step that was read by this reader
     */
    SUMOTime getLastReadTimeStep() const {
        return myCurrentDepart;
    }
    /// @}


protected:
    /// @name inherited from GenericSAXHandler
    //@{

    /** @brief Called on the opening of a tag;
     *
     * @param[in] element ID of the currently opened element
     * @param[in] attrs Attributes within the currently opened element
     * @exception ProcessError If something fails
     * @see GenericSAXHandler::myStartElement
     */
    virtual void myStartElement(int element,
                                const SUMOSAXAttributes& attrs) ;


    /** @brief Called when characters occure
     *
     * @param[in] element ID of the last opened element
     * @param[in] chars The read characters (complete)
     * @exception ProcessError If something fails
     * @see GenericSAXHandler::myCharacters
     */
    void myCharacters(int element,
                      const std::string& chars) ;


    /** @brief Called when a closing tag occurs
     *
     * @param[in] element ID of the currently opened element
     * @exception ProcessError If something fails
     * @see GenericSAXHandler::myEndElement
     */
    virtual void myEndElement(int element) ;
    //@}

    /// Begins the parsing of the next route alternative in the file
    void startAlternative(const SUMOSAXAttributes& attrs);

    /// Begins the parsing of a route alternative of the opened route
    void startRoute(const SUMOSAXAttributes& attrs);

    bool closeVehicle() ;



    /// @name inherited from ROTypedXMLRoutesLoader
    /// @{

    /** Returns the information whether a route was read
     *
     * @return Whether a further route was read
     * @see ROTypedXMLRoutesLoader::nextRouteRead
     */
    bool nextRouteRead() {
        return myHaveNextRoute;
    }


    /** @brief Returns Initialises the reading of a further route
     *
     * @todo recheck/refactor
     * @see ROTypedXMLRoutesLoader::beginNextRoute
     */
    void beginNextRoute() ;
    //@}


protected:
    /// @brief The parsed vehicle parameter
    SUMOVehicleParameter* myVehicleParameter;

    /// @brief The color of the current route/vehicle
    RGBColor* myColor;

    /// @brief Information whether the currently parsed item is valid
    bool myCurrentIsOk;

    /// @brief Information whether the currently parsed alternatives set is valid
    bool myAltIsValid;

    /// @brief Information whether a further route has been read
    bool myHaveNextRoute;

    /// @brief The currently parsed route alternatives
    RORouteDef_Alternatives* myCurrentAlternatives;

    /// @brief The costs of the current alternative
    SUMOReal myCost;

    /// @brief The probability of the current alternative's usage
    SUMOReal myProbability;

    /// @brief gawron or logit beta - value
    const SUMOReal myBeta;

    /// @brief gawron a - value
    const SUMOReal myGawronA;

    /// @brief logit gamma - value
    const SUMOReal myLogitGamma;

    /// @brief The maximum route alternatives number
    int myMaxRouteNumber;

    /// @brief Information whether a read route shall be tried to be repaired
    bool myTryRepair;

    /// @brief Information whether zones (districts) are used as origins / destinations
    const bool myWithTaz;

    /// @brief Information whether all routes should be saved
    const bool myKeepRoutes;

    /// @brief Information whether new routes should be calculated
    const bool mySkipRouteCalculation;

    /// @brief The currently parsed route
    RORouteDef_Complete* myCurrentRoute;

    /// @brief The name of the currently parsed route
    std::string myCurrentRouteName;

    /// @brief The currently read vehicle's depart
    SUMOTime myCurrentDepart;

    /// @brief The currently parsed vehicle type
    SUMOVTypeParameter* myCurrentVType;

    bool myHaveWarnedAboutDeprecatedVType;
    bool myHaveWarnedAboutDeprecatedRoute;

private:
    /// @brief Invalidated copy constructor
    RORDLoader_SUMOBase(const RORDLoader_SUMOBase& src);

    /// @brief Invalidated assignment operator
    RORDLoader_SUMOBase& operator=(const RORDLoader_SUMOBase& src);

};


#endif

/****************************************************************************/

