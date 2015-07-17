/****************************************************************************/
/// @file    ROAbstractRouteDefLoader.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id: ROAbstractRouteDefLoader.h 11671 2012-01-07 20:14:30Z behrisch $
///
// The abstract base class for loading routes or route definitions
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
#ifndef ROAbstractRouteDefLoader_h
#define ROAbstractRouteDefLoader_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <utils/common/SUMOTime.h>
#include <utils/common/UtilExceptions.h>


// ===========================================================================
// class declarations
// ===========================================================================
class ROLoader;
class RONet;
class OptionsCont;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class ROAbstractRouteDefLoader
 * @brief The abstract base class for loading routes or route definitions
 *
 * XML-reading loaders are not derived directly, but use the derived
 *  ROTypedXMLRoutesLoader class as their upper class.
 */
class ROAbstractRouteDefLoader {
public:
    /** @brief Constructor
     *
     * @param[in] net The network to add loaded route definitions to
     * @param[in] begin Vehicles departing before this time shall not be loaded
     * @param[in] end Vehicles departing after this time shall not be loaded
     */
    ROAbstractRouteDefLoader(RONet& net, SUMOTime begin, SUMOTime end) ;


    /// @brief Destructor
    virtual ~ROAbstractRouteDefLoader() ;


    /// @name Methods to be implemented
    /// @{

    /** @brief Adds routes from the file until the given time is reached
     *
     * If any error occurs that may not be handled (false input, f.e.), a ProcessError
     *  which contains the error message should be thrown.
     *
     * @param[in] time The time until which route definitions shall be loaded
     * @param[in] skipping Whether routes shall not be added
     * @exception ProcessError If a major error occured
     * @return Whether any errors occured
     * @todo Recheck usage of exceptions vs. return value
     */
    virtual bool readRoutesAtLeastUntil(SUMOTime time, bool skipping) = 0;


    /** @brief Returns the time the current (last read) route starts at
     *
     * @return The least time step that was read by this reader
     */
    virtual SUMOTime getLastReadTimeStep() const = 0;


    /** @brief Returns the information whether no routes are available from this loader anymore
     *
     * @return Whether the whole input has been processed
     */
    virtual bool ended() const = 0;
    /// @}


protected:
    /// @brief The network to add routes to
    RONet& myNet;

    /// @brief The time for which the first route shall be loaded
    SUMOTime myBegin;

    /// @brief The time for which the first route shall be loaded
    SUMOTime myEnd;


private:
    /// @brief Invalidated copy constructor
    ROAbstractRouteDefLoader(const ROAbstractRouteDefLoader& src);

    /// @brief Invalidated assignment operator
    ROAbstractRouteDefLoader& operator=(const ROAbstractRouteDefLoader& src);

};


#endif

/****************************************************************************/

