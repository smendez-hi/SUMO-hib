/****************************************************************************/
/// @file    ROLoader.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @author  Christian Roessel
/// @date    Sept 2002
/// @version $Id: ROLoader.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// Loader for networks and route imports
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

#include <iostream>
#include <string>
#include <iomanip>
#include <xercesc/parsers/SAXParser.hpp>
#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/util/TransService.hpp>
#include <xercesc/sax2/SAX2XMLReader.hpp>
#include <utils/options/OptionsCont.h>
#include <utils/common/ToString.h>
#include <utils/common/StringTokenizer.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/UtilExceptions.h>
#include <utils/common/FileHelpers.h>
#include <utils/xml/XMLSubSys.h>
#include <utils/xml/SAXWeightsHandler.h>
#include "RONet.h"
#include "RONetHandler.h"
#include "ROLoader.h"
#include "ROEdge.h"
#include "RORDLoader_TripDefs.h"
#include "RORDLoader_SUMOBase.h"
#include "RORDGenerator_ODAmounts.h"
#include "ROAbstractRouteDefLoader.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// ROLoader::EdgeFloatTimeLineRetriever_EdgeTravelTime - methods
// ---------------------------------------------------------------------------
void
ROLoader::EdgeFloatTimeLineRetriever_EdgeTravelTime::addEdgeWeight(const std::string& id,
        SUMOReal val, SUMOReal beg, SUMOReal end) const {
    ROEdge* e = myNet.getEdge(id);
    if (e != 0) {
        e->addTravelTime(val, beg, end);
    } else {
        if (id[0] != ':') {
            WRITE_ERROR("Trying to set a weight for the unknown edge '" + id + "'.");
        }
    }
}


// ---------------------------------------------------------------------------
// ROLoader::EdgeFloatTimeLineRetriever_EdgeWeight - methods
// ---------------------------------------------------------------------------
void
ROLoader::EdgeFloatTimeLineRetriever_EdgeWeight::addEdgeWeight(const std::string& id,
        SUMOReal val, SUMOReal beg, SUMOReal end) const {
    ROEdge* e = myNet.getEdge(id);
    if (e != 0) {
        e->addEffort(val, beg, end);
    } else {
        if (id[0] != ':') {
            WRITE_ERROR("Trying to set a weight for the unknown edge '" + id + "'.");
        }
    }
}


// ---------------------------------------------------------------------------
// ROLoader - methods
// ---------------------------------------------------------------------------
ROLoader::ROLoader(OptionsCont& oc, bool emptyDestinationsAllowed)
    : myOptions(oc), myEmptyDestinationsAllowed(emptyDestinationsAllowed) {}


ROLoader::~ROLoader() {
    destroyHandlers();
}

void ROLoader::loadNet(RONet& toFill, ROAbstractEdgeBuilder& eb)
{
    std::string file = myOptions.getString("net-file");
    if (file == "") {
        throw ProcessError("Missing definition of network to load!");
    }
    if (!FileHelpers::exists(file)) {
        throw ProcessError("The network file '" + file + "' could not be found.");
    }
    PROGRESS_BEGIN_MESSAGE("Loading net");
    RONetHandler handler(toFill, eb);
    handler.setFileName(file);
    if (!XMLSubSys::runParser(handler, file)) {
        PROGRESS_FAILED_MESSAGE();
        throw ProcessError();
    } else {
        PROGRESS_DONE_MESSAGE();
    }
    if (myOptions.isSet("taz-files", false)) { // dfrouter does not register this option
        file = myOptions.getString("taz-files");
        if (!FileHelpers::exists(file)) {
            throw ProcessError("The districts file '" + file + "' could not be found.");
        }
        PROGRESS_BEGIN_MESSAGE("Loading districts");
        handler.setFileName(file);
        if (!XMLSubSys::runParser(handler, file)) {
            PROGRESS_FAILED_MESSAGE();
            throw ProcessError();
        } else {
            PROGRESS_DONE_MESSAGE();
        }
    }
}

unsigned int ROLoader::openRoutes(RONet &net)
{
  OptionsCont &oc = OptionsCont::getOptions();
  if(oc.getLoadVerbosity())
    std::cout<<"----> unsigned int ROLoader::openRoutes(RONet& net)"<<std::endl;
    bool ok;
    // build loader
    // load sumo-routes when wished
    ok = openTypedRoutes("route-files", net);
  if(oc.getLoadVerbosity())
    std::cout<<"[010] unsigned int ROLoader::openRoutes(RONet& net) - ok="<<ok<<std::endl;
    // load the XML-trip definitions when wished
    ok &= openTypedRoutes("trip-files", net);
  if(oc.getLoadVerbosity())
    std::cout<<"[020] unsigned int ROLoader::openRoutes(RONet& net) - ok="<<ok<<std::endl;
    // load the sumo-alternative file when wished
    ok &= openTypedRoutes("alternative-files", net);
  if(oc.getLoadVerbosity())
    std::cout<<"[030] unsigned int ROLoader::openRoutes(RONet& net) - ok="<<ok<<std::endl;
    // load the amount definitions if wished
    ok &= openTypedRoutes("flow-files", net);
  if(oc.getLoadVerbosity())
    std::cout<<"[040] unsigned int ROLoader::openRoutes(RONet& net) - ok="<<ok<<std::endl;
    // check
    if(ok && myHandler.size() == 0)
    {
        throw ProcessError("No route input specified.");
    }
    // skip routes prior to the begin time
    if(ok && !myOptions.getBool("unsorted-input"))
    {
        WRITE_MESSAGE("Skipping...")
        for(RouteLoaderCont::iterator i = myHandler.begin(); ok && i != myHandler.end(); i++)
        {
            ok &= (*i)->readRoutesAtLeastUntil(string2time(myOptions.getString("begin")), true);
        }
        WRITE_MESSAGE("Skipped until: " + time2string(getMinTimeStep()));
    }
    // check whether everything's ok
    if(!ok)
    {
        destroyHandlers();
        throw ProcessError();
    }
    return (unsigned int) myHandler.size();
}

void
ROLoader::processRoutesStepWise(SUMOTime start, SUMOTime end,
                                RONet& net, SUMOAbstractRouter<ROEdge, ROVehicle> &router) {
    SUMOTime absNo = end - start;
    // skip routes that begin before the simulation's begin
    // loop till the end
    bool endReached = false;
    bool errorOccured = false;
    SUMOTime time = myHandler.size() != 0 ? getMinTimeStep() : start;
    SUMOTime firstStep = time; //firstStep = 0; // HACK FIXME ALERT DANGER
    SUMOTime lastStep = time;
    for (; time < end && !errorOccured && !endReached; time += DELTA_T) {
        writeStats(time, start, absNo);
        makeSingleStep(time, net, router);
        // check whether further data exist
        endReached = !net.furtherStored();
        lastStep = time;
        for (RouteLoaderCont::iterator i = myHandler.begin(); endReached && i != myHandler.end(); i++) {
            if (!(*i)->ended()) {
                endReached = false;
            }
        }
        errorOccured = MsgHandler::getErrorInstance()->wasInformed() && !myOptions.getBool("ignore-errors");
    }
    WRITE_MESSAGE("Routes found between time steps " + time2string(firstStep) + " and " + time2string(lastStep) + ".");
}

bool ROLoader::makeSingleStep
 (SUMOTime end, RONet& net, SUMOAbstractRouter<ROEdge, ROVehicle> &router)
{
  RouteLoaderCont::iterator i;
  // Go through all handlers
  if(myHandler.size() != 0)
  {
    for(i = myHandler.begin(); i != myHandler.end(); i++)
    {
      // Load routes until the time point is reached
      if((*i)->readRoutesAtLeastUntil(end, false))
        // Save the routes
        net.saveAndRemoveRoutesUntil(myOptions, router, end);
      else
        return false;
    }
    return MsgHandler::getErrorInstance()->wasInformed();
  }
  else
    return false;
}

SUMOTime ROLoader::getMinTimeStep() const
{
  OptionsCont &oc = OptionsCont::getOptions();
  if(oc.getLoadVerbosity())
    std::cout<<"----> SUMOTime ROLoader::getMinTimeStep()"<<std::endl;  
    RouteLoaderCont::const_iterator i = myHandler.begin();
    SUMOTime ret = (*i)->getLastReadTimeStep();
    ++i;
    for (; i != myHandler.end(); i++) {
        SUMOTime akt = (*i)->getLastReadTimeStep();
        if (akt < ret) {
            ret = akt;
        }
    }
    if(oc.getLoadVerbosity())
      std::cout<<"[100] SUMOTime ROLoader::getMinTimeStep() - ret="<<ret<<std::endl;  
    return ret;
}

void ROLoader::processAllRoutes
 (SUMOTime start, SUMOTime end, RONet& net, SUMOAbstractRouter<ROEdge,
  ROVehicle> &router)
{
    long absNo = end - start;
    bool ok = true;
    for (RouteLoaderCont::iterator i = myHandler.begin(); ok && i != myHandler.end(); i++) {
        ok &= (*i)->readRoutesAtLeastUntil(SUMOTime_MAX, false);
    }
    // save the routes
    SUMOTime time = start;
    for (; time < end;) {
        writeStats(time, start, absNo);
        time = net.saveAndRemoveRoutesUntil(myOptions, router, time);
        if (time < 0) {
            time = end;
        }
    }
}

bool ROLoader::openTypedRoutes
 (const std::string &optionName, RONet &net)
{
  OptionsCont &oc = OptionsCont::getOptions();
  if(oc.getLoadVerbosity())
    std::cout<<"----> bool ROLoader::openTypedRoutes("<<optionName<<", RONet& net)"<<std::endl;  
    // check whether the current loader is known
    //  (not all routers import all route formats)
    if (!myOptions.exists(optionName)) {
        return true;
    }
    // check whether the current loader is wished
    //  and the file(s) can be used
    if (!myOptions.isUsableFileList(optionName)) {
        return !myOptions.isSet(optionName);
    }
    bool ok = true;
    std::vector<std::string> files = myOptions.getStringVector(optionName);
    for (std::vector<std::string>::const_iterator fileIt = files.begin(); fileIt != files.end(); ++fileIt) {
        // build the instance when everything's all right
        try {
            ROAbstractRouteDefLoader* instance = buildNamedHandler(optionName, *fileIt, net);
            myHandler.push_back(instance);
        } catch (ProcessError& e) {
            std::string msg = "The loader for " + optionName + " from file '" + *fileIt + "' could not be initialised;";
            std::string reason = e.what();
            if (reason != "Process Error" && reason != "") {
                msg = msg + "\n Reason: " + reason + ".";
            } else {
                msg = msg + "\n (unknown reason).";
            }
            WRITE_ERROR(msg);
            ok = false;
        }
    }
    return ok;
}

ROAbstractRouteDefLoader* ROLoader::buildNamedHandler
 (const std::string& optionName, const std::string& file, RONet& net)
{
  OptionsCont &oc = OptionsCont::getOptions();
  if(oc.getLoadVerbosity())
    std::cout<<"----> bool ROAbstractRouteDefLoader* ROLoader::buildNamedHand"<<
      "ler(optionName{"<<optionName<<"}, file{"<<file<<"}, RONet& net)"<<
      std::endl;  
    if (optionName == "route-files" || optionName == "alternative-files") {
        const SUMOReal beta = myOptions.getBool("logit")
                              ? myOptions.getFloat("logit.beta")
                              : myOptions.getFloat("gawron.beta");
        const SUMOReal gamma = myOptions.getBool("logit")
                               ? myOptions.getFloat("logit.gamma")
                               : -1;
        return new RORDLoader_SUMOBase(net,
                                       string2time(myOptions.getString("begin")), string2time(myOptions.getString("end")),
                                       beta, myOptions.getFloat("gawron.a"), gamma,
                                       myOptions.getInt("max-alternatives"), myOptions.getBool("repair"),
                                       myOptions.getBool("with-taz"), myOptions.getBool("keep-all-routes"),
                                       myOptions.getBool("skip-new-routes"), file);
    }
    if (optionName == "trip-files") {
        return new RORDLoader_TripDefs(net,
                                       string2time(myOptions.getString("begin")), string2time(myOptions.getString("end")),
                                       myEmptyDestinationsAllowed, myOptions.getBool("with-taz"), file);
    }
    if (optionName == "flow-files") {
        return new RORDGenerator_ODAmounts(net,
                                           string2time(myOptions.getString("begin")), string2time(myOptions.getString("end")),
                                           myEmptyDestinationsAllowed, myOptions.getBool("randomize-flows"), file);
    }
    return 0;
}

bool
ROLoader::loadWeights(RONet& net, const std::string& optionName,
                      const std::string& measure, bool useLanes) {
    // check whether the file exists
    if (!myOptions.isUsableFileList(optionName)) {
        return false;
    }
    // build and prepare the weights handler
    std::vector<SAXWeightsHandler::ToRetrieveDefinition*> retrieverDefs;
    //  travel time, first (always used)
    EdgeFloatTimeLineRetriever_EdgeTravelTime ttRetriever(net);
    retrieverDefs.push_back(new SAXWeightsHandler::ToRetrieveDefinition("traveltime", !useLanes, ttRetriever));
    //  the measure to use, then
    EdgeFloatTimeLineRetriever_EdgeWeight eRetriever(net);
    if (measure != "traveltime") {
        std::string umeasure = measure;
        if (measure == "CO" || measure == "CO2" || measure == "HC" || measure == "PMx" || measure == "NOx" || measure == "fuel") {
            umeasure = measure + "_perVeh";
        }
        retrieverDefs.push_back(new SAXWeightsHandler::ToRetrieveDefinition(umeasure, !useLanes, eRetriever));
    }
    //  set up handler
    SAXWeightsHandler handler(retrieverDefs, "");
    // go through files
    std::vector<std::string> files = myOptions.getStringVector(optionName);
    for (std::vector<std::string>::const_iterator fileIt = files.begin(); fileIt != files.end(); ++fileIt) {
        PROGRESS_BEGIN_MESSAGE("Loading precomputed net weights from '" + *fileIt + "'");
        if (XMLSubSys::runParser(handler, *fileIt)) {
            PROGRESS_DONE_MESSAGE();
        } else {
            WRITE_MESSAGE("failed.");
            return false;
        }
    }
    // build edge-internal time lines
    const std::map<std::string, ROEdge*> &edges = net.getEdgeMap();
    for (std::map<std::string, ROEdge*>::const_iterator i = edges.begin(); i != edges.end(); ++i) {
        (*i).second->buildTimeLines(measure);
    }
    return true;
}


void
ROLoader::writeStats(SUMOTime time, SUMOTime start, int absNo) {
    if (myOptions.getBool("verbose")) {
        SUMOReal perc = (SUMOReal)(time - start) / (SUMOReal) absNo;
        std::cout.setf(std::ios::fixed, std::ios::floatfield); // use decimal format
        std::cout.setf(std::ios::showpoint); // print decimal point
        std::cout << std::setprecision(OUTPUT_ACCURACY);
        MsgHandler::getMessageInstance()->progressMsg("Reading time step: " + time2string(time) + "  (" + time2string(time - start) + "/" + time2string(absNo) + " = " + toString(perc * 100) + "% done)       ");
    }
}


void
ROLoader::destroyHandlers() {
    for (RouteLoaderCont::const_iterator i = myHandler.begin(); i != myHandler.end(); ++i) {
        delete *i;
    }
    myHandler.clear();
}


/****************************************************************************/

