/****************************************************************************/
/// @file    sumo_main.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Axel Wegener
/// @author  Thimor Bohn
/// @author  Michael Behrisch
/// @date    Tue, 20 Nov 2001
/// @version $Id: sumo_main.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// Main for SUMO
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

#ifdef HAVE_VERSION_H
#include <version.h>
#endif

#include <ctime>
#include <string>
#include <iostream>
#include <microsim/MSNet.h>
#include <microsim/MSRoute.h>
#include <microsim/MSVehicleControl.h>
#include <netload/NLBuilder.h>
#include <netload/NLHandler.h>
#include <netload/NLTriggerBuilder.h>
#include <netload/NLEdgeControlBuilder.h>
#include <netload/NLJunctionControlBuilder.h>
#include <netload/NLDetectorBuilder.h>
#include "scenload/ScenParser.h"
#include "utils/options/OptionsCont.h"
#include "utils/options/OptionsIO.h"
#include <utils/common/MsgHandler.h>
#include <utils/common/SystemFrame.h>
#include <utils/common/UtilExceptions.h>
#include <utils/common/FileHelpers.h>
#include <utils/common/StringTokenizer.h>
#include <utils/common/ToString.h>
#include <utils/xml/XMLSubSys.h>
#include <microsim/MSFrame.h>
#include <microsim/output/MSDetectorControl.h>
#include <utils/iodevices/OutputDevice.h>

#ifdef HAVE_MESOSIM
#include <mesosim/MEVehicleControl.h>
#endif

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif

// ===========================================================================
// functions
// ===========================================================================
/* -------------------------------------------------------------------------
 * data processing methods
 * ----------------------------------------------------------------------- */
/**
 * loads the net, additional routes and the detectors
 */
MSNet* load(OptionsCont& oc)
{
  MSFrame::setMSGlobals(oc);
  MSVehicleControl* vc = 0;
#ifdef HAVE_MESOSIM
  if(MSGlobals::gUseMesoSim)
  {
    vc = new MEVehicleControl();
  }
  else
  {
    if(oc.isSet("ecoGemRouting"))
      vc = new MSVehicleControl(true);
    else if(oc.isSet("nonEcoGemRouting"))
      vc = new MSVehicleControl(false);
    else
      // Apply EcoGem routing by default
      vc = new MSVehicleControl(true);
  }
#else
  if(oc.isSet("ecoGemRouting")) // TODO change4 getBool ?
    vc = new MSVehicleControl(true);
  else if(oc.isSet("nonEcoGemRouting")) // TODO change4 getBool ?
    vc = new MSVehicleControl(false);
  else
    // Apply EcoGem routing by default
    vc = new MSVehicleControl(true);
#endif
  MSNet* net = new MSNet
   (vc, new MSEventControl(), new MSEventControl(), new MSEventControl());
  NLEdgeControlBuilder eb;
  NLDetectorBuilder db(*net);
  NLJunctionControlBuilder jb(*net, db);
  NLTriggerBuilder tb;
  NLHandler handler("", *net, db, tb, eb, jb);
  tb.setHandler(&handler);
  NLBuilder builder(oc, *net, eb, jb, db, handler);
  if(!builder.build())
  {
    delete net;
    throw ProcessError();
  }
  return net;
}

/* -------------------------------------------------------------------------
 * main
 * ----------------------------------------------------------------------- */
int main(int argc, char** argv)
{
  system("rm -f COMMAND_TRACE");
  system("rm -f NMEA_TRACE_fev0.nmea");
  system("rm -f NMEA_TRACE_fev1.nmea");
  system("rm -f NMEA_TRACE_all.nmea");
  system("rm -f NMEA_TRACE2_fev0.nmea");
  system("rm -f NMEA_TRACE2_fev1.nmea");
  system("rm -f NMEA_TRACE2_all.nmea");
  OptionsCont& oc = OptionsCont::getOptions();
  // give some application descriptions
  oc.setApplicationDescription("A microscopic road traffic simulation.");
  oc.setApplicationName("sumo",
    "SUMO sumo Version " + (std::string) VERSION_STRING +
    " adapted by HI-Iberia");
  int ret = 0;
  MSNet* net = 0;
  try
  {
    // Initialize subsystems
    XMLSubSys::init(false);
    MSFrame::fillOptions();
    OptionsIO::getOptions(true, argc, argv);
    if(oc.processMetaOptions(argc < 2))
    {
      SystemFrame::close();
      return 0;
    }
    MsgHandler::initOutputOptions();
    if(!MSFrame::checkOptions()){
      throw ProcessError();
    }
    RandHelper::initRandGlobal();
    
    /* load extra file (now has to be before the net load because 
     * route handler depends now on vehiclesToTrackList
     */
    std::string simulFile;
    ScenParser* parser;
    if(oc.isSet("extra-file"))
    {
      std::cout << "Loading extra-file: " << oc.getString("extra-file") << "\n";
      simulFile = oc.getString("extra-file");
      parser = new ScenParser(simulFile.c_str());
    }
    
    // load the net
    if(OptionsCont::getOptions().getInt("buildVerbosity") ||
       OptionsCont::getOptions().getInt("loadVerbosity"))
      std::cout<<"Loading net..."<<std::endl;
    net = load(oc);
    if(OptionsCont::getOptions().getInt("buildVerbosity") ||
       OptionsCont::getOptions().getInt("loadVerbosity"))
      std::cout<<"Net loading complete"<<std::endl;
    
    if(net != 0)
    {
      if(OptionsCont::getOptions().getInt("buildVerbosity") ||
         OptionsCont::getOptions().getInt("loadVerbosity"))
        std::cout<<"Net could be loaded"<<std::endl;
      net->checkSlopesSet();
      SUMOTime begin = string2time(oc.getString("begin"));
      SUMOTime end = string2time(oc.getString("end"));
      // report the begin when wished
      WRITE_MESSAGE("Simulation started with time: " + time2string(begin));
      if(oc.getSimulationVerbosity())
        std::cout<<"Simulation started with time: "<<time2string(begin)<<std::endl;
      // simulate
      ret = net->simulate(begin, end);
      // report the end when wished
      WRITE_MESSAGE("Simulation ended at time: " + time2string(net->getCurrentTimeStep()));
      if(oc.getSimulationVerbosity())
        std::cout<<"Simulation started with time: "<<time2string(net->getCurrentTimeStep())<<std::endl;
    }
    else
    {
      if(OptionsCont::getOptions().getBuildVerbosity() ||
         OptionsCont::getOptions().getLoadVerbosity())
        std::cout<<"Net could not be loaded"<<std::endl;
    }
    
    if(oc.isSet("extra-file"))
      delete parser;      
  }
  catch(ProcessError& e)
  {
    if(OptionsCont::getOptions().getInt("buildVerbosity") ||
       OptionsCont::getOptions().getInt("loadVerbosity"))
      std::cout<<"Got ProcessError?"<<std::endl;
    if(std::string(e.what()) != std::string("Process Error") &&
       std::string(e.what()) != std::string(""))
    {
      WRITE_ERROR(e.what());
    }    
    MsgHandler::getErrorInstance()->inform("Quitting (on error).", false);
    ret = 1;
#ifndef _DEBUG
  }
  catch(...)
  {
    if(OptionsCont::getOptions().getInt("buildVerbosity") ||
       OptionsCont::getOptions().getInt("loadVerbosity"))
      std::cout<<"Got unknown error?"<<std::endl;
    MsgHandler::getErrorInstance()->inform("Quitting (on unknown error).", false);
    ret = 1;
#endif
  }
  delete net;
  OutputDevice::closeAll();
  SystemFrame::close();
#ifndef _MSC_VER
  /*   This is a little helper for stdout prettifying. Not a valid syscall for
   * Windows systems */
  system("echo Success");
#endif
  return ret;
}
