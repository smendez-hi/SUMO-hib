/****************************************************************************/
/// @file    guisim_main.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @author  Felix Brack
/// @date    Tue, 20 Nov 2001
/// @version $Id: guisim_main.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// Main for GUISIM
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
#include <signal.h>
#include <iostream>
#include <fx.h>
#include <fx3d.h>
#include <microsim/MSFrame.h>
#include <microsim/MSNet.h>
#include "scenload/ScenParser.h"
#include <utils/options/Option.h>
#include <utils/options/OptionsCont.h>
#include <utils/options/OptionsIO.h>
#include <utils/common/UtilExceptions.h>
#include <utils/common/FileHelpers.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/SystemFrame.h>
#include <utils/xml/XMLSubSys.h>
#include <gui/GUIApplicationWindow.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <gui/GUIGlobals.h>
#include <guisim/GUIEdge.h>
#include <utils/gui/windows/GUIAppGlobals.h>
#include <utils/gui/images/GUITexturesHelper.h>
#include <utils/gui/settings/GUICompleteSchemeStorage.h>
#include <gui/GUIViewTraffic.h>
#include <guisim/GUIVehicle.h>
#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif
int main(int argc, char** argv)
{
  system("rm -f COMMAND_TRACE");
  system("rm -f NMEA_TRACE_fev0.nmea");
  system("rm -f NMEA_TRACE_fev1.nmea");
  system("rm -f NMEA_TRACE_all.nmea");
  system("rm -f NMEA_TRACE2_fev0.nmea");
  system("rm -f NMEA_TRACE2_fev1.nmea");
  system("rm -f NMEA_TRACE2_all.nmea");
  // make the output aware of threading
  MFXMutex lock;
  MsgHandler::assignLock(&lock);
  // get the options
  OptionsCont& oc = OptionsCont::getOptions();
  // give some application descriptions
  oc.setApplicationDescription("GUI version of the simulation SUMO.");
#ifdef _MSC_VER
  oc.setApplicationName("sumo-gui.exe", "SUMO gui Version " +
    (std::string)VERSION_STRING);
#else
  oc.setApplicationName("sumo-gui", "SUMO gui Version " +
    (std::string)VERSION_STRING);
#endif
  int ret = 0;
#ifndef _DEBUG
  try
  {
#else
  {
#endif
    // initialise subsystems
    XMLSubSys::init(false);
    MSFrame::fillOptions();
    //OptionsIO::getOptions(false, argc, argv);
    OptionsIO::getOptions(true, argc, argv);
    //if(oc.processMetaOptions(false))
    if(oc.processMetaOptions(argc < 2))
    {
      SystemFrame::close();
      return 0;
    }
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
    }else{
      MsgHandler::getErrorInstance()->inform("Quitting (no extra file provided!).",false);
      ret=1;
      SystemFrame::close();
      return ret;
    }
    // within gui-based applications, nothing is reported to the console
    MsgHandler::getErrorInstance()->report2cout(false);
    MsgHandler::getErrorInstance()->report2cerr(false);
    MsgHandler::getWarningInstance()->report2cout(false);
    MsgHandler::getWarningInstance()->report2cerr(false);
    MsgHandler::getMessageInstance()->report2cout(false);
    MsgHandler::getMessageInstance()->report2cerr(false);
    // Make application
    FXApp application("SUMO GUISimulation", "DLR");
    gFXApp = &application;
    // Open display
    application.init(argc, argv);
    int minor, major;
    if(!FXGLVisual::supported(&application, major, minor))
    {
      throw ProcessError("This system has no OpenGL support. Exiting.");
    }
    // initialise global settings
    gQuitOnEnd = oc.getBool("quit-on-end");
    gAllowTextures = !oc.getBool("disable-textures");
    /*bool ecoGemRouter = !(oc.isSet("nonEcoGemRouting")) | 
                          (oc.isSet("ecoGemRouting"));*/
    // build the main window
    GUIApplicationWindow* window;
    window = new GUIApplicationWindow(&application, "*.sumo.cfg,*.sumocfg");
    window->dependentBuild(oc.getBool("game"));
    gSchemeStorage.init(&application);
    // Create app
    application.addSignal(SIGINT, window, MID_QUIT);
    application.create();
    // Load configuration given on command line
    if(oc.isSet("configuration-file") || oc.isSet("net-file") ||
       oc.isSet("extra-file"))
    {
      window->loadOnStartup(!oc.getBool("no-start"));
    }
    // Run
    // DEBUG (UXIO) FIXME VERBOSITY
    std::cout<<"ret = application.run()\n"<<std::endl;
    ret = application.run();
#ifndef _DEBUG
  }
  catch(...)
  {
    MsgHandler::getErrorInstance()->inform("Quitting (on error).", false);
    ret = 1;
  }
#else
  }
#endif
  SystemFrame::close();
  return ret;
}