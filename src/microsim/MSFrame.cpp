/****************************************************************************/
/// @file    MSFrame.cpp
/// @author  Daniel Krajzewicz
/// @author  Eric Nicolay
/// @author  Jakob Erdmann
/// @author  Axel Wegener
/// @author  Thimor Bohn
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id: MSFrame.cpp 11752 2012-01-20 11:26:53Z namdre $
///
// Sets and checks options for microsim; inits global outputs and settings
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
#include <iomanip>
#include <fstream>
#include <ctime>
#include <utils/options/OptionsCont.h>
#include <utils/options/Option.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/UtilExceptions.h>
#include <utils/common/ToString.h>
#include <utils/iodevices/OutputDevice.h>
#include <microsim/MSJunction.h>
#include <microsim/MSNet.h>
#include <microsim/MSGlobals.h>
#include <microsim/devices/MSDevice_Vehroutes.h>
#include <microsim/devices/MSDevice_Routing.h>
#include <microsim/devices/MSDevice_HBEFA.h>
#include <microsim/devices/MSDevice_FEV.h>
#include <utils/common/RandHelper.h>
#include "MSFrame.h"
#include <utils/common/SystemFrame.h>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
void
MSFrame::fillOptions() {
    OptionsCont& oc = OptionsCont::getOptions();
    oc.addCallExample("-b 0 -e 1000 -n net.xml -r routes.xml");
    oc.addCallExample("-c munich_config.cfg");
    oc.addCallExample("--help");

    // insert options sub-topics
    SystemFrame::addConfigurationOptions(oc); // fill this subtopic, too
    oc.addOptionSubTopic("Input");
    oc.addOptionSubTopic("Output");
    oc.addOptionSubTopic("Time");
    oc.addOptionSubTopic("Processing");
    SystemFrame::addReportOptions(oc); // fill this subtopic, too


    // register configuration options
    //  register input options
    oc.doRegister("net-file", 'n', new Option_FileName());
    oc.addSynonyme("net-file", "net");
    oc.addDescription("net-file", "Input", "Load road network description from FILE");

    oc.doRegister("route-files", 'r', new Option_FileName());
    oc.addSynonyme("route-files", "routes");
    oc.addDescription("route-files", "Input", "Load routes descriptions from FILE(s)");

    oc.doRegister("additional-files", 'a', new Option_FileName());
    oc.addSynonyme("additional-files", "additional");
    oc.addDescription("additional-files", "Input", "Load additional descriptions from FILE(s)");

   /* *
     * The 'extraInf' option is being added by the HI-Iberia (doancea).
     * This option shall be registered in order to be able to load additional configuration files.*/

    oc.doRegister("extra-file", 'E', new Option_FileName());
    oc.addSynonyme("extra-file", "external description file");
    oc.addDescription("extra-file", "Input", "Load scenario simulation description from FILE(s)");
    // Here ends HI-Iberia contribution.

    oc.doRegister("weight-files", 'w', new Option_FileName());
    oc.addSynonyme("weight-files", "weights");
    oc.addDescription("weight-files", "Input", "Load edge/lane weights for online rerouting from FILE");
    oc.doRegister("weight-attribute", 'x', new Option_String("traveltime"));
    oc.addSynonyme("weight-attribute", "measure", true);
    oc.addDescription("weight-attribute", "Input", "Name of the xml attribute which gives the edge weight");

#ifdef HAVE_MESOSIM
    oc.doRegister("load-state", new Option_FileName());//!!! check, describe
    oc.addDescription("load-state", "Input", "Loads a network state from FILE");
    oc.doRegister("load-state.offset", new Option_String("0", "TIME"));//!!! check, describe
    oc.addDescription("load-state.offset", "Input", "Sets the time offset for vehicle segment exit times.");
#endif

    //  register output options
    oc.doRegister("netstate-dump-fev", new Option_FileName());
    oc.doRegister("netstate-dump", new Option_FileName());
    oc.addSynonyme("netstate-dump", "ndump");
    oc.addSynonyme("netstate-dump", "netstate");
    oc.addSynonyme("netstate-dump-fev", "the simulation result file"); // Added by HI-Iberia (doancea)
    oc.addDescription("netstate-dump", "Output", "Save complete network states into FILE");
    oc.addDescription("netstate-dump-fev","Output","Save electric vehicle simulation states into a FILE"); //Added by HI-Iberia (doancea)
    oc.doRegister("netstate-dump.empty-edges", new Option_Bool(false));
    oc.addSynonyme("netstate-dump.empty-edges", "netstate.empty-edges");
    oc.addSynonyme("netstate-dump.empty-edges", "dump-empty-edges", true);
    oc.addDescription("netstate-dump.empty-edges", "Output", "Write also empty edges completely when dumping");

    oc.doRegister("summary-output", new Option_FileName());
    oc.addSynonyme("summary-output", "summary");
    oc.addSynonyme("summary-output", "emissions-output", true);
    oc.addSynonyme("summary-output", "emissions", true);
    oc.addDescription("summary-output", "Output", "Save aggregated vehicle departure info into FILE");

    oc.doRegister("tripinfo-output", new Option_FileName());
    oc.addSynonyme("tripinfo-output", "tripinfo");
    oc.addDescription("tripinfo-output", "Output", "Save single vehicle trip info into FILE");

    oc.doRegister("tripinfo-fev", new Option_FileName());
    oc.addSynonyme("tripinfo-fev", "tripinfofev");
    oc.addDescription("tripinfo-fev", "Output", "Save single vehicle trip info into FILE");

    oc.doRegister("vehroute-output", new Option_FileName());
    oc.addSynonyme("vehroute-output", "vehroutes");
    oc.addDescription("vehroute-output", "Output", "Save single vehicle route info into FILE");

    oc.doRegister("vehroute-output.exit-times", new Option_Bool(false));
    oc.addSynonyme("vehroute-output.exit-times", "vehroutes.exit-times");
    oc.addDescription("vehroute-output.exit-times", "Output", "Write the exit times for all edges");

    oc.doRegister("vehroute-output.last-route", new Option_Bool(false));
    oc.addSynonyme("vehroute-output.last-route", "vehroutes.last-route");
    oc.addDescription("vehroute-output.last-route", "Output", "Write the last route only");

    oc.doRegister("vehroute-output.sorted", new Option_Bool(false));
    oc.addSynonyme("vehroute-output.sorted", "vehroutes.sorted");
    oc.addDescription("vehroute-output.sorted", "Output", "Sorts the output by departure time");

#ifdef HAVE_MESOSIM
    oc.doRegister("save-state.times", new Option_IntVector(IntVector()));//!!! check, describe
    oc.addDescription("save-state.times", "Output", "Use INT[] as times at which a network state written");
    oc.doRegister("save-state.prefix", new Option_FileName());//!!! check, describe
    oc.addDescription("save-state.prefix", "Output", "Prefix for network states");
    oc.doRegister("save-state.files", new Option_FileName());//!!! check, describe
    oc.addDescription("save-state.files", "Output", "Files for network states");
#endif

    // register the simulation settings
    oc.doRegister("begin", 'b', new Option_String("0", "TIME"));
    oc.addDescription("begin", "Time", "Defines the begin time; The simulation starts at this time");

    oc.doRegister("end", 'e', new Option_String("-1", "TIME"));
    oc.addDescription("end", "Time", "Defines the end time; The simulation ends at this time");

#ifdef HAVE_SUBSECOND_TIMESTEPS
    oc.doRegister("step-length", new Option_String("1", "TIME"));
    oc.addDescription("step-length", "Time", "Defines the step duration");
#endif


    // register the processing options
    oc.doRegister("route-steps", 's', new Option_Integer(200));
    oc.addDescription("route-steps", "Processing", "Load routes for the next INT seconds ahead");

#ifdef HAVE_INTERNAL_LANES
    oc.doRegister("no-internal-links", new Option_Bool(false));
    oc.addDescription("no-internal-links", "Processing", "Disable (junction) internal links");
#endif

    oc.doRegister("ignore-accidents", new Option_Bool(false));
    oc.addDescription("ignore-accidents", "Processing", "Do not check whether accidents occure more deeply");

    oc.doRegister("ignore-route-errors", new Option_Bool(false));
    oc.addDescription("ignore-route-errors", "Processing", "Do not check whether routes are connected");

    oc.doRegister("max-num-vehicles", new Option_Integer(-1));
    oc.addSynonyme("max-num-vehicles", "too-many-vehicles", true);
    oc.addDescription("max-num-vehicles", "Processing", "Quit simulation if this number of vehicles is exceeded");

    oc.doRegister("incremental-dua-step", new Option_Integer());//!!! deprecated
    oc.addDescription("incremental-dua-step", "Processing", "Perform the simulation as a step in incremental DUA");
    oc.doRegister("incremental-dua-base", new Option_Integer(10));//!!! deprecated
    oc.addDescription("incremental-dua-base", "Processing", "Base value for incremental DUA");
    oc.doRegister("scale", new Option_Float());
    oc.addDescription("scale", "Processing", "Scale demand by the given factor (0..1)");

    oc.doRegister("time-to-teleport", new Option_String("300", "TIME"));
    oc.addDescription("time-to-teleport", "Processing", "Specify how long a vehicle may wait until being teleported, defaults to 300, non-positive values disable teleporting");

    oc.doRegister("max-depart-delay", new Option_String("-1", "TIME"));
    oc.addDescription("max-depart-delay", "Processing", "How long vehicles wait for departure before being skipped, defaults to -1 which means vehicles are never skipped");

    oc.doRegister("sloppy-insert", new Option_Bool(false));
    oc.addDescription("sloppy-insert", "Processing", "Whether insertion on an edge shall not be repeated in same step once failed.");

    oc.doRegister("lanechange.allow-swap", new Option_Bool(false));
    oc.addDescription("lanechange.allow-swap", "Processing", "Whether blocking vehicles trying to change lanes may be swapped.");


    // devices
    MSDevice_Routing::insertOptions();
    MSDevice_HBEFA::insertOptions();
    //MSDevice_FEV::insertOptions();



    // register report options
    oc.doRegister("no-duration-log", new Option_Bool(false));
    oc.addDescription("no-duration-log", "Report", "Disable performance reports for individual simulation steps");

    oc.doRegister("no-step-log", new Option_Bool(false));
    oc.addDescription("no-step-log", "Report", "Disable console output of current simulation step");


#ifndef NO_TRACI
    //remote port 0 if not used
    oc.addOptionSubTopic("TraCI Server");
    oc.doRegister("remote-port", new Option_Integer(0));
    oc.addDescription("remote-port", "TraCI Server", "Enables TraCI Server if set");
#ifdef HAVE_PYTHON
    oc.doRegister("python-script", new Option_String());
    oc.addDescription("python-script", "TraCI Server", "Runs TraCI script with embedded python");
#endif
#endif
    //
#ifdef HAVE_MESOSIM
    oc.addOptionSubTopic("Mesoscopic");
    oc.doRegister("mesosim", new Option_Bool(false));
    oc.addDescription("mesosim", "Mesoscopic", "Enables mesoscopic simulation");
    oc.doRegister("meso-edgelength", new Option_Float(98.0f));
    oc.addDescription("meso-edgelength", "Mesoscopic", "Length of an edge segment in mesoscopic simulation");
    oc.doRegister("meso-tauff", new Option_String("1.4", "TIME"));
    oc.addDescription("meso-tauff", "Mesoscopic", "Factor for calculating the free-free headway time");
    oc.doRegister("meso-taufj", new Option_String("1.4", "TIME"));
    oc.addDescription("meso-taufj", "Mesoscopic", "Factor for calculating the free-jam headway time");
    oc.doRegister("meso-taujf", new Option_String("2", "TIME"));
    oc.addDescription("meso-taujf", "Mesoscopic", "Factor for calculating the jam-free headway time");
    oc.doRegister("meso-taujj", new Option_String("2", "TIME"));
    oc.addDescription("meso-taujj", "Mesoscopic", "Factor for calculating the jam-jam headway time");
    oc.doRegister("meso-jam-threshold", new Option_Float(0.29f));
    oc.addDescription("meso-jam-threshold", "Mesoscopic", "Minimum percentage of occupied space to consider a segment jammed");
    oc.doRegister("meso-multi-queue", new Option_Bool(false));
    oc.addDescription("meso-multi-queue", "Mesoscopic", "Enable multiple queues at edge ends");
    oc.doRegister("meso-junction-control", new Option_Bool(false));
    oc.addDescription("meso-junction-control", "Mesoscopic", "Enable mesoscopic traffic light and priority junction handling");
    oc.doRegister("meso-recheck", new Option_String("0", "TIME"));
    oc.addDescription("meso-recheck", "Mesoscopic", "Time interval for rechecking insertion into the next segment after failure");
#endif

    // add rand options
    RandHelper::insertRandOptions();

    // add GUI options
    // the reason that we include them in vanilla sumo as well is to make reusing config files easy
    oc.addOptionSubTopic("GUI Only");
    oc.doRegister("gui-settings-file", new Option_FileName());
    oc.addDescription("gui-settings-file", "GUI Only", "Load visualisation settings from FILE");

    oc.doRegister("quit-on-end", 'Q', new Option_Bool(false));
    oc.addDescription("quit-on-end", "GUI Only", "Quits the GUI when the simulation stops");

    oc.doRegister("game", 'G', new Option_Bool(false));
    oc.addDescription("game", "GUI Only", "Start the GUI in gaming mode");

    // Needed for executing the GUI without crashing (uprego 2012-08-06)
    oc.doRegister("start", 'S', new Option_Bool(false));
    oc.addDescription("start", "GUI Only", "Start the simulation after loading");
    
    oc.doRegister("no-start", 'N', new Option_Bool(false));
    oc.addDescription("no-start", "GUI Only", "Does not start the simulation after loading");

    oc.doRegister("disable-textures", 'T', new Option_Bool(false));
    oc.addDescription("disable-textures", "GUI Only", "Do not load background pictures");
    
    // finally, add HIB options
    // the reason that we include them in vanilla sumo as well is to make reusing config files easy
    
    oc.addOptionSubTopic("HI-iberia");
    
    /* This is currently unused, but its references are not purged */
    oc.doRegister("ecoGemRouting", 'A', new Option_Bool(false));
    oc.addDescription("ecoGemRouting", "HI-iberia", "Forces the EcoGem routing mode");
    
    /* This is currently unused, but its references are not purged */
    oc.doRegister("nonEcoGemRouting", 'B', new Option_Bool(false));
    oc.addDescription("nonEcoGemRouting", "HI-iberia", "Forces the non EcoGem routing mode");
    
    oc.doRegister("ARFTmode", new Option_String());
    oc.addDescription("ARFTmode", "HI-iberia", "Activates the Augmented Reality"
      " for Field Testing mode");
    /* following the arguments:
     * <vehicleId>:<adasRp>:<webServiceUrl>
     * {,<vehicleId>:<adasRp>:<webServiceUrl>}
     */
    
    /*
    oc.doRegister("ARFT+mode", new Option_String());
    oc.addDescription("ARFT+mode", "HI-iberia", "Activates the Augmented Reality"
      " for Field Testing Plus mode");
    */
    /* following the arguments:
     * <vehicleId>:<adasRp>:<webServiceUrl>
     * {,<vehicleId>:<adasRp>:<webServiceUrl>}
     */    
    
    oc.doRegister("RTSDGmode", new Option_String());
    oc.addDescription("RTSDGmode", "HI-iberia", "Activates the Real Time Synthe"
      "tic Data Generation mode");
    /* following the arguments:
     * <webServiceUrl>,<vehicleId>:<adasRp>
     * {,<vehicleId>:<adasRp>}
     */
    
    oc.doRegister("RTSDG+mode", new Option_String());
    oc.addDescription("RTSDG+mode", "HI-iberia", "Activates the Real Time Synthe"
      "tic Data Generation Plus mode");
    /* following the arguments:
     * <vehicleId>:<adasRp>:<webServiceUrl>
     * {,<vehicleId>:<adasRp>:<webServiceUrl>}
     */
    
    /* force initial envtemp */
    oc.doRegister("fiet",new Option_Float(20.0));
    oc.addDescription("fiet", "HI-iberia", "Force an initial environment temperature");
    
    /* force initial envhum */
    oc.doRegister("fieh",new Option_Float(60.0));
    oc.addDescription("fieh", "HI-iberia", "Force an initial environment humidity");
    
    /* randomize weather conditions */
    oc.doRegister("rwc",new Option_Bool(false));
    oc.addDescription("rwc", "HI-iberia", "Randomize weather conditions");
    
    /* reallife weather conditions */
    oc.doRegister("rlwc",new Option_Bool(false));
    oc.addDescription("rlwc", "HI-iberia", "Simulate real life weather conditions");
    
    /* randomize auxiliary services */
    oc.doRegister("ras",new Option_Bool(false));
    oc.addDescription("ras", "HI-iberia", "Randomize auxiliary services");
    
    /* reallife auxiliary services */
    oc.doRegister("rlas",new Option_Bool(false));
    oc.addDescription("rlas", "HI-iberia", "Simulate real life auxiliary services");
    
    /* force Germany customised simulation */
    oc.doRegister("ger",new Option_Bool(false));
    oc.addDescription("ger", "HI-iberia", "Simulate real life characteristics in Germany");
    
    /* force Italy customised simulation */
    oc.doRegister("ita",new Option_Bool(false));
    oc.addDescription("ita", "HI-iberia", "Simulate real life characteristics in Italy");
    
    /* Ported to SystemFrame.cpp
    oc.doRegister("verbosity", 'L', new Option_Integer(0));
    oc.addDescription("verbosity", "HI-Iberia", "Adjust the verbosity level of SUMO HIB debug messages");
    */
}

void MSFrame::buildStreams()
{
  // standard outputs
  OutputDevice::createDeviceByOption("netstate-dump-fev", "ecogem-synthetic-data");
  OutputDevice::createDeviceByOption("netstate-dump", "sumo-netstate");
  OutputDevice::createDeviceByOption("summary-output", "summary");
  OutputDevice::createDeviceByOption("tripinfo-output", "tripinfos");
  OutputDevice::createDeviceByOption("tripinfo-fev", "tripinfofev");
  MSDevice_Vehroutes::init();
}

bool MSFrame::checkOptions() {
    OptionsCont& oc = OptionsCont::getOptions();
    bool ok = true;
    if (!oc.isSet("net-file")) {
        WRITE_ERROR("No network file (-n) specified.");
        ok = false;
    }
    if (oc.isSet("incremental-dua-step") && oc.isSet("incremental-dua-base")) {
        WRITE_WARNING("The options 'incremental-dua-step' and 'incremental-dua-base' are deprecated, use 'scale' instead.");
        if (oc.getInt("incremental-dua-step") > oc.getInt("incremental-dua-base")) {
            WRITE_ERROR("Invalid dua step.");
            ok = false;
        }
    }
    if (!oc.isDefault("scale")) {
        if (oc.getFloat("scale") < 0. || oc.getFloat("scale") > 1.) {
            WRITE_ERROR("Invalid scaling factor.");
            ok = false;
        }
    }
    if (oc.getBool("vehroute-output.exit-times") && !oc.isSet("vehroute-output")) {
        WRITE_ERROR("A vehroute-output file is needed for exit times.");
        ok = false;
    }
    return ok;
}

void MSFrame::setMSGlobals(OptionsCont& oc) {
    // pre-initialise the network
    // set whether empty edges shall be printed on dump
    MSGlobals::gOmitEmptyEdgesOnDump = !oc.getBool("netstate-dump.empty-edges");
#ifdef HAVE_INTERNAL_LANES
    // set whether internal lanes shall be used
    MSGlobals::gUsingInternalLanes = !oc.getBool("no-internal-links");
#else
    MSGlobals::gUsingInternalLanes = false;
#endif
    // set the grid lock time
    MSGlobals::gTimeToGridlock = string2time(oc.getString("time-to-teleport")) < 0 ? 0 : string2time(oc.getString("time-to-teleport"));
    MSGlobals::gCheck4Accidents = !oc.getBool("ignore-accidents");
    MSGlobals::gCheckRoutes = !oc.getBool("ignore-route-errors");
#ifdef HAVE_MESOSIM
    MSGlobals::gStateLoaded = oc.isSet("load-state");
    MSGlobals::gUseMesoSim = oc.getBool("mesosim");
#endif

#ifdef HAVE_SUBSECOND_TIMESTEPS
    DELTA_T = string2time(oc.getString("step-length"));
#endif
}