/****************************************************************************/
/// @file    SystemFrame.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Mon, 23.06.2003
/// @version $Id: SystemFrame.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// A set of actions common to all applications
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

#include "SystemFrame.h"
#include <string>
#include <utils/xml/XMLSubSys.h>
#include <utils/common/MsgHandler.h>
#include <utils/options/OptionsCont.h>
#include "RandHelper.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS

// ===========================================================================
// method definitions
// ===========================================================================
void SystemFrame::addConfigurationOptions(OptionsCont& oc)
{
    oc.addOptionSubTopic("Configuration");

    oc.doRegister("configuration-file", 'c', new Option_FileName());
    oc.addSynonyme("configuration-file", "configuration");
    oc.addDescription("configuration-file", "Configuration", "Loads the named config on startup");

    oc.doRegister("save-configuration", new Option_FileName());
    oc.addSynonyme("save-config", "save-configuration");
    oc.addDescription("save-configuration", "Configuration", "Saves current configuration into FILE");

    oc.doRegister("save-template", new Option_FileName());
    oc.addDescription("save-template", "Configuration", "Saves a configuration template (empty) into FILE");

    oc.doRegister("save-schema", new Option_FileName());
    oc.addDescription("save-schema", "Configuration", "Saves the configuration schema into FILE");

    oc.doRegister("save-commented", new Option_Bool(false));
    oc.addSynonyme("save-commented", "save-template.commented");
    oc.addDescription("save-commented", "Configuration", "Adds comments to saved template, configuration, or schema");
}

void SystemFrame::addReportOptions(OptionsCont& oc)
{
    oc.addOptionSubTopic("Report");

    oc.doRegister("verbose", 'v', new Option_Bool(false));
    oc.addDescription("verbose", "Report", "Switches to verbose output");

    oc.doRegister("print-options", 'p', new Option_Bool(false));
    oc.addDescription("print-options", "Report", "Prints option values before processing");

    oc.doRegister("help", '?', new Option_Bool(false));
    oc.addDescription("help", "Report", "Prints this screen");

    oc.doRegister("version", 'V', new Option_Bool(false));
    oc.addDescription("version", "Report", "Prints the current version");

    oc.doRegister("no-warnings", 'W', new Option_Bool(false));
    oc.addSynonyme("no-warnings", "suppress-warnings", true);
    oc.addDescription("no-warnings", "Report", "Disables output of warnings");

    oc.doRegister("log", 'l', new Option_FileName());
    oc.addSynonyme("log", "log-file");
    oc.addDescription("log", "Report", "Writes all messages to FILE");

    oc.doRegister("message-log", new Option_FileName());
    oc.addDescription("message-log", "Report", "Writes all non-error messages to FILE");

    oc.doRegister("error-log", new Option_FileName());
    oc.addDescription("error-log", "Report", "Writes all warnings and errors to FILE");
    
    // DEPRECATED but is maintained because references remain...
    oc.doRegister("verbosity", new Option_Integer(0));
    oc.addDescription("verbosity", "Report" /* FIXME */, "Adjust the verbosity "
    "level of SUMO HIB & auxiliar applications debug messages");
    
    oc.doRegister("loadVerbosity", new Option_Integer(0));
    oc.addDescription("loadVerbosity", "Report" /* FIXME */, "Be more verbose about loading");    
    
    oc.doRegister("buildVerbosity", new Option_Integer(0));
    oc.addDescription("buildVerbosity", "Report" /* FIXME */, "Be more verbose about building");    
    
    oc.doRegister("simulationVerbosity", new Option_Integer(0));
    oc.addDescription("simulationVerbosity", "Report" /* FIXME */, "Be more verbose about simulating");
    
    oc.doRegister("allVerbosities", new Option_Integer(0));
    oc.addDescription("allVerbosities", "Report" /* FIXME */, "Activate all the latter verbosity flags");
        
}

void SystemFrame::close()
{
    // close the xml-subsystem
    XMLSubSys::close();
    // delete build program options
    OptionsCont::getOptions().clear();
    // delete messages
    MsgHandler::cleanupOnEnd();
}