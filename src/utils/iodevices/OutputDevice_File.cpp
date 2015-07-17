/****************************************************************************/
/// @file    OutputDevice_File.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    2004
/// @version $Id: OutputDevice_File.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// An output device that encapsulates an ofstream
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
#include "OutputDevice_File.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
OutputDevice_File::OutputDevice_File(const std::string& fullName)
    : myFileStream(0) {
#ifdef WIN32
    if (fullName == "/dev/null") {
        myFileStream = new std::ofstream("NUL");
#else
    if (fullName == "nul" || fullName == "NUL") {
        myFileStream = new std::ofstream("/dev/null");
#endif
    } else {
        myFileStream = new std::ofstream(fullName.c_str());
    }
    if (!myFileStream->good()) {
        delete myFileStream;
        throw IOError("Could not build output file '" + fullName + "'.");
    }
}


OutputDevice_File::~OutputDevice_File() {
    myFileStream->close();
    delete myFileStream;
}


std::ostream&
OutputDevice_File::getOStream() {
    return *myFileStream;
}


/****************************************************************************/

