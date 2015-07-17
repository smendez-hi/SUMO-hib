/****************************************************************************/
/// @file    GUITexturesHelper.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Sept 2006
/// @version $Id: GUITexturesHelper.h 11671 2012-01-07 20:14:30Z behrisch $
///
// Global storage for textures; manages and draws them
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
#ifndef GUITexturesHelper_h
#define GUITexturesHelper_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <fx.h>


// ===========================================================================
// variable declarations
// ===========================================================================
/// determines whether the textures shall be displayed within the gui
extern bool gAllowTextures;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GUITexturesHelper
 * @brief Global storage for textures; manages and draws them
 */
class GUITexturesHelper {
public:
    /// Draws a named texture as a box with the given size
    static void drawTexturedBox(unsigned int which, SUMOReal size);

    /// Draws a named texture as a rectangle with the given sizes
    static void drawTexturedBox(unsigned int which,
                                SUMOReal sizeX1, SUMOReal sizeY1, SUMOReal sizeX2, SUMOReal sizeY2);

    /// Adds a texture to use
    static unsigned int add(FXImage* i);

};


#endif

/****************************************************************************/
