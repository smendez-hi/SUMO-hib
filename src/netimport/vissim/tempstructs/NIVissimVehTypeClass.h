/****************************************************************************/
/// @file    NIVissimVehTypeClass.h
/// @author  Daniel Krajzewicz
/// @date    Sept 2002
/// @version $Id: NIVissimVehTypeClass.h 11671 2012-01-07 20:14:30Z behrisch $
///
// -------------------
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
#ifndef NIVissimVehTypeClass_h
#define NIVissimVehTypeClass_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <map>
#include <utils/common/RGBColor.h>
#include <utils/common/VectorHelper.h>


// ===========================================================================
// class definitions
// ===========================================================================
/**
 *
 */
class NIVissimVehTypeClass {
public:
    NIVissimVehTypeClass(int id, const std::string& name,
                         const RGBColor& color, IntVector& types);
    ~NIVissimVehTypeClass();
    static bool dictionary(int id, const std::string& name,
                           const RGBColor& color, IntVector& types);
    static bool dictionary(int id, NIVissimVehTypeClass* o);
    static NIVissimVehTypeClass* dictionary(int name);
    static void clearDict();
    SUMOReal meanSpeed() const;
private:
    int myID;
    std::string myName;
    RGBColor myColor;
    IntVector myTypes;
private:
    typedef std::map<int, NIVissimVehTypeClass*> DictType;
    static DictType myDict;
};


#endif

/****************************************************************************/

