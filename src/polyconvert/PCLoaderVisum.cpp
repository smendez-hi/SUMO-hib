/****************************************************************************/
/// @file    PCLoaderVisum.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Christoph Sommer
/// @author  Michael Behrisch
/// @date    Thu, 02.11.2006
/// @version $Id: PCLoaderVisum.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// A reader of pois and polygons stored in VISUM-format
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

#include <string>
#include <map>
#include <fstream>
#include <utils/common/StringTokenizer.h>
#include <utils/common/UtilExceptions.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/StringUtils.h>
#include <utils/common/TplConvert.h>
#include <utils/common/ToString.h>
#include <utils/common/FileHelpers.h>
#include <utils/options/OptionsCont.h>
#include <utils/options/Option.h>
#include <utils/importio/LineReader.h>
#include <utils/common/StdDefs.h>
#include <polyconvert/PCPolyContainer.h>
#include "PCLoaderVisum.h"
#include <utils/common/RGBColor.h>
#include <utils/geom/GeomHelper.h>
#include <utils/geom/Boundary.h>
#include <utils/geom/Position.h>
#include <utils/geom/GeoConvHelper.h>
#include <utils/importio/NamedColumnsParser.h>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
void
PCLoaderVisum::loadIfSet(OptionsCont& oc, PCPolyContainer& toFill,
                         PCTypeMap& tm) {
    if (!oc.isSet("visum-files")) {
        return;
    }
    // parse file(s)
    std::vector<std::string> files = oc.getStringVector("visum-files");
    for (std::vector<std::string>::const_iterator file = files.begin(); file != files.end(); ++file) {
        if (!FileHelpers::exists(*file)) {
            throw ProcessError("Could not open visum-file '" + *file + "'.");
        }
        PROGRESS_BEGIN_MESSAGE("Parsing from visum-file '" + *file + "'");
        load(*file, oc, toFill, tm);
        PROGRESS_DONE_MESSAGE();
    }
}



void
PCLoaderVisum::load(const std::string& file, OptionsCont& oc, PCPolyContainer& toFill,
                    PCTypeMap& tm) {
    GeoConvHelper& geoConvHelper = GeoConvHelper::getProcessing();
    std::string what;
    std::map<long, Position> punkte;
    std::map<long, PositionVector> kanten;
    std::map<long, PositionVector> teilflaechen;
    std::map<long, long> flaechenelemente;
    NamedColumnsParser lineParser;
    LineReader lr(file);
    while (lr.hasMore()) {
        std::string line = lr.readLine();
        // reset if current is over
        if (line.length() == 0 || line[0] == '*' || line[0] == '$') {
            what = "";
        }
        // read items
        if (what == "$PUNKT") {
            lineParser.parseLine(line);
            long id = TplConvert<char>::_2long(lineParser.get("ID").c_str());
            SUMOReal x = TplConvert<char>::_2SUMOReal(lineParser.get("XKOORD").c_str());
            SUMOReal y = TplConvert<char>::_2SUMOReal(lineParser.get("YKOORD").c_str());
            Position pos(x, y);
            if (!geoConvHelper.x2cartesian(pos)) {
                WRITE_WARNING("Unable to project coordinates for point '" + toString(id) + "'.");
            }
            punkte[id] = pos;
            continue;
        } else if (what == "$KANTE") {
            lineParser.parseLine(line);
            long id = TplConvert<char>::_2long(lineParser.get("ID").c_str());
            long fromID = TplConvert<char>::_2long(lineParser.get("VONPUNKTID").c_str());
            long toID = TplConvert<char>::_2long(lineParser.get("NACHPUNKTID").c_str());
            PositionVector vec;
            vec.push_back(punkte[fromID]);
            vec.push_back(punkte[toID]);
            kanten[id] = vec;
            continue;
        } else if (what == "$ZWISCHENPUNKT") {
            lineParser.parseLine(line);
            long id = TplConvert<char>::_2long(lineParser.get("KANTEID").c_str());
            int index = TplConvert<char>::_2int(lineParser.get("INDEX").c_str());
            SUMOReal x = TplConvert<char>::_2SUMOReal(lineParser.get("XKOORD").c_str());
            SUMOReal y = TplConvert<char>::_2SUMOReal(lineParser.get("YKOORD").c_str());
            Position pos(x, y);
            if (!geoConvHelper.x2cartesian(pos)) {
                WRITE_WARNING("Unable to project coordinates for edge '" + toString(id) + "'.");
            }
            kanten[id].insertAt(index, pos);
            continue;
        } else if (what == "$TEILFLAECHENELEMENT") {
            lineParser.parseLine(line);
            long id = TplConvert<char>::_2long(lineParser.get("TFLAECHEID").c_str());
            int index = TplConvert<char>::_2int(lineParser.get("INDEX").c_str());
            index = 0; /// hmmmm - assume it's sorted...
            long kid = TplConvert<char>::_2long(lineParser.get("KANTEID").c_str());
            int dir = TplConvert<char>::_2int(lineParser.get("RICHTUNG").c_str());
            if (teilflaechen.find(id) == teilflaechen.end()) {
                teilflaechen[id] = PositionVector();
            }
            if (dir == 0) {
                for (int i = 0; i < (int) kanten[kid].size(); ++i) {
                    teilflaechen[id].push_back_noDoublePos(kanten[kid][i]);
                }
            } else {
                for (int i = (int) kanten[kid].size() - 1; i >= 0; --i) {
                    teilflaechen[id].push_back_noDoublePos(kanten[kid][i]);
                }
            }
            continue;
        } else if (what == "$FLAECHENELEMENT") {
            lineParser.parseLine(line);
            long id = TplConvert<char>::_2long(lineParser.get("FLAECHEID").c_str());
            long tid = TplConvert<char>::_2long(lineParser.get("TFLAECHEID").c_str());
            int enklave = TplConvert<char>::_2int(lineParser.get("ENKLAVE").c_str()); // !!! unused
            enklave = 0;
            flaechenelemente[id] = tid;
            continue;
        }
        // set if read
        if (line[0] == '$') {
            what = "";
            if (line.find("$PUNKT") == 0) {
                what = "$PUNKT";
            } else if (line.find("$KANTE") == 0) {
                what = "$KANTE";
            } else if (line.find("$ZWISCHENPUNKT") == 0) {
                what = "$ZWISCHENPUNKT";
            } else if (line.find("$TEILFLAECHENELEMENT") == 0) {
                what = "$TEILFLAECHENELEMENT";
            } else if (line.find("$FLAECHENELEMENT") == 0) {
                what = "$FLAECHENELEMENT";
            }
            if (what != "") {
                lineParser.reinit(line.substr(what.length() + 1));
            }
        }
    }

    // do some more sane job...
    RGBColor c = RGBColor::parseColor(oc.getString("color"));
    std::map<std::string, std::string> typemap;
    // load the pois/polys
    lr.reinit();
    bool parsingCategories = false;
    bool parsingPOIs = false;
    bool parsingDistrictsDirectly = false;
    PositionVector vec;
    std::string polyType, lastID;
    bool first = true;
    while (lr.hasMore()) {
        std::string line = lr.readLine();
        // do not parse empty lines
        if (line.length() == 0) {
            continue;
        }
        // do not parse comment lines
        if (line[0] == '*') {
            continue;
        }

        if (line[0] == '$') {
            // reset parsing on new entry type
            parsingCategories = false;
            parsingPOIs = false;
            parsingDistrictsDirectly = false;
            polyType = "";
        }

        if (parsingCategories) {
            // parse the category
            StringTokenizer st(line, ";");
            std::string catid = st.next();
            std::string catname = st.next();
            typemap[catid] = catname;
        }
        if (parsingPOIs) {
            // parse the poi
            // $POI:Nr;CATID;CODE;NAME;Kommentar;XKoord;YKoord;
            StringTokenizer st(line, ";");
            std::string num = st.next();
            std::string catid = st.next();
            std::string code = st.next();
            std::string name = st.next();
            std::string comment = st.next();
            std::string xpos = st.next();
            std::string ypos = st.next();
            // process read values
            SUMOReal x = TplConvert<char>::_2SUMOReal(xpos.c_str());
            SUMOReal y = TplConvert<char>::_2SUMOReal(ypos.c_str());
            Position pos(x, y);
            if (!geoConvHelper.x2cartesian(pos)) {
                WRITE_WARNING("Unable to project coordinates for POI '" + num + "'.");
            }
            std::string type = typemap[catid];
            // check the poi
            name = num;
            // patch the values
            bool discard = oc.getBool("discard");
            int layer = oc.getInt("layer");
            RGBColor color;
            if (tm.has(type)) {
                const PCTypeMap::TypeDef& def = tm.get(type);
                name = def.prefix + name;
                type = def.id;
                color = RGBColor::parseColor(def.color);
                discard = def.discard;
                layer = def.layer;
            } else {
                name = oc.getString("prefix") + name;
                type = oc.getString("type");
                color = c;
            }
            if (!discard) {
                PointOfInterest* poi = new PointOfInterest(name, type, pos, color);
                if (!toFill.insert(name, poi, layer)) {
                    WRITE_ERROR("POI '" + name + "' could not been added.");
                    delete poi;
                }
            }
        }

        // poly
        if (polyType != "") {
            StringTokenizer st(line, ";");
            std::string id = st.next();
            std::string type;
            if (!first && lastID != id) {
                // we have parsed a polygon completely
                RGBColor color;
                int layer = oc.getInt("layer");
                bool discard = oc.getBool("discard");
                if (tm.has(polyType)) {
                    const PCTypeMap::TypeDef& def = tm.get(polyType);
                    id = def.prefix + id;
                    type = def.id;
                    color = RGBColor::parseColor(def.color);
                    discard = def.discard;
                    layer = def.layer;
                } else {
                    id = oc.getString("prefix") + id;
                    type = oc.getString("type");
                    color = c;
                }
                if (!discard) {
                    Polygon* poly = new Polygon(id, type, color, vec, false);
                    if (!toFill.insert(id, poly, 1)) {
                        WRITE_ERROR("Polygon '" + id + "' could not been added.");
                        delete poly;
                    }
                }
                vec.clear();
            }
            lastID = id;
            first = false;
            // parse current poly
            std::string index = st.next();
            std::string xpos = st.next();
            std::string ypos = st.next();
            Position pos2D((SUMOReal) atof(xpos.c_str()), (SUMOReal) atof(ypos.c_str()));
            if (!geoConvHelper.x2cartesian(pos2D)) {
                WRITE_WARNING("Unable to project coordinates for polygon '" + id + "'.");
            }
            vec.push_back(pos2D);
        }

        // district refering a shape
        if (parsingDistrictsDirectly) {
            //$BEZIRK:NR	CODE	NAME	TYPNR	XKOORD	YKOORD	FLAECHEID	BEZART	IVANTEIL_Q	IVANTEIL_Z	OEVANTEIL	METHODEANBANTEILE	ZWERT1	ZWERT2	ZWERT3	ISTINAUSWAHL	OBEZNR	NOM_COM	COD_COM
            StringTokenizer st(line, ";");
            std::string num = st.next();
            std::string code = st.next();
            std::string name = st.next();
            st.next(); // typntr
            std::string xpos = st.next();
            std::string ypos = st.next();
            long id = TplConvert<char>::_2long(st.next().c_str());
            // patch the values
            std::string type = "district";
            name = num;
            bool discard = oc.getBool("discard");
            int layer = oc.getInt("layer");
            RGBColor color;
            if (tm.has(type)) {
                const PCTypeMap::TypeDef& def = tm.get(type);
                name = def.prefix + name;
                type = def.id;
                color = RGBColor::parseColor(def.color);
                discard = def.discard;
                layer = def.layer;
            } else {
                name = oc.getString("prefix") + name;
                type = oc.getString("type");
                color = c;
            }
            if (!discard) {
                if (teilflaechen[flaechenelemente[id]].size() > 0) {
                    Polygon* poly = new Polygon(name, type, color, teilflaechen[flaechenelemente[id]], false);
                    if (!toFill.insert(name, poly, layer)) {
                        WRITE_ERROR("Polygon '" + name + "' could not been added.");
                        delete poly;
                    }
                } else {
                    SUMOReal x = TplConvert<char>::_2SUMOReal(xpos.c_str());
                    SUMOReal y = TplConvert<char>::_2SUMOReal(ypos.c_str());
                    Position pos(x, y);
                    if (!geoConvHelper.x2cartesian(pos)) {
                        WRITE_WARNING("Unable to project coordinates for POI '" + name + "'.");
                    }
                    PointOfInterest* poi = new PointOfInterest(name, type, pos, color);
                    if (!toFill.insert(name, poi, layer)) {
                        WRITE_ERROR("POI '" + name + "' could not been added.");
                        delete poi;
                    }
                }
            }
        }


        if (line.find("$POIKATEGORIEDEF:") == 0 || line.find("$POIKATEGORIE:") == 0) {
            // ok, got categories, begin parsing from next line
            parsingCategories = true;
        }
        if (line.find("$POI:") == 0) {
            // ok, got pois, begin parsing from next line
            parsingPOIs = true;
        }
        if (line.find("$BEZIRK") == 0 && line.find("FLAECHEID") != std::string::npos) {
            // ok, have a district header, and it seems like districts would reference shapes...
            parsingDistrictsDirectly = true;
        }


        if (line.find("$BEZIRKPOLY") != std::string::npos) {
            polyType = "district";
        }
        if (line.find("$GEBIETPOLY") != std::string::npos) {
            polyType = "area";
        }

    }
}



/****************************************************************************/

