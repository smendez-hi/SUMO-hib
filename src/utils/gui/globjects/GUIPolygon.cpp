/****************************************************************************/
/// @file    GUIPolygon.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @author  Laura Bieker
/// @date    June 2006
/// @version $Id: GUIPolygon.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// The GUI-version of a polygon
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
#include "GUIPolygon.h"
#include <utils/gui/globjects/GUIGlObject.h>
#include <utils/gui/div/GUIParameterTableWindow.h>
#include <utils/gui/globjects/GUIGLObjectPopupMenu.h>
#include <utils/gui/settings/GUIVisualizationSettings.h>
#include <utils/gui/div/GLHelper.h>
#include <foreign/polyfonts/polyfonts.h>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS

#ifdef WIN32
#include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>


// ===========================================================================
// method definitions
// ===========================================================================
GUIPolygon::GUIPolygon(int layer,
                       const std::string name, const std::string type,
                       const RGBColor& color,
                       const PositionVector& Pos,
                       bool fill)
    : Polygon(name, type, color, Pos, fill),
      GUIGlObject_AbstractAdd("poly", GLO_SHAPE, name), myLayer(layer) {}


GUIPolygon::~GUIPolygon() {}



GUIGLObjectPopupMenu*
GUIPolygon::getPopUpMenu(GUIMainWindow& app,
                         GUISUMOAbstractView& parent) {
    GUIGLObjectPopupMenu* ret = new GUIGLObjectPopupMenu(app, parent, *this);
    buildPopupHeader(ret, app, false);
    FXString t(myType.c_str());
    new FXMenuCommand(ret, "(" + t + ")", 0, 0, 0);
    new FXMenuSeparator(ret);
    buildCenterPopupEntry(ret);
    buildNameCopyPopupEntry(ret);
    buildSelectionPopupEntry(ret);
    buildPositionCopyEntry(ret, false);
    return ret;
}


GUIParameterTableWindow*
GUIPolygon::getParameterWindow(GUIMainWindow&,
                               GUISUMOAbstractView&) {
    return 0;
}


Boundary
GUIPolygon::getCenteringBoundary() const {
    Boundary b;
    b.add(myShape.getBoxBoundary());
    b.grow(10);
    return b;
}


void APIENTRY beginCallback(GLenum which) {
    glBegin(which);
}

void APIENTRY errorCallback(GLenum errorCode) {
    const GLubyte* estring;

    estring = gluErrorString(errorCode);
    fprintf(stderr, "Tessellation Error: %s\n", estring);
    exit(0);
}

void APIENTRY endCallback(void) {
    glEnd();
}

void APIENTRY vertexCallback(GLvoid* vertex) {
    const GLdouble* pointer;

    pointer = (GLdouble*) vertex;
    glVertex3dv((GLdouble*) vertex);
}

void APIENTRY combineCallback(GLdouble coords[3],
                              GLdouble* vertex_data[4],
                              GLfloat weight[4], GLdouble** dataOut) {
    UNUSED_PARAMETER(weight);
    UNUSED_PARAMETER(*vertex_data);
    GLdouble* vertex;

    vertex = (GLdouble*) malloc(7 * sizeof(GLdouble));

    vertex[0] = coords[0];
    vertex[1] = coords[1];
    vertex[2] = coords[2];
    *dataOut = vertex;
}

double glvert[6];
void
GUIPolygon::drawGL(const GUIVisualizationSettings& s) const {
    UNUSED_PARAMETER(s);
    if (fill()) {
        if (myShape.size() < 3) {
            return;
        }
    } else {
        if (myShape.size() < 2) {
            return;
        }
    }
    glPushName(getGlID());
    glPushMatrix();
    glTranslated(0, 0, getLayer());
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    GLHelper::setColor(getColor());
    if (fill()) {
        double* points = new double[myShape.size() * 3];
        GLUtesselator* tobj = gluNewTess();
        gluTessCallback(tobj, GLU_TESS_VERTEX, (GLvoid(APIENTRY*)()) &glVertex3dv);
        gluTessCallback(tobj, GLU_TESS_BEGIN, (GLvoid(APIENTRY*)()) &beginCallback);
        gluTessCallback(tobj, GLU_TESS_END, (GLvoid(APIENTRY*)()) &endCallback);
        //gluTessCallback(tobj, GLU_TESS_ERROR, (GLvoid (APIENTRY*) ()) &errorCallback);
        gluTessCallback(tobj, GLU_TESS_COMBINE, (GLvoid(APIENTRY*)()) &combineCallback);
        gluTessProperty(tobj, GLU_TESS_WINDING_RULE, GLU_TESS_WINDING_ODD);
        gluTessBeginPolygon(tobj, NULL);
        gluTessBeginContour(tobj);
        for (size_t i = 0; i != myShape.size(); ++i) {
            points[3 * i]  = myShape[(int) i].x();
            points[3 * i + 1]  = myShape[(int) i].y();
            points[3 * i + 2]  = 0;
            glvert[0] = myShape[(int) i].x();
            glvert[1] = myShape[(int) i].y();
            glvert[2] = 0;
            glvert[3] = 1;
            glvert[4] = 1;
            glvert[5] = 1;
            gluTessVertex(tobj, points + 3 * i, points + 3 * i) ;
        }
        gluTessEndContour(tobj);

        gluTessEndPolygon(tobj);
        gluDeleteTess(tobj);
        delete[] points;
    } else {
        GLHelper::drawLine(myShape);
        GLHelper::drawBoxLines(myShape, 1.);
    }
    glPopName();
    glPopMatrix();
}



int
GUIPolygon::getLayer() const {
    return myLayer;
}



/****************************************************************************/

