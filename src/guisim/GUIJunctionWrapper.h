/****************************************************************************/
/// @file    GUIJunctionWrapper.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Mon, 1 Jul 2003
/// @version $Id: GUIJunctionWrapper.h 11671 2012-01-07 20:14:30Z behrisch $
///
// Holds geometrical values for a junction
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
#ifndef GUIJunctionWrapper_h
#define GUIJunctionWrapper_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <utility>
#include <utils/geom/PositionVector.h>
#include <utils/gui/globjects/GUIGLObjectPopupMenu.h>
#include <utils/gui/globjects/GUIGlObject.h>


// ===========================================================================
// class declarations
// ===========================================================================
class MSNet;
class MSJunction;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GUIJunctionWrapper
 *
 * As MSJunctions do not have a graphical representation but a complex
 *  inheritance tree, this class is used to encapsulate the geometry of an
 *  abstract junction and to be used as a gl-object.
 *
 * In the case the represented junction's shape is empty, the boundary
 *  is computed using the junction's position to which an offset of 1m to each
 *  side is added.
 */
class GUIJunctionWrapper : public GUIGlObject {
public:
    /** @brief Constructor
     * @param[in, changed] idStorage The storage to retrieve the gl-id from
     * @param[in] junction The represented junction
     */
    GUIJunctionWrapper(MSJunction& junction) ;


    /// @brief Destructor
    virtual ~GUIJunctionWrapper() ;



    /// @name inherited from GUIGlObject
    //@{

    /** @brief Returns an own popup-menu
     *
     * @param[in] app The application needed to build the popup-menu
     * @param[in] parent The parent window needed to build the popup-menu
     * @return The built popup-menu
     * @see GUIGlObject::getPopUpMenu
     */
    GUIGLObjectPopupMenu* getPopUpMenu(GUIMainWindow& app,
                                       GUISUMOAbstractView& parent) ;


    /** @brief Returns an own parameter window
     *
     * @param[in] app The application needed to build the parameter window
     * @param[in] parent The parent window needed to build the parameter window
     * @return The built parameter window
     * @see GUIGlObject::getParameterWindow
     */
    GUIParameterTableWindow* getParameterWindow(
        GUIMainWindow& app, GUISUMOAbstractView& parent) ;


    /** @brief Returns the boundary to which the view shall be centered in order to show the object
     *
     * @return The boundary the object is within
     * @see GUIGlObject::getCenteringBoundary
     */
    Boundary getCenteringBoundary() const ;


    /** @brief Draws the object
     * @param[in] s The settings for the current view (may influence drawing)
     * @see GUIGlObject::drawGL
     */
    void drawGL(const GUIVisualizationSettings& s) const ;
    //@}

    /** @brief Returns the boundary of the junction
     * @return This junction's boundary
     */
    Boundary getBoundary() const {
        return myBoundary;
    }

    /// @brief whether this is an inner junction (a waiting spot for crossing a "real" junction)
    bool isInner() const {
        return myIsInner;
    }


protected:
    /// @brief A reference to the represented junction
    MSJunction& myJunction;

    /// @brief The maximum size (in either x-, or y-dimension) for determining whether to draw or not
    SUMOReal myMaxSize;

    /// @brief The represented junction's boundary
    Boundary myBoundary;


private:
    /// @brief Invalidated copy constructor.
    GUIJunctionWrapper(const GUIJunctionWrapper&);

    /// @brief Invalidated assignment operator.
    GUIJunctionWrapper& operator=(const GUIJunctionWrapper&);

    /// @brief whether this wraps an instance of MSInternalJunction
    bool myIsInner;

};


#endif

/****************************************************************************/
