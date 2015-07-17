/****************************************************************************/
/// @file    GUIShapeContainer.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    08.10.2009
/// @version $Id: GUIShapeContainer.h 11671 2012-01-07 20:14:30Z behrisch $
///
// Storage for geometrical objects extended by mutexes
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
#ifndef GUIShapeContainer_h
#define GUIShapeContainer_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <utils/shapes/ShapeContainer.h>
#include <utils/gui/globjects/GUIGlObject.h>
#include <utils/foxtools/MFXMutex.h>


// ===========================================================================
// class declarations
// ===========================================================================
class SUMORTree;
class Position;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GUIShapeContainer
 * @brief Storage for geometrical objects extended by mutexes
 * @see ShapeContainer
 */
class GUIShapeContainer : public ShapeContainer {
public:
    /// @brief Constructor
    GUIShapeContainer(SUMORTree& vis) ;


    /// @brief Destructor
    virtual ~GUIShapeContainer() ;



    /** @brief Builds a PoI using the given values and adds it to the according layer
     * @param[in] name The name of the PoI to add
     * @param[in] layer The layer to add the PoI to
     * @param[in] type The type of the PoI to add
     * @param[in] c The color of the PoI to add
     * @param[in] pos The position of the PoI to add
     * @return Whether the PoI could be added (no other with same id exists in the layer)
     * @see ShapeContainer::addPoI
     */
    virtual bool addPoI(const std::string& name, int layer, const std::string& type,
                        const RGBColor& c, const Position& pos) ;


    /** @brief Builds a polygon using the given values and adds it to the according layer
     * @param[in] name The name of the polygon to add
     * @param[in] layer The layer to add the polygon to
     * @param[in] type The type of the polygon to add
     * @param[in] c The color of the polygon to add
     * @param[in] pos The position of the polygon to add
     * @return Whether the polygon could be added (no other with same id exists in the layer)
     * @see ShapeContainer::addPolygon
     */
    virtual bool addPolygon(const std::string& name, int layer,
                            const std::string& type, const RGBColor& c, bool filled, const PositionVector& shape) ;



    /** @brief Removes a polygon from the container
     * @param[in] layer The layer the polygon is located in
     * @param[in] id The id of the polygon
     * @return Whether the polygon could be removed
     * @see ShapeContainer::removePolygon
     */
    bool removePolygon(int layer, const std::string& id) ;


    /** @brief Removes a PoI from the container
     * @param[in] layer The layer the PoI is located in
     * @param[in] id The id of the PoI
     * @return Whether the poi could be removed
     * @see ShapeContainer::removePoI
     */
    bool removePoI(int layer, const std::string& id) ;



    /** @brief Assigns a new position to the named PoI
     * @param[in] layer The layer the PoI is located in
     * @param[in] id The id of the PoI to move
     * @param[in] pos The PoI's new position
     * @see ShapeContainer::movePoI
     */
    void movePoI(int layer, const std::string& id, const Position& pos) ;


    /** @brief Assigns a shape to the named polygon
     * @param[in] layer The layer the polygon is located in
     * @param[in] id The id of the polygon to reshape
     * @param[in] shape The polygon's new shape
     */
    void reshapePolygon(int layer, const std::string& id, const PositionVector& shape) ;


    /// Returns the gl-ids of all shapes
    std::vector<GUIGlID> getShapeIDs() const;


private:
    /// @brief The mutex for adding/removing operations
    MFXMutex myLock;

    /// @brief The RTree structure to add and remove visualization elements
    SUMORTree& myVis;

};


#endif

/****************************************************************************/

