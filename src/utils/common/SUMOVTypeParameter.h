/****************************************************************************/
/// @file    SUMOVTypeParameter.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    10.09.2009
/// @version $Id: SUMOVTypeParameter.h 11671 2012-01-07 20:14:30Z behrisch $
///
// Structure representing possible vehicle parameter
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
#ifndef SUMOVTypeParameter_h
#define SUMOVTypeParameter_h

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
#include "SUMOVehicleClass.h"
#include "RGBColor.h"
#include "SUMOTime.h"

// ===========================================================================
// class declarations
// ===========================================================================
class OutputDevice;
class OptionsCont;

// ===========================================================================
// value definitions
// ===========================================================================
const int VTYPEPARS_LENGTH_SET = 1;
const int VTYPEPARS_MINGAP_SET = 2;
const int VTYPEPARS_MAXSPEED_SET = 2 << 2;
const int VTYPEPARS_PROBABILITY_SET = 2 << 3;
const int VTYPEPARS_SPEEDFACTOR_SET = 2 << 4;
const int VTYPEPARS_SPEEDDEVIATION_SET = 2 << 5;
const int VTYPEPARS_EMISSIONCLASS_SET = 2 << 6;
const int VTYPEPARS_COLOR_SET = 2 << 7;
const int VTYPEPARS_VEHICLECLASS_SET = 2 << 8;
const int VTYPEPARS_WIDTH_SET = 2 << 9;
const int VTYPEPARS_SHAPE_SET = 2 << 10;

// ===========================================================================
// struct definitions
// ===========================================================================
/**
 * @class SUMOVTypeParameter
 * @brief Structure representing possible vehicle parameter
 */
class SUMOVTypeParameter
{
  public:
    /** @brief Constructor
     *  @param[in] defVehTypeId The default vehicle type id
     *
     * Initialises the structure with default values
     */
    SUMOVTypeParameter(const std::string defVehTypeId);

    /** @brief Returns whether the given parameter was set
     * @param[in] what The parameter which one asks for
     * @return Whether the given parameter was set
     */
    bool wasSet(int what) const {
        return (setParameter & what) != 0;
    }

    /** @brief Writes the vtype
     *
     * @param[in, out] dev The device to write into
     * @exception IOError not yet implemented
     */
    void write(OutputDevice& dev) const;

    /** @brief Validates stored car-following parameter
     */
    void validateCFParameter() const ;
    
    void setMaxSpeed(SUMOReal ms);

    SUMOReal getMass() const;
    
    void setMass(SUMOReal m);
    
    void setRollingRes(SUMOReal rr);
    
    void setAirDens(SUMOReal ad);
    
    /**
     * Documentation for this:
     * http://www.baranidesign.com/air-density/air-density.htm used for humidity
     * modifier
     * http://en.wikipedia.org/wiki/Density_of_air#Temperature_and_Pressure used
     * for temperature modifier
     *
     * Table used:
     *                hum
     *              1     99
     * t <10      1.288 1.284
     * e >=10 <20 1.247 1.241
     * m >=20 <30 1.204 1.194
     * p >=30     1.164 1.146
     */
    void updateAirDens(SUMOReal et,SUMOReal eh);
    
    void setFrontalArea(SUMOReal fa);
    
    void setDragCoefficient(SUMOReal dc);
    
    void setGearRatio(SUMOReal gr);
        
    void setGearSysEffc(SUMOReal gse);
        
    void setRegenRatio(SUMOReal rr);
        
    void setMotorSysEffc(SUMOReal mse);
        
    void setCopperLosses(SUMOReal cl);
        
    void setIronLosses(SUMOReal il);
        
    void setWindageLosses(SUMOReal wl);
        
    void setConstantLosses(SUMOReal cl);
        
    void setPowerAccs(SUMOReal pa);
    
    void setNumCells(SUMOReal nc);
    
    void updateCapacity(SUMOReal et);
    
    void setCapacity(SUMOReal c);
    
    SUMOReal getCapacity()const;
    
    void setNominalCapacity(SUMOReal nc);
    
    SUMOReal getNominalCapacity()const;

    void setTimeCharge(SUMOReal time_charge);
    
    SUMOReal getTimeCharge()const;
    
    void setCelsius(SUMOReal c);
    
    SUMOReal getCelsius();
        
    void setHumidity(SUMOReal h);
    
    SUMOReal getHumidity();
        
    void setAuxMask(std::string am);
        
    void setLightMask(std::string lm);
    
    void setVehicleHeadlightsConsumption(SUMOReal vhlc);
    
    void setFrontFogLightsConsumption(SUMOReal fflc);
    
    void setCockpitLightsConsumption(SUMOReal clc);
    
    void setRadioConsumption(SUMOReal rc);
    
    void setHeatingACConsumption(SUMOReal hacc);

    /// @brief The vehicle type's id
    std::string id;

    /// @brief The physical vehicle length + the standing gap in m
    SUMOReal length;

    /// @brief This class' free space in front of the vehicle itself
    SUMOReal minGap;

    /// @brief The vehicle type's maximum speed [m/s]
    SUMOReal maxSpeed;

    /// @brief The probability when being added to a distribution without an explicit probability
    SUMOReal defaultProbability;

    /// @brief The factor by which the maximum speed may deviate from the allowed max speed on the street
    SUMOReal speedFactor;

    /// @brief The standard deviation for speed variations
    SUMOReal speedDev;

    /// @brief The emission class of this vehicle
    SUMOEmissionClass emissionClass;

    /// @brief The color
    RGBColor color;

    /// @brief The vehicle's class
    SUMOVehicleClass vehicleClass;

    /// @name Values for drawing this class' vehicles
    /// @{

    // What's the purpose of the '@{' and the '@}' ???

    /// @brief This class' width
    SUMOReal width;

    /// @brief This class' shape
    SUMOVehicleShape shape;

    /// @}

    /// @brief The enum-representation of the car-following model to use
    SumoXMLTag cfModel;

    /// @brief Car-following parameter
    typedef std::map < SumoXMLAttr, SUMOReal > CFParams;
    CFParams cfParameter;

    /// @brief The name of the lane-change model to use
    std::string lcModel;

    /// @brief Information for the router which parameter were set
    int setParameter;

    /// @brief Information whether this type was already saved (needed by routers)
    mutable bool saved;

    /// @brief Information whether this is a type-stub, being only referenced but not defined (needed by routers)
    mutable bool onlyReferenced;

    SUMOReal mass;
    SUMOReal rolling_res;
    SUMOReal air_dens;
    SUMOReal frontal_area;
    SUMOReal drag_coefficient;
    SUMOReal Gear_ratio;
    SUMOReal Gear_sys_effc;
    SUMOReal regen_ratio;
    SUMOReal motor_sys_effc;
    SUMOReal cooper_losses;
    SUMOReal iron_losses;
    SUMOReal windage_losses;
    SUMOReal constant_losses;
    SUMOReal power_accs;
    SUMOReal internal_resistance;
    SUMOReal num_cells;
    SUMOReal capacity;
    SUMOReal nominalCapacity;
    SUMOReal celsius;
    SUMOReal humidity;
    SUMOReal time_charge;
    SUMOReal radio_consumption;
    SUMOReal heatingAC_consumption;
    SUMOReal vehicle_headlights_consumption;
    SUMOReal cockpit_lights_consumption;
    SUMOReal front_fog_lights_consumption;
    std::string aux_mask ;
    std::string light_mask ;
};

#endif