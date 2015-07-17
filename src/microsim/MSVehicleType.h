/****************************************************************************/
/// @file    MSVehicleType.h
/// @author  Christian Roessel
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Mon, 12 Mar 2001
/// @version $Id: MSVehicleType.h 11671 2012-01-07 20:14:30Z behrisch $
///
// The car-following model and parameter
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
#ifndef MSVehicleType_h
#define MSVehicleType_h

// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <cassert>
#include <map>
#include <string>
#include "MSCFModel.h"
#include <utils/common/SUMOTime.h>
#include <utils/common/StdDefs.h>
#include <utils/common/SUMOVehicleClass.h>
#include <utils/common/RandHelper.h>
#include <utils/common/SUMOVTypeParameter.h>
#include <utils/common/RGBColor.h>

// ===========================================================================
// class declarations
// ===========================================================================
class MSLane;
class BinaryInputDevice;
class MSCFModel;
class SUMOVTypeParameter;

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSVehicleType
 * @brief The car-following model and parameter
 *
 * MSVehicleType stores the parameter of a single vehicle type and methods
 *  that use these for computing the vehicle's car-following behavior
 *
 * It is assumed that within the simulation many vehicles are using the same
 *  vehicle type, quite common is using only one vehicle type for all vehicles.
 *
 * You can think of it like of having a vehicle type for each VW Golf or
 *  Ford Mustang in your simulation while the car instances just refer to it.
 */
class MSVehicleType
{
 public:
  /** @brief Constructor.
   *
   * @param[in] id The vehicle type's id
   * @param[in] length The length of vehicles that of this type
   * @param[in] minGap The free space in front of the vehicles of this class
   * @param[in] maxSpeed The maximum velocity vehicles of this type may drive with
   * @param[in] prob The probability of this vehicle type
   * @param[in] speedFactor The speed factor to scale maximum speed with
   * @param[in] speedDev The speed deviation
   * @param[in] vclass The class vehicles of this type belong to
   * @param[in] emissionClass The emission class vehicles of this type belong to
   * @param[in] guiWidth The width of the vehicles when being drawn
   * @param[in] shape How vehicles of this class shall be drawn
   * @param[in] lcModel Name of the lane-change model to use
   * @param[in] c Color of this vehicle type
   */
  MSVehicleType
   (const std::string& id, double lengthWithGap, double minGap, double maxSpeed,
    double prob, double speedFactor, double speedDev, SUMOVehicleClass vclass,
    SUMOEmissionClass emissionClass, SUMOReal guiWidth, SUMOVehicleShape shape,
    const std::string & lcModel, const RGBColor & c);
    
  /** @brief HIB Constructor
   * 
   * @abstract has the additional parameters for FEVs
   */
  MSVehicleType
   (const std::string & id, SUMOReal lengthWithGap, SUMOReal minGap,
    SUMOReal maxSpeed, SUMOReal prob, SUMOReal speedFactor, SUMOReal speedDev,
    SUMOVehicleClass vclass, SUMOEmissionClass emissionClass,
    SUMOReal guiWidth, SUMOVehicleShape shape, const std::string & lcModel,
    const RGBColor & c, SUMOReal m, SUMOReal r_res, SUMOReal air_r,
    SUMOReal frontal, SUMOReal drag_c, double gear_r, double gear_s_e,
    double reg_rat, double motor_s_e, double coop_l, double iron_l,
    double windage_l, double constant_l, double power_a, double num_c,
    double capac, double time, SUMOReal temp, double hum, std::string aux_m,
    std::string light_m,
    double vehicle_lights, double front_light, double cockpit_lights,
    double radio, SUMOReal heating);
    
  /// @brief Destructor
  virtual ~MSVehicleType();
    
  /// @name Atomar getter for simulation
  //  @{
  /// @{
        
  /** @brief Returns the name of the vehicle type
   * @return This type's id
   */
  const std::string & getID() const {
    return myID;
  }

  /** @brief Get vehicle's length [m]
   * @return The length vehicles of this type have in m
   */
  double getLength() const {
    return myLength;
  }

  /** @brief Get vehicle's length including the minimum gap [m]
   * @return The length vehicles of this type have (including the minimum gap in m
   */
  double getLengthWithGap() const {
    return myLength + myMinGap;
  }

  /** @brief Get the free space in front of vehicles of this class
   * @return The place before the vehicle
   */
  double getMinGap() const {
    return myMinGap;
  }

  /** @brief Returns the vehicle type's car following model definition (const version)
   * @return The vehicle type's car following model definition
   */
  inline const MSCFModel & getCarFollowModel() const {
    return *myCarFollowModel;
  }

  /** @brief Returns the vehicle type's car following model definition (non-const version)
   * @return The vehicle type's car following model definition
   */
  inline MSCFModel & getCarFollowModel() {
    return *myCarFollowModel;
  }

  /** @brief Get vehicle's maximum speed [m/s].
   * @return The maximum speed (in m/s) of vehicles of this class
   */
  double getMaxSpeed() const {
    return myMaxSpeed;
  }

  /** @brief Get vehicle's maximum speed [m/s].
   * @return The maximum speed (in m/s) of vehicles of this class
   */
  double hasSpeedDeviation() const {
    return mySpeedDev != 0.0 || mySpeedFactor != 1.0;
  }

  /** @brief Get vehicle's maximum speed [m/s] drawn from a normal distribution.
   *
   * The speed is calculated relative to the reference speed (which is usually
   *  the maximum allowed speed on a lane or edge).
   * @return The maximum speed (in m/s) of vehicles of this class
   */
  double getMaxSpeedWithDeviation(double referenceSpeed) const {
    SUMOReal meanSpeed = mySpeedFactor * referenceSpeed;
    SUMOReal speedDev = mySpeedDev * meanSpeed;
    SUMOReal speed = MIN3(RandHelper::randNorm(meanSpeed, speedDev),
        meanSpeed + 2 * speedDev, myMaxSpeed);
    return MAX3((SUMOReal) 0.0, speed, meanSpeed - 2 * speedDev);
  }

  /** @brief Get the default probability of this vehicle type
   * @return The probability to use this type
   */
  double getDefaultProbability() const {
    return myDefaultProbability;
  }

  /** @brief Get this vehicle type's vehicle class
   *  @return The class of this vehicle type
   *  @see SUMOVehicleClass
   */
  SUMOVehicleClass getVehicleClass() const { return myVehicleClass; }

  /** @brief Get this vehicle type's emission class
   * @return The emission class of this vehicle type
   * @see SUMOEmissionClass
   */
  SUMOEmissionClass getEmissionClass() const {
    return myEmissionClass;
  }

  /** @brief Returns this type's color
   * @return The color of this type
   */
  const RGBColor & getColor() const {
    return myColor;
  }

  /** @brief Returns this type's speed factor
   * @return The speed factor of this type
   */
  double getSpeedFactor() const {
    return mySpeedFactor;
  }

  /** @brief Returns this type's speed deviation
   * @return The speed deviation of this type
   double rolling_res;
   double air_dens;
   double frontal_area;
   double drag_coefficient;
   double Gear_ratio;
   double Gear_sys_effc;
   double regen_ratio;
   double motor_sys_effc;
   double cooper_losses;
   double iron_losses;
   double windage_losses;
   double constant_losses;
   double power_accs;
   double internal_resis;
   double num_cells;
   double capacity;
   */
  double getSpeedDeviation() const {
    return mySpeedDev;
  }

  /// @}
  //  @}

  /// @name Atomar getter for visualization
  /// @{
  /** @brief Get the width which vehicles of this class shall have when being drawn
   * @return The width of this type's vehicles
   */
  double getGuiWidth() const {
    return myWidth;
  }

  /** @brief Get this vehicle type's shape
   * @return The shape of this vehicle type
   * @see SUMOVehicleShape
   */
  SUMOVehicleShape getGuiShape() const {
    return myShape;
  }

  /// @}
  /// Saves the states of a vehicle
  void saveState(std::ostream & os);
  /// @name Setter methods
  /// @{
  /** @brief Set a new value for this type's length
   *
   * If the given value<0 then the one from the original type will
   *  be used.
   *
   * @param[in] length The new length of this type
   */
  void setLength(const double & length);
  /** @brief Set a new value for this type's minimum gap
   *
   * If the given value<0 then the one from the original type will
   *  be used.
   *
   * @param[in] offset The new minimum gap of this type
   */
  void setMinGap(const double & minGap);
  /** @brief Set a new value for this type's maximum speed
   *
   * If the given value<0 then the one from the original type will
   *  be used.
   *
   * @param[in] maxSpeed The new maximum speed of this type
   */
  void setMaxSpeed(const double & maxSpeed);
  /** @brief Set a new value for this type's vehicle class
   * @param[in] vclass The new vehicle class of this type
   */
  void setVClass(SUMOVehicleClass vclass);
  /** @brief Set a new value for this type's default probability
   *
   * If the given value<0 then the one from the original type will
   *  be used.
   *
   * @param[in] prob The new default probability of this type
   */
  void setDefaultProbability(const double & prob);
  /** @brief Set a new value for this type's speed factor
   *
   * If the given value<0 then the one from the original type will
   *  be used.
   *
   * @param[in] factor The new speed factor of this type
   */
  void setSpeedFactor(const double & factor);
  /** @brief Set a new value for this type's speed deviation
   *
   * If the given value<0 then the one from the original type will
   *  be used.
   *
   * @param[in] dev The new speed deviation of this type
   */
  void setSpeedDeviation(const double & dev);
  /** @brief Set a new value for this type's emission class
   * @param[in] eclass The new emission class of this type
   */
  void setEmissionClass(SUMOEmissionClass eclass);
  /** @brief Set a new value for this type's color
   * @param[in] color The new color of this type
   */
  void setColor(const RGBColor & color);
  /** @brief Set a new value for this type's width
   *
   * If the given value<0 then the one from the original type will
   *  be used.
   *
   * @param[in] width The new width of this type
   */
  void setWidth(const double & width);
  /** @brief Set a new value for this type's shape
   * @param[in] shape The new shape of this type
   */
  void setShape(SUMOVehicleShape shape);
  /// @}
    
  /// @name Static methods for building vehicle types
  //  @{
  /// @{
        
  /** @brief Builds the microsim vehicle type described by the given parameter
   * @param[in] from The vehicle type description
   * @return The built vehicle type
   * @exception ProcessError on false values (not et used)
   */
  static MSVehicleType *build(SUMOVTypeParameter & from);
    
  /** @brief Duplicates the microsim vehicle type giving it a the given id
   * @param[in] id The new id of the type
   * @param[in] from The vehicle type
   * @return The built vehicle type
   */
  static MSVehicleType *build
   (const std::string & id, const MSVehicleType *from);
     
  /// @}
  //  @}
    
    
  /** @brief Returns the named value from the map, or the default if it is ot contained there
   * @param[in] from The map to retrieve values from
   * @param[in] attr The corresponding xml attribute
   * @param[in] defaultValue The value to return if the given map does not contain the named variable
   * @return The named value from the map or the default if it does not exist there
   */
  static double get(const SUMOVTypeParameter::CFParams & from,
      SumoXMLAttr attr, double defaultValue);
    
  void setAirDens(double ad);
  
  /**
   * Documentation for this:
   * http://www.baranidesign.com/air-density/air-density.htm used for humidity
   * modifier
   * http://en.wikipedia.org/wiki/Density_of_air#Temperature_and_Pressure used
   * for temperature modifier
   */
  void updateAirDens(SUMOReal envtemp,SUMOReal envhum/*, MSVehicle msv*/);
  
  double getAirDens() const;
  //void setAirDens(SUMOReal ad) { air_dens = ad; }
  
  void updateCapacity(SUMOReal envtemp/*, MSVehicle msv*/);
  void setCapacity(SUMOReal c);
  SUMOReal getCapacity()const;
  void setNominalCapacity(SUMOReal nc);
  SUMOReal getNominalCapacity()const;
  
  void setConstantLosses(double constant_losses);
  double getConstantLosses() const;
    
  double getCooperLosses() const;
  double getDragCoefficient() const;
  double getFrontalArea() const;
  double getGearRatio() const;
  double getGearSysEffc() const;
  double getInternalResis() const;
  double getIronLosses() const;
  double getMass() const;
  double getMotorSysEffc() const;
  double getNumCells() const;
  double getPowerAccs() const;
  double getRegenRatio() const;
  double getRollingRes() const;
  double getWindageLosses() const;
  void setCooperLosses(double cooper_losses);
  void setDragCoefficient(double drag_coefficient);
  void setFrontalArea(double frontal_area);
  void setGearRatio(double Gear_ratio);
  void setGearSysEffc(double Gear_sys_effc);
  void setInternalResis(double internal_resis);
  void setIronLosses(double iron_losses);
  void setMass(double mass);
  void setMotorSysEffc(double motor_sys_effc);
  void setNumCells(double num_cells);
  void setPowerAccs(double power_accs);
  void setRegenRatio(double regen_ratio);
  void setRollingRes(double rolling_res);
  void setWindageLosses(double windage_losses);
    
  std::string getAuxMask() const;
  double getCelsius() const;
  double getHumidity() const;
  std::string getLightMask() const;
  void setAuxMask(std::string aux_mask);
  void setCelsius(double celsius);
  void setHumidity(double humidity);
  void setLightMask(std::string light_mask);
    
  double getTimeCharge() const;
  void setTimeCharge(double time_charge);
    
  double getCockpitLightsConsumption() const;
  double getFrontFogLightsConsumption() const;
  double getHeatingAcConsumption() const;
  double getRadioConsumption() const;
  double getVehicleHeadligthsConsumption() const;
  void setCockpitLigthsConsumption(double cockpit_ligths_consumption);
  void setFrontFogLigthsConsumption(double front_fog_ligths_consumption);
  void setHeatingAcConsumption(double heatingAC_consumption);
  void setRadioConsumption(double radio_consumption);
  void setVehicleHeadligthsConsumption(double vehicle_headligths_consumption__);
  
  /** @brief Returns whether this type belongs to a single vehicle only (was modified)
   * @return Whether this vehicle type is based on a differen one, and belongs to one vehicle only
   */
  bool amVehicleSpecific() const {
    return myOriginalType != 0;
  }

private:
  /// @brief Unique ID
  std::string myID;
  /// @brief Vehicles' length [m]
  double myLength;
  /// @brief This class' free space in front of the vehicle itself
  double myMinGap;
  /// @brief Vehicles' maximum speed [m/s]
  double myMaxSpeed;
  /// @brief The probability when being added to a distribution without an explicit probability
  double myDefaultProbability;
  /// @brief The factor by which the maximum speed may deviate from the allowed max speed on the street
  double mySpeedFactor;
  /// @brief The standard deviation for speed variations
  double mySpeedDev;
  /// @brief ID of the car following model.
  MSCFModel *myCarFollowModel;
  /// @brief ID of the lane change model.
  std::string myLaneChangeModel;
  /// @brief The emission class of such vehicles
  SUMOEmissionClass myEmissionClass;
  /// @brief The color
  RGBColor myColor;
  /// @brief The vehicles' class
  SUMOVehicleClass myVehicleClass;
    
  /// @name Values for drawing this class' vehicles
  /// @{
  /// @brief This class' width
  double myWidth;

  /// @brief This class' shape
  SUMOVehicleShape myShape;
  /// @}

  /// @brief The original type
  const MSVehicleType* myOriginalType;

  double mass;
  double rolling_res;
  double air_dens;
  double frontal_area;
  double drag_coefficient;
  double Gear_ratio;
  double Gear_sys_effc;
  double regen_ratio;
  double motor_sys_effc;
  double cooper_losses;
  double iron_losses;
  double windage_losses;
  double constant_losses;
  double power_accs;
  double internal_resis;
  double num_cells;
  SUMOReal capacity;
  SUMOReal nominalCapacity;
  double celsius;
  double time_charge;
  double humidity;
  double vehicle_headlights_consumption;
  double cockpit_lights_consumption;
  double front_fog_lights_consumption;
  double radio_consumption;
  double heatingAC_consumption;
  std::string aux_mask;
  std::string light_mask;

  private:
  /// @brief Invalidated copy constructor
  MSVehicleType(const MSVehicleType&);

  /// @brief Invalidated assignment operator
  MSVehicleType& operator=(const MSVehicleType&);
  
  /// Options container reference
  OptionsCont &oc;
};

#endif
