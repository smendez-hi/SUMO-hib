/****************************************************************************/
/// @file    MSDevice_FEV.h
/// @author  Emilio Martin
/// @author  Daniel Oancea
/// @date    Fri, 30.01.2012
/// @version $Id: MSDevice_FEV.h 11671 2012-01-07 20:14:30Z behrisch $
///
// A device which collects vehicular emissions (using FEV-reformulation)
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
#ifndef MSDevice_FEV_h
#define MSDevice_FEV_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <set>
#include <vector>
#include <map>
#include "MSDevice.h"
#include <utils/common/SUMOTime.h>
#include <microsim/MSVehicle.h>
#include <utils/common/WrappingCommand.h>


// ===========================================================================
// class declarations
// ===========================================================================
class MSLane;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSDevice_FEV
 * @brief A device which collects Full electric vehicles consumptions
 *
 * Each device collects the vehicular consumption
 *
 * @see MSDevice
 * @see
 */
class MSDevice_FEV : public MSDevice
{
  public:
    /** @brief Inserts MSDevice_FEV-options
     */
    static void insertOptions() ;

    /** @brief Build devices for the given vehicle, if needed
     *
     * The options are read and evaluated whether hbefa-devices shall be built
     *  for the given vehicle.
     *
     * For each seen vehicle, the global vehicle index is increased.
     *
     * The built device is stored in the given vector.
     *
     * @param[in] v The vehicle for which a device may be built
     * @param[in,out] into The vector to store the built device in
     */
    static void buildVehicleDevices
     (SUMOVehicle& v, std::vector<MSDevice*> &into);

  public:
    /// @name Methods called on vehicle movement / state change, overwriting MSDevice
    //  @{
    /// @{

    /** @brief Computes current emission values and adds them to their sums
      *
      * The vehicle's current emission values
      *  are computed using the current velocity and acceleration.
      *
      * @param[in] veh The regarded vehicle
      * @param[in] oldPos Position before the move-micro-timestep.
      * @param[in] newPos Position after the move-micro-timestep.
      * @param[in] newSpeed The vehicle's current speed
      * @return false, if the vehicle is beyond the lane, true otherwise
      * @see MSMoveReminder
      * @see MSMoveReminder::notifyMove
      * @see HelpersHBEFA
      */
    bool notifyMove
     (SUMOVehicle& veh, double oldPos, double newPos, double newSpeed);
     
    /// @}
    //  @}
    
    /** TODO DOCUMENTME */
    SUMOReal computeMaxSpeed(SUMOReal nextSpeed);
    
    /** @brief Called on writing tripinfo output
     *
     * @param[in] os The stream to write the information into
     * @exception IOError not yet implemented
     * @see MSDevice::tripInfoOutput
     */
    void generateOutput() const;
    /// @brief Destructor.
    ~MSDevice_FEV();
    MSDevice_FEV();
    double getMyCurrChargeRem() const;
    /* depth of discharge */
    SUMOReal getMyCurrDoD() const;
    double getMyCurrSpeed() const;
    double getMyLastChargeRem() const;
    double getMyLastDoD() const;
    double getMyLastSpeed() const;
    //void setMyCurrChargeRem(double my_curr_charge_rem);
    void setMyCurrDoD(double my_curr_DoD);
    void setMyCurrSpeed(double my_curr_speed);
    //void setMyLastChargeRem(double my_last_charge_rem);
    void setMyLastDoD(double my_last_DoD);
    void setMyLastSpeed(double my_last_speed);
    double getMySoH() const;
    //void setMySoH(double my_SOH);
    double getMyCurrE() const;
    void setMyCurrE(double my_curr_E);
    /*
    ???
    const char* cout;
    const char* cout;
    const char* cout;
    ???
    */
    int getCac(){return cac;}
    int getCch(){return cch;}
    int getCca(){return cca;}
    int getCvhl(){return cvhl;}
    int getCffl(){return cffl;}
    int getCcl(){return ccl;}
    void setCac(short ncac){cac=ncac;}
    void setCch(short ncch){cch=ncch;}
    void setCca(short ncca){cca=ncca;}
    void setCvhl(short ncvhl){cvhl=ncvhl;}
    void setCffl(short ncffl){cffl=ncffl;}
    void setCcl(short nccl){ccl=nccl;}
    /** This changes values for the upper six properties */
    void updateAuxiliaryServicesOnOffStatus();
    
  private:
    /** @brief Constructor
     *
     * @param[in] holder The vehicle that holds this device
     * @param[in] id The ID of the device
     */
    MSDevice_FEV(SUMOVehicle & holder, const std::string & id);
    
  private:  
    /// @name Internal storages for removed charge, state of health, speed, etc.
    //  @{
    /// @{
      
    SUMOReal my_curr_E;
    
    /** m/s */
    SUMOReal my_last_speed;
    SUMOReal my_curr_speed;
    
    /** W*h? / W? / A?
     *  (seems that are A(mps)
     */
    SUMOReal my_last_charge_rem;
    SUMOReal my_curr_charge_rem;
    
    /** percentage of battery spent? */
    SUMOReal my_last_DoD;
    SUMOReal my_curr_DoD;
    
    SUMOReal my_SOH;
    
    /** W*h */
    SUMOReal my_curr_energy_removed_a;
    SUMOReal my_curr_energy_removed_b;
    SUMOReal my_curr_energy_removed_c;
    SUMOReal my_curr_energy_removed_d;
    
    /**   Auxstatus - variables indicating which services are on. Refer to
     *  D.3.3, page 28.
     */
    //  @{
    /** current air conditioner status */
    int cac; // 0 to 3
    /** current cockpit heating status */
    int cch; // 0 to 3
    /** current car audio status */
    int cca; // 0 or 1
    /** current vehicle headlights status */
    int cvhl; // 0 to 3
    /** current front fog lights status */
    int cffl; // 0 or 1
    /** current cockpit lights */
    int ccl; // 0 or 1
    //  @}
    
    /// @}
    //  @}

  private:
    double computeElectricAuxiliaries();
    /// @brief Invalidated copy constructor.
    MSDevice_FEV(const MSDevice_FEV&);

    /// @brief Invalidated assignment operator.
    MSDevice_FEV& operator=(const MSDevice_FEV&);
};

#endif
