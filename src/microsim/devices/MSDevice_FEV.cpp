/* ALERT RELEVANT THEORETICAL MODELING DOCUMENTATION FOLLOWS
 * http://en.wikipedia.org/wiki/Peukert's_law
 */

/****************************************************************************/
/// @file    MSDevice_FEV.cpp
/// @author  Emilio Martin
/// @author  Daniel Oancea
/// @author  Uxio Prego
/// @date    ???, ??.??.2011
/// @version $Id: MSDevice_FEV.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// A device which collects vehicular consumptions (using FEV-reformulation)
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

#include "additionalConfig.h"
#include "MSDevice_FEV.h"
#include <cmath>
#include <iostream>
#include <string>
#include "microsim/MSNet.h"
#include <microsim/MSLane.h>
#include <microsim/MSVehicleControl.h>
#include <utils/options/OptionsCont.h>
#include <utils/common/HelpersHBEFA.h>
#include "utils/iodevices/OutputDevice.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS
static const bool calibracionParaStepCeroPuntoUno = false;
// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// static initialization methods
// ---------------------------------------------------------------------------
void MSDevice_FEV::insertOptions() {
  /*
  OptionsCont& oc = OptionsCont::getOptions();
  oc.addOptionSubTopic("Emissions");

  oc.doRegister("device.hbefa.probability", new Option_Float(0.));//!!! describe
  oc.addDescription("device.hbefa.probability", "Emissions", "The probability for a vehicle to have an emission logging device");

  oc.doRegister("device.hbefa.explicit", new Option_String());//!!! describe
  oc.addSynonyme("device.hbefa.explicit", "device.hbefa.knownveh", true);
  oc.addDescription("device.hbefa.explicit", "Emissions", "Assign a device to named vehicles");

  oc.doRegister("device.hbefa.deterministic", new Option_Bool(false)); //!!! describe
  oc.addDescription("device.hbefa.deterministic", "Emissions", "The devices are set deterministic using a fraction of 1000");
  */
}

void MSDevice_FEV::buildVehicleDevices
 (SUMOVehicle& v, std::vector<MSDevice*> &into)
{
  OptionsCont &oc=OptionsCont::getOptions();
  if(oc.getSimulationVerbosity()>1)
  {
    std::cout<<"----> void MSDevice_FEV::buildVehicleDevices(...)"<<std::endl;
    /* WRITE_MESSAGE("void MSDevice_FEV::buildVehicleDevices(...)") */
  }
  /* TASK
   *   This could be being dangerous!!!
   *   Because changing vehicles to something not DEFAULT_VTYPE_ID
   *   And also because having the MSDevice_FEV object for vehicles not being
   * Fev
   *   And maybe also because some other reasons (please note them here...)
   */
  // Only if this is a FEV...
  if(oc.getSimulationVerbosity()>2)
    std::cout<<" (v.getVehicleType().getID()=="<<v.getVehicleType().getID()<<")"
     <<std::endl;
  if(v.getVehicleType().getID()=="ecomile" ||
    v.getVehicleType().getID()=="nido" ||
    v.getVehicleType().getID()=="miev" ||
    v.getVehicleType().getID()=="temsa" ||
    v.getVehicleType().getID()=="ducato" ||
    v.getVehicleType().getID()=="comarth" ||
    v.getVehicleType().getID()=="transit" ||
    v.getVehicleType().getID()=="urbino" ||
    v.getVehicleType().getID()==DEFAULT_ELECVTYPE_ID)
  {
    // ...Build the FEV Device
    MSDevice_FEV* device = new MSDevice_FEV(
      v, v.getID() + "fevDevice" /* "fev_" + v.getID() */ );
    into.push_back(device);
  }
  else
  {
    // Should be of this type...
    if(v.getVehicleType().getID()==DEFAULT_VTYPE_ID)
    {
      // Compromise solution
      MSDevice_FEV* device = NULL;
    }
    else
    {
      // Not FEV nor normal?
      std::cout<<"ERROR, exiting... "<<__FILE__<<":"<<__LINE__<<std::endl;
      exit(-1);
    }
  }
}

// ---------------------------------------------------------------------------
// MSDevice_FEV-methods
// ---------------------------------------------------------------------------
MSDevice_FEV::MSDevice_FEV
 (SUMOVehicle& holder, const std::string& id) 
 :MSDevice(holder, id),
  my_last_speed(0), my_curr_speed(0.001), my_last_charge_rem(0),
  my_curr_charge_rem(0.0001), my_last_DoD(0), my_curr_DoD(0.000001), my_SOH(1),
  my_curr_energy_removed_a(0), 
  my_curr_energy_removed_b(0),
  my_curr_energy_removed_c(0)
{
  if(OptionsCont::getOptions().getSimulationVerbosity() > 1)
  {
    printf("----> MSDevice_FEV::MSDevice_FEV(...id=%s...)\n", id.c_str());
    /* WRITE_MESSAGE("void MSDevice_FEV::buildVehicleDevices(...)") */
  }
  if(OptionsCont::getOptions().getSimulationVerbosity() > 1)
  {
    printf("[100] MSDevice_FEV::MSDevice_FEV(...id=%s...)\n", id.c_str());
    printf("  holder.getID().c_str()=%s\n",holder.getID().c_str());
    printf("  holder.getVehicleType().getID().c_str()=%s\n",
     holder.getVehicleType().getID().c_str());      
  }
}

const SUMOReal stdGrav = 9.80665;

MSDevice_FEV::~MSDevice_FEV(){}

bool MSDevice_FEV::notifyMove
 (SUMOVehicle& veh, SUMOReal oldPos, SUMOReal newPos, SUMOReal newSpeed)
{
  SUMOReal PeuCap, accel, Fad, slope, Fhc, Fla, Frr, Fte, Pte, omega, Pbat,
   Pmot_out, Pmot_in, Torque, eff_mot, E, I, Ri, algo1, algo2, aux1, aux2, a, b,
   c, d,
#ifdef VEH_FAAM_ECOMILE // same as #if defined VEH_FAAM_ECOMILE
   k = 1.12 /* Peukert Coeff */;
#elif defined VEH_HIB_ORIGINAL
   k = 1.16 /* Was not 1.12 ?! */;
#endif
  OptionsCont &oc = OptionsCont::getOptions();
  if(oc.getSimulationVerbosity()>1)
    std::cout<<"----> bool MSDevice_FEV::notifyMove(..."
     //SUMOVehicle& veh, SUMOReal oldPos, SUMOReal newPos, SUMOReal newSpeed)
     <<")"<<std::endl;
  //const SUMOEmissionClass c = veh.gestdtVehicleType().getEmissionClass();
  /*
  doancea dprctd
  if(veh.getID().std::string::compare("t3") == 0 &&
     MSNet::getInstance()->getCurrentTimeStep() == 5100)
    int a = 0;
  */
  // Update capacity, dependent on envtemp
  const_cast<MSVehicleType&>(veh.getVehicleType()).updateCapacity(
   MSNet::getInstance()->getCurrentEnvTemp()/*, veh, this*/);
  const_cast<MSVehicleType&>(veh.getVehicleType()).updateAirDens(
   MSNet::getInstance()->getCurrentEnvTemp(),
   MSNet::getInstance()->getCurrentEnvHum()/*, veh, this*/);
  if(my_last_speed < 0)
    my_last_speed = 0;
  if(oc.getSimulationVerbosity()>2)
    std::cout<<"my_last_speed="<<my_last_speed<<std::endl;
  /*
  accel = (newSpeed - my_last_speed) / 
          ((1000.0/DELTA_T)*(1000.0/DELTA_T));
  */
  accel = (newSpeed - my_last_speed) / (DELTA_T / 1000.0);
  if(oc.getSimulationVerbosity()>2)
    std::cout<<"accel="<<accel<<"m/s^2"<<std::endl;
  /* Air resistance force against the vehicle's advance */
  Fad = 0.5 * veh.getVehicleType().getAirDens()
      * veh.getVehicleType().getFrontalArea()
      * veh.getVehicleType().getDragCoefficient()
      * newSpeed * newSpeed;
  if(oc.getSimulationVerbosity()>2)
    std::cout<<"Fad="<<Fad<<"N"<<std::endl;
   if(oc.getSimulationVerbosity()>2)
    std::cout << "[200] MSDevice_FEV::notifyMove(...)\n";
  //slope = dynamic_cast<MSVehicle*>(&veh)->getMyLane()->getMySlope();
  slope = static_cast<MSVehicle*>(&veh)->getMyLane()->getMySlope();
   if(oc.getSimulationVerbosity()>2)
  {
    if(slope != 0)
    {
      std::cout << "[210] MSDevice_FEV::notifyMove(...) - using a nonzero slope"
                   "=" << slope <<" for Fhc=" << Fhc <<" calculation"
                << std::endl;      
    }
    else
    {
      std::cout << "[220] MSDevice_FEV::notifyMove(...) - using a zero slope"
                   "=" << slope <<" for Fhc=" << Fhc <<" calculation"
                << std::endl;      
    }
  }
  if(slope>1)
  {
    if(oc.getSimulationVerbosity()>2)
      std::cout<<"> 1 slope detected: "<<slope<<" (setting to 1)"<<std::endl;
    slope = 1;
  }
  else if(slope<-1)
  {
    if(oc.getSimulationVerbosity()>2)
      std::cout<<"< -1 slope detected: "<<slope<<" (setting to -1)"<<std::endl;
    slope = -1;
  }
  else if(slope!=slope)
  {
    if(oc.getSimulationVerbosity()>2)
      std::cout<<"NaN slope detected: "<<slope<<" (setting to 0)"<<std::endl;
    slope = 0;
  }
  else
  {
    if(oc.getSimulationVerbosity()>2)
      std::cout<<"normal slope detected: "<<slope<<" (no action required)"<<std::endl;
  }
  if(oc.getSimulationVerbosity()>2)
  {
    std::cout<<" getID(){"<<getID()<<"}"<<std::endl;
    std::cout<<" veh.getID(){"<<veh.getID()<<"}"<<std::endl;
    std::cout<<" veh.getVehicleType().getMass{"<<veh.getVehicleType().getMass()<<"kg}"<<std::endl;
  }
  Fhc = veh.getVehicleType().getMass() * stdGrav * sin(slope);
  if(oc.getSimulationVerbosity()>2)
    std::cout<<"Fhc="<<Fhc<<"N"<<std::endl;
  /* hardwired 5% increment in Fla allows to ignore Fwa */
  Fla = 1.05 * veh.getVehicleType().getMass() * accel;
  //Fla = veh.getVehicleType().getMass() * accel;
  //Fwa = ...
  if(oc.getSimulationVerbosity()>2)
    std::cout<<"Fla="<<Fla<<"N"<<std::endl;
  /* Road resistance force against the vehicle's advance */
  Frr = veh.getVehicleType().getRollingRes()
      * veh.getVehicleType().getMass() * stdGrav;
  if(oc.getSimulationVerbosity()>2)
    std::cout<<"Frr="<<Frr<<"N"<<std::endl;
  /* Fte = Frr + Fad + Fhc + Fla + Fwa */
  Fte = Frr + Fad + Fhc + Fla;
  if(oc.getSimulationVerbosity()>2)
  {
    std::cout<<"Fte="<<Fte<<"N"<<std::endl;
    std::cout<<"newSpeed="<<newSpeed<<"m/s"<<std::endl;
  }
  Pte = Fte * newSpeed;
  if(oc.getSimulationVerbosity()>2)
    std::cout<<"Pte="<<Pte<<"W"<<std::endl;
  /* w = Gv (Omega = G * v) */
  omega = veh.getVehicleType().getGearRatio() * newSpeed;
  if(oc.getSimulationVerbosity()>2)  
    std::cout<<"omega="<<omega<<std::endl;
  Pbat = 0;
  Pmot_out = 0;
  Pmot_in = 0;
  Torque = 0;
  eff_mot = 0;
  my_last_charge_rem = my_curr_charge_rem;
  my_last_DoD = my_curr_DoD;
  my_last_speed = my_curr_speed;
  my_curr_speed = newSpeed;
  if(omega==0)
  {
    if(oc.getSimulationVerbosity()>2)
      std::cout<<"omega zero processing"<<std::endl;
    Pte = 0;
    Pmot_in = 0;
    Torque = 0;
    eff_mot = 0.5;
  }
  else
  {
    if(oc.getSimulationVerbosity()>2)
      std::cout<<"omega nonzero processing"<<std::endl;
    if(Pte < 0)
    {
      /* Regenerative case */
      Pte = veh.getVehicleType().getRegenRatio() * Pte;
      if(oc.getSimulationVerbosity()>2)
        std::cout<<"Pte="<<Pte<<"W"<<std::endl;    
    }
    /* Wrong? */
    /*
    if(Pte >= 0)
      Pmot_out = Pte / veh.getVehicleType().getGearSysEffc();
    else
      Pmot_out = Pte * veh.getVehicleType().getGearSysEffc();
    */    
    /* Right? */    
    if(Pte>=0)
      Pmot_out = Pte * veh.getVehicleType().getGearSysEffc();
    else
      Pmot_out = Pte / veh.getVehicleType().getGearSysEffc();    
    if(oc.getSimulationVerbosity()>2)
      std::cout<<"Pmot_out="<<Pmot_out<<"W"<<std::endl;    
    Torque = Pmot_out / omega;
    if(oc.getSimulationVerbosity()>2)
      std::cout<<"Torque="<<Torque<<std::endl;
    /* eff_mot is the motor system efficiency */
    /*
    if(Torque > 0)
      eff_mot = (Torque * omega)
          / ((Torque * omega)
              + (pow(Torque, 2.0)
                  * veh.getVehicleType().getCooperLosses())
              + (omega * veh.getVehicleType().getIronLosses())
              + (pow(omega, 3.0)
                  * veh.getVehicleType().getWindageLosses())
              + veh.getVehicleType().getConstantLosses());
    else if(Torque < 0)
      eff_mot = (-Torque * omega)
          / ((-Torque * omega)
              + (pow(Torque, 2.0)
                  * veh.getVehicleType().getCooperLosses())
              + (omega * veh.getVehicleType().getIronLosses())
              + (pow(omega, 3.0)
                  * veh.getVehicleType().getWindageLosses())
              + veh.getVehicleType().getConstantLosses());
    */
    eff_mot
      = (Torque * omega)
      / (Torque * omega
        +veh.getVehicleType().getCooperLosses()*pow(Torque,2)
        +veh.getVehicleType().getIronLosses()*omega
        +veh.getVehicleType().getWindageLosses()*pow(omega,3)
        +veh.getVehicleType().getConstantLosses());
    if(oc.getSimulationVerbosity()>2)
      std::cout<<"eff_mot="<<eff_mot<<std::endl;    
    if(/*Pmot_out*/Pte>=0)
      /* Wrong? */
      //Pmot_in = Pmot_out / eff_mot;
      /* Right? */
      Pmot_in = Pmot_out * eff_mot;
    else
      /* Wrong? */
      //Pmot_in = Pmot_out * eff_mot;
      /* Right */
      Pmot_in = Pmot_out / eff_mot;
    if(oc.getSimulationVerbosity()>2)
    {
      std::cout<<"Pmot_in="<<Pmot_in<<"W"<<std::endl;
    }
  }
  if(oc.getSimulationVerbosity()>2)
    std::cout<<"computeElectricAuxiliaries()="<<computeElectricAuxiliaries()<<std::endl;    
  //Pbat = Pmot_in + (DELTA_T / 1000.0) * computeElectricAuxiliaries(); //Need to include accessories.
  Pbat = Pmot_in + DELTA_T_hours * computeElectricAuxiliaries(); //Need to include accessories.
  if(oc.getSimulationVerbosity()>2)
    std::cout<<"Pbat="<<Pbat<<"W"<<std::endl;
  //Needs further modifications in order to include several kinds of batteries.
#if defined BATT_TYPE_LEAD_ACID
  E = veh.getVehicleType().getNumCells() * (2.15 - 0.15 * my_last_DoD);
  //Calculate internal resistance for lead acid.
  Ri = (0.022 / veh.getVehicleType().getCapacity()) * veh.getVehicleType().getNumCells();
#elif defined BATT_TYPE_LI_PO
  Ri = E = 0.;
#elif defined BATT_TYPE_CUSTOM_MODEL
  if(oc.getSimulationVerbosity()>2)
  {
    std::cout<<"my_last_DoD="<<OutputDevice::realString(my_last_DoD,9)<<std::endl;
    std::cout<<"0.15 * my_last_DoD="<<OutputDevice::realString(0.15 * my_last_DoD,9)<<std::endl;
    std::cout<<"2.15 - 0.15 * my_last_DoD="<<OutputDevice::realString(2.15 - 0.15 * my_last_DoD,9)<<std::endl;
  }
  E = veh.getVehicleType().getNumCells() * (2.15 - 0.15 * my_last_DoD);
  //E = veh.getVehicleType().getNumCells() * (1.84 + .15 - .15 * my_last_DoD); // 1.84 = 96/52 (Volts/NumBatts)
  if(oc.getSimulationVerbosity()>2)
    std::cout<<"E="<<OutputDevice::realString(E,9)<<std::endl;
  if(oc.getSimulationVerbosity()>2)
  {
    std::cout<<"veh.getVehicleType().getCapacity()="<<veh.getVehicleType().getCapacity()<<std::endl;
    std::cout<<"veh.getVehicleType().getNumCells()="<<veh.getVehicleType().getNumCells()<<std::endl;
  }
  Ri=(0.022/veh.getVehicleType().getCapacity())*veh.getVehicleType().getNumCells();
  if(oc.getSimulationVerbosity()>2)
    std::cout<<"Ri=(0.022/veh.getVehicleType().getCapacity())*veh.getVehicleType().getNumCells()="<<OutputDevice::realString(Ri,9)<<std::endl;  
#else
  std::cout<<"CRITICAL undefined batteries tipe model"<<std::endl;
  exit(0xffff);
#endif
  if(Pbat > 0)
  {
    /* Requiring energy from batteries */
     if(oc.getSimulationVerbosity()>2)
      std::cout << "[250] bool MSDevice_FEV::notifyMove(...) - the vehicle "
                << veh.getID() << " is requiring energy from its batteries (Pb"
                   "at=" << Pbat << ")" << std::endl;
    /*
    if (4 * Ri * Pbat > E * E)
      int c = 0;
    */
    /* E is Voc */
    a=E*E;
    if(oc.getSimulationVerbosity()>2)
      std::cout<<"E*E="<<a<<std::endl;
    b=4*Ri*Pbat;
    if(oc.getSimulationVerbosity()>2)
      std::cout<<"4*Ri*Pbat="<<b<<std::endl;
    c=a-b;
    if(oc.getSimulationVerbosity()>2)
      std::cout<<"E*E-4*Ri*Pbat="<<c<<std::endl;
    if(c<0)
      c=0;
    a=sqrt(c);
    if(oc.getSimulationVerbosity()>2)
      std::cout<<"sqrt(E*E-4*Ri*Pbat)="<<a<<std::endl;
    b=E-a;
    if(oc.getSimulationVerbosity()>2)
      std::cout<<"E-sqrt(E*E-4*Ri*Pbat)="<<b<<std::endl;
    a=2*Ri;
    if(oc.getSimulationVerbosity()>2)
      std::cout<<"2*Ri="<<a<<std::endl;
    I = b/a;
    if(oc.getSimulationVerbosity()>2)
      std::cout<<"I=(E-sqrt(E*E-4*Ri*Pbat))/2*Ri="<<I<<std::endl;
    aux1 = sqrt(E*E+4*Ri*Pbat);
    aux2 = pow(I,k);
    if(oc.getSimulationVerbosity()>2)
      std::cout << "[255] bool MSDevice_FEV::notifyMove(...) - my_last_charge_rem="
                << my_last_charge_rem << std::endl;
    if(calibracionParaStepCeroPuntoUno)
    {
      if(this->getHolder().getVehicleType().getID()=="ecomile")
      {
        if(my_last_speed*3.6>65)
        {
          my_curr_charge_rem=(
            my_last_charge_rem
            + aux2 * DELTA_T_hours
            * 1.2212
            / 5 // FIXME DANGER HAND ADJUSTMENT
            /* in A*h (Amp per hour) */)*1.25
            ;    
        }else if(my_last_speed*3.6>55){
          my_curr_charge_rem=(
            my_last_charge_rem
            + aux2 * DELTA_T_hours
            * 1.2212
            / 5 // FIXME DANGER HAND ADJUSTMENT
            /* in A*h (Amp per hour) */)*1.1
            ; 
        }else if(my_last_speed*3.6>45){
          my_curr_charge_rem=(
            my_last_charge_rem
            + aux2 * DELTA_T_hours
            * 1.2212
            / 5 // FIXME DANGER HAND ADJUSTMENT
            /* in A*h (Amp per hour) */)*1.05
            ; 
        }else{ /* 45 or less*/
          my_curr_charge_rem=
            my_last_charge_rem
            + aux2 * DELTA_T_hours
            * 1.2212
            / 5 // FIXME DANGER HAND ADJUSTMENT
            /* in A*h (Amp per hour) */
            ; 
        }
      }
    }
    else /* calibracionParaStepUno */
    {
      if(this->getHolder().getVehicleType().getID()=="miev")
      {
        if(my_last_speed*3.6>105)
        {
          // A good adjustment factor could be ???
          my_curr_charge_rem=my_last_charge_rem+(aux2*DELTA_T_hours)*4.583;
        }
        else if(my_last_speed*3.6>95)
        {
          // A good adjustment factor could be ???
          my_curr_charge_rem=my_last_charge_rem+(aux2*DELTA_T_hours)*4.;
        }
        else if(my_last_speed*3.6>85)
        {
          // A good adjustment factor could be ???
          my_curr_charge_rem=my_last_charge_rem+(aux2*DELTA_T_hours)*2.39;
        }
        else if(my_last_speed*3.6>75)
        {
          // A good adjustment factor could be ???
          my_curr_charge_rem=my_last_charge_rem+(aux2*DELTA_T_hours)*1.7171;
        }
        else if(my_last_speed*3.6>65)
        {
          // A good adjustment factor could be ???
          my_curr_charge_rem=my_last_charge_rem+(aux2*DELTA_T_hours)*1.647;
        }
        else if(my_last_speed*3.6>55)
        {
          // A good adjustment factor could be ???
          my_curr_charge_rem=my_last_charge_rem+(aux2*DELTA_T_hours)*.75;
        }
        else if(my_last_speed*3.6>45)
        {
          // A good adjustment factor could be ???
          my_curr_charge_rem=my_last_charge_rem+(aux2*DELTA_T_hours)*1.19;
        }
        else if(my_last_speed*3.6>35)
        {
          // A good adjustment factor could be ???
          my_curr_charge_rem=my_last_charge_rem+(aux2*DELTA_T_hours)*.15;
        }
        else if(my_last_speed*3.6>25)
        {
          // A good adjustment factor could be ???
          my_curr_charge_rem=my_last_charge_rem+(aux2*DELTA_T_hours)*.3;
        }
        else // 25 or less
        {
          // A good adjustment factor could be ???
          if(oc.getSimulationVerbosity()>2)
            std::cout<<"check25"<<std::endl;
          my_curr_charge_rem=my_last_charge_rem+(aux2*DELTA_T_hours)*.4;
        }
      }
      else if(this->getHolder().getVehicleType().getID()=="nido")
      {
        if(my_last_speed*3.6>75)
        {
          // A good adjustment factor could be 175
          my_curr_charge_rem=my_last_charge_rem+(aux2*DELTA_T_hours)*175;
        }
        else if(my_last_speed*3.6>65)
        {
          // A good adjustment factor could be 12
          my_curr_charge_rem=my_last_charge_rem+(aux2*DELTA_T_hours)*12;
        }
        else if(my_last_speed*3.6>55)
        {
          // A good adjustment factor could be 4
          my_curr_charge_rem=my_last_charge_rem+(aux2*DELTA_T_hours)*4;
        }
        else if(my_last_speed*3.6>45)
        {
          // A good adjustment factor could be 2.5
          my_curr_charge_rem=my_last_charge_rem+(aux2*DELTA_T_hours)*2.5;
        }
        else if(my_last_speed*3.6>35)
        {
          // A good adjustment factor could be 1.08
          my_curr_charge_rem=my_last_charge_rem+(aux2*DELTA_T_hours)*(1.08);
        }
        else if(my_last_speed*3.6>25)
        {
          // A good adjustment factor could be 1.1326
          my_curr_charge_rem=my_last_charge_rem+(aux2*DELTA_T_hours)*(1.1326);
        }
        else if(my_last_speed*3.6>15)
        {
          // A good adjustment factor could be 1.22
          my_curr_charge_rem=my_last_charge_rem+(aux2*DELTA_T_hours)*(1.22);
        }
        else // 15 or less
        {
          // A good adjustment factor could be 1.555
          my_curr_charge_rem=my_last_charge_rem+(aux2*DELTA_T_hours)*(1.555);
        }
      }
      else if(this->getHolder().getVehicleType().getID()=="ducato")
      {
	//FILL ME ??
	my_curr_charge_rem=my_last_charge_rem+aux2*DELTA_T_hours;
      }
      else if(this->getHolder().getVehicleType().getID()=="comarth")
      {
	//FILL ME ??
	my_curr_charge_rem=my_last_charge_rem+aux2*DELTA_T_hours;
      }
      else if(this->getHolder().getVehicleType().getID()=="transit")
      {
	//FILL ME ??
	my_curr_charge_rem=my_last_charge_rem+aux2*DELTA_T_hours;
      }
      else if(this->getHolder().getVehicleType().getID()=="urbino")
      {
	//FILL ME ??
	my_curr_charge_rem=my_last_charge_rem+aux2*DELTA_T_hours;
      }
      else
        my_curr_charge_rem=my_last_charge_rem+aux2*DELTA_T_hours;
    }
    if(oc.getSimulationVerbosity()>2)
    {
      std::cout << "[260] bool MSDevice_FEV::notifyMove(...) - my_curr_charge_rem = "
                << my_curr_charge_rem << std::endl;
      if(my_curr_charge_rem != my_curr_charge_rem)
      {
        /* my_curr_charge_rem turned into nan */
        std::cout << "my_last_charge_rem = " << my_last_charge_rem << std::endl;
        std::cout << "((I * (DELTA_T / 1000.0)) / 3600.0) = "
                  <<  ((I * (DELTA_T / 1000.0)) / 3600.0) << std::endl;
        if(((I * (DELTA_T / 1000.0)) / 3600.0) !=
           ((I * (DELTA_T / 1000.0)) / 3600.0))
        {
          /* ((I * (DELTA_T / 1000.0)) / 3600.0) turned into nan */
          my_curr_charge_rem = 0;
        }
      }
    }
  }
  else /* Pbat < 0 */
  {
    /* Generating energy to batteries */
    if(oc.getSimulationVerbosity()>2)
      std::cout << "[265] bool MSDevice_FEV::notifyMove(...) - the vehicle "
                << veh.getID() << " is generating energy to its batteries (Pb"
                   "at=" << Pbat << ")" << std::endl;
    a=E*E;
    if(oc.getSimulationVerbosity()>2)
      std::cout<<"E*E="<<a<<std::endl;
    b=4*Ri*Pbat;
    if(oc.getSimulationVerbosity()>2)
      std::cout<<"4*Ri*Pbat="<<b<<std::endl;
    c=a+b;
    if(oc.getSimulationVerbosity()>2)
      std::cout<<"E*E+4*Ri*Pbat="<<c<<std::endl;
    if(c<0)
      c=0;
    a=sqrt(c);
    if(oc.getSimulationVerbosity()>2)
      std::cout<<"sqrt(E*E+4*Ri*Pbat)="<<a<<std::endl;
    b=-E+a;
    if(oc.getSimulationVerbosity()>2)
      std::cout<<"-E+sqrt(E*E+4*Ri*Pbat)="<<b<<std::endl;
    a=2*2*Ri;
    if(oc.getSimulationVerbosity()>2)
      std::cout<<"2*2*Ri="<<a<<std::endl;
    //I = abs((-E + sqrt((E * E) + 4 * 2 * Ri * Pbat)) / (2 * 2 * Ri));
    //I = abs((-E + sqrt((E * E) + 4 * Ri * Pbat)) / (2 * Ri));
    //I = (-E + sqrt((E * E) + 4 * Ri * Pbat)) / (2 * Ri);
    //I = (-E + sqrt((E * E) + 4 * Ri * Pbat)) / (2 * (2 * Ri)); <-- esta
    I = b/a;
    if(oc.getSimulationVerbosity()>2)
      std::cout<<"I=(-E+sqrt(E*E+4*Ri*Pbat))/(2*2*Ri)="<<I<<std::endl;
    if(oc.getSimulationVerbosity()>2)
      std::cout << "[270] bool MSDevice_FEV::notifyMove(...) - my_last_charge_rem = "
                << my_last_charge_rem << std::endl;
    if(calibracionParaStepCeroPuntoUno)
    {
      if(this->getHolder().getVehicleType().getID()=="ecomile")
      {
        if(my_last_speed*3.6>65)
        {
          my_curr_charge_rem=(
            my_last_charge_rem
            - I * DELTA_T_hours
            * 1.2212
            / 5 // FIXME DANGER HAND ADJUSTMENT
            /* in A*h (Amp per hour) */)*1.25
            ;    
        }else if(my_last_speed*3.6>55){
          my_curr_charge_rem=(
            my_last_charge_rem
            - I * DELTA_T_hours
            * 1.2212
            / 5 // FIXME DANGER HAND ADJUSTMENT
            /* in A*h (Amp per hour) */)*1.1
            ; 
        }else if(my_last_speed*3.6>45){
          my_curr_charge_rem=(
            my_last_charge_rem
            - I * DELTA_T_hours
            * 1.2212
            / 5 // FIXME DANGER HAND ADJUSTMENT
            /* in A*h (Amp per hour) */)*1.05
            ; 
        }else{ /* 45 or less */
          my_curr_charge_rem=
            my_last_charge_rem
            - I * DELTA_T_hours
            * 1.2212
            / 5 // FIXME DANGER HAND ADJUSTMENT
            /* in A*h (Amp per hour) */
            ; 
        }
      }
    }
    else /* calibracionParaStepUno */
    {
      if(this->getHolder().getVehicleType().getID()=="miev")
      {
        if(my_last_speed*3.6>105)
        {
          // A good adjustment factor could be ???
          my_curr_charge_rem=my_last_charge_rem-(I*DELTA_T_hours)*4.583;
        }
        else if(my_last_speed*3.6>95)
        {
          // A good adjustment factor could be ???
          my_curr_charge_rem=my_last_charge_rem-(I*DELTA_T_hours)*4.;
        }
        else if(my_last_speed*3.6>85)
        {
          // A good adjustment factor could be ???
          my_curr_charge_rem=my_last_charge_rem-(I*DELTA_T_hours)*2.39;
        }
        else if(my_last_speed*3.6>75)
        {
          // A good adjustment factor could be ???
          my_curr_charge_rem=my_last_charge_rem-(I*DELTA_T_hours)*1.7171;
        }
        else if(my_last_speed*3.6>65)
        {
          // A good adjustment factor could be ???
          my_curr_charge_rem=my_last_charge_rem-(I*DELTA_T_hours)*1.647;
        }
        else if(my_last_speed*3.6>55)
        {
          // A good adjustment factor could be ???
          my_curr_charge_rem=my_last_charge_rem-(I*DELTA_T_hours)*.75;
        }
        else if(my_last_speed*3.6>45)
        {
          // A good adjustment factor could be ???
          my_curr_charge_rem=my_last_charge_rem-(I*DELTA_T_hours)*1.19;
        }
        else if(my_last_speed*3.6>35)
        {
          // A good adjustment factor could be ???
          my_curr_charge_rem=my_last_charge_rem-(I*DELTA_T_hours)*.15;
        }
        else if(my_last_speed*3.6>25)
        {
          // A good adjustment factor could be ???
          my_curr_charge_rem=my_last_charge_rem-(I*DELTA_T_hours)*.3;
        }
        else // 25 or less
        {
          // A good adjustment factor could be ???
          my_curr_charge_rem=my_last_charge_rem-(I*DELTA_T_hours)*.4;
        }
      }
      else if(this->getHolder().getVehicleType().getID()=="nido")
      {
        if(my_last_speed*3.6>75)
        {
          // A good adjustment factor could be 175
          my_curr_charge_rem=my_last_charge_rem-(I*DELTA_T_hours)*175;
        }
        else if(my_last_speed*3.6>65)
        {
          // A good adjustment factor could be 12
          my_curr_charge_rem=my_last_charge_rem-(I*DELTA_T_hours)*12;
        }
        else if(my_last_speed*3.6>55)
        {
          // A good adjustment factor could be 4
          my_curr_charge_rem=my_last_charge_rem-(I*DELTA_T_hours)*4;
        }
        else if(my_last_speed*3.6>45)
        {
          // A good adjustment factor could be 2.5
          my_curr_charge_rem=my_last_charge_rem-(I*DELTA_T_hours)*2.5;
        }
        else if(my_last_speed*3.6>35)
        {
          // A good adjustment factor could be 1.08
          my_curr_charge_rem=my_last_charge_rem-(I*DELTA_T_hours)*(1.08);
        }
        else if(my_last_speed*3.6>25)
        {
          // A good adjustment factor could be 1.1326
          my_curr_charge_rem=my_last_charge_rem-(I*DELTA_T_hours)*(1.1326); // TODO REDUCIR
        }
        else if(my_last_speed*3.6>15)
        {
          // A good adjustment factor could be 1.22
          my_curr_charge_rem=my_last_charge_rem-(I*DELTA_T_hours)*(1.22); // TODO REDUCIR
        }
        else // 15 or less
        {
          // A good adjustment factor could be 1.555
          my_curr_charge_rem=my_last_charge_rem-(I*DELTA_T_hours)*(1.555); // TODO REDUCIR
        }
      }
      else if(this->getHolder().getVehicleType().getID()=="ducato")
      {
	//FILL ME ??
	my_curr_charge_rem=my_last_charge_rem-I*DELTA_T_hours;
      }
      else if(this->getHolder().getVehicleType().getID()=="comarth")
      {
	//FILL ME ??
	my_curr_charge_rem=my_last_charge_rem-I*DELTA_T_hours;
      }
      else if(this->getHolder().getVehicleType().getID()=="transit")
      {
	//FILL ME ??
	my_curr_charge_rem=my_last_charge_rem-I*DELTA_T_hours;
      }
      else if(this->getHolder().getVehicleType().getID()=="urbino")
      {
	//FILL ME ??
	my_curr_charge_rem=my_last_charge_rem-I*DELTA_T_hours;
      }
      else
        my_curr_charge_rem=my_last_charge_rem-I*DELTA_T_hours;
    }
    if(oc.getSimulationVerbosity()>2)
    {
      std::cout << "[275] bool MSDevice_FEV::notifyMove(...) - my_curr_charge_rem = "
                << my_curr_charge_rem << std::endl;
      if(my_curr_charge_rem != my_curr_charge_rem)
      {
        /* my_curr_charge_rem turned into nan */
        std::cout << "my_last_charge_rem = " << my_last_charge_rem << std::endl;
        std::cout << "(I * DELTA_T_hours) = "
                  <<  (I * DELTA_T_hours) << std::endl;
      }
    }
  }
  if(oc.getSimulationVerbosity()>2)
  {
    std::cout<<" my_curr_charge_rem{"
     <<OutputDevice::realString(my_curr_charge_rem,9)<<"}"<<std::endl;
    std::cout<<" my_last_charge_rem{"
     <<OutputDevice::realString(my_last_charge_rem,9)<<"}"<<std::endl;
  }
  if((my_curr_charge_rem/my_last_charge_rem>1.1)&&(my_curr_charge_rem>.05))
  {
    if(oc.getSimulationVerbosity()>2)
    {
      std::cout<<" CRITICAL: my_curr_charge_rem/my_last_charge_rem>1.1"
       <<std::endl;
      std::cout<<" my_curr_charge_rem/my_last_charge_rem{"
       <<OutputDevice::realString(my_curr_charge_rem/my_last_charge_rem,9)<<"}"
       <<std::endl;
    }
    my_curr_charge_rem=1.1*my_last_charge_rem;
  }
  if((my_curr_charge_rem-my_last_charge_rem>0.0125)&&(my_curr_charge_rem>.05))
  {
    if(oc.getSimulationVerbosity()>2)
    {
      std::cout<<" CRITICAL: my_curr_charge_rem-my_last_charge_rem>0.0125"
       <<std::endl;
      std::cout<<" my_curr_charge_rem-my_last_charge_rem{"
       <<OutputDevice::realString(my_curr_charge_rem-my_last_charge_rem,9)<<"}"
       <<std::endl;
    }
    my_curr_charge_rem=.0125+my_last_charge_rem;
  }
  if(oc.getSimulationVerbosity()>2)
    std::cout<<" my_curr_charge_rem{"
     <<OutputDevice::realString(my_curr_charge_rem,9)<<"}"<<std::endl;  
  PeuCap = pow((veh.getVehicleType().getCapacity() / 
                veh.getVehicleType().getTimeCharge()),
               k)
         * veh.getVehicleType().getTimeCharge();
  if(oc.getSimulationVerbosity()>2)
    std::cout << "[280] bool MSDevice_FEV::notifyMove(...) - PeuCap = " << PeuCap
              << std::endl;         
  my_curr_DoD = my_curr_charge_rem / PeuCap;
  //my_curr_DoD /= 5; // DANGER
  if(oc.getSimulationVerbosity()>2)
    std::cout << "[285] bool MSDevice_FEV::notifyMove(...) - my_curr_DoD = " 
              << my_curr_DoD << std::endl;
  if(my_curr_DoD > 1)
  {
    std::cout<<"CRITICAL ["<<veh.getID()<<"] DoD > 1 (this means battery finish"
     <<"ed, reverting to DoD 0)"<<std::endl;
    my_last_DoD = .000001;
    my_curr_DoD = .00000105;
    my_last_charge_rem = .0001;
    my_curr_charge_rem = .000105;
  }
  if(oc.getSimulationVerbosity()>2)
    std::cout << "[290] my_SOH = " << my_SOH << std::endl;
  /* Nonsense? */
  /*   According to wikipedia 20120925, 'As SOH does not correspond to a
   * particular physical quality, there is no consensus in the industry on how
   * SOH should be determined. 
   */
  /* Additionally, we currently don't know how to modelize the degradation of
   * the battery health
   */
  my_SOH = (veh.getVehicleType().getCapacity() 
           -my_curr_charge_rem/1000)
    / veh.getVehicleType().getCapacity();
  if(oc.getSimulationVerbosity()>2)
  {
    if(my_SOH != my_SOH)
    {
      /* my_SOH turned nan */
      if((veh.getVehicleType().getCapacity() - my_curr_charge_rem) !=
         (veh.getVehicleType().getCapacity() - my_curr_charge_rem))
      {
        /* problem traced back to the dividend */
        std::cout << "(veh.getVehicleType().getCapacity() - my_curr_charge_rem) ="
                  << " " << (veh.getVehicleType().getCapacity() - 
                           my_curr_charge_rem) << std::endl;
        if(my_curr_charge_rem != my_curr_charge_rem)
        {
          /* problem traced back to my_curr_charge_rem */
          std::cout << "my_curr_charge_rem = " << my_curr_charge_rem 
                    << std::endl;
        }
      }
      else
      {
        /* problem traced back to the divider */
        std::cout << "veh.getVehicleType().getCapacity() ="
                  << " " << veh.getVehicleType().getCapacity() << std::endl;         
      }
      ;
    }
    else
    {
      ;
    }
  }
  my_curr_energy_removed_a += my_curr_charge_rem * E;
  my_curr_energy_removed_b = veh.getVehicleType().getCapacity()
    //* veh.getVehicleType().getNumCells()
    * my_curr_DoD;
  my_curr_energy_removed_c=veh.getVehicleType().getCapacity()
    //* veh.getVehicleType().getNumCells()
    *my_curr_DoD*E;
  /* Nonsense? (remember that DELTA_T_hours is 'hours in one DELTA_T' and 
   * DELTA_T is 0.1 secs
   */
  my_curr_energy_removed_d += Pbat * DELTA_T_hours;
  algo1 += my_curr_DoD * E;
  algo2 += my_curr_charge_rem;
  if(oc.getSimulationVerbosity()>2)
  {
    std::cout << "veh.getVehicleType().getCapacity()=" <<
      veh.getVehicleType().getCapacity() << " (per battery?)" << std::endl;
    std::cout << "veh.getVehicleType().getNumCells()=" <<
      veh.getVehicleType().getNumCells() << std::endl;
    std::cout << "my_curr_speed = " << my_curr_speed << std::endl;
    std::cout << "my_curr_DoD = " << OutputDevice::realString(my_curr_DoD, 9) << std::endl;    
    std::cout<<" my_curr_charge_rem = " << my_curr_charge_rem << std::endl;
    std::cout<<" E = " << E << std::endl;
    std::cout<<" my_curr_charge_rem * E = " << my_curr_charge_rem * E << std::endl;
    std::cout<<" my_curr_energy_removed_a = "<<my_curr_energy_removed_a<<std::endl;
    std::cout<<" my_curr_energy_removed_b = "<<my_curr_energy_removed_b<<std::endl;
    std::cout<<" Pbat * DELTA_T_hours = "<<Pbat*DELTA_T_hours<<std::endl;
    std::cout<<" my_curr_energy_removed_c = "<<my_curr_energy_removed_c<<std::endl;
    std::cout << "Pbat = " << Pbat << std::endl;
    std::cout << "DELTA_T = " << DELTA_T << std::endl;
    std::cout << "DELTA_T_secs = " << DELTA_T_secs << std::endl;
    std::cout << " (std::cout) DELTA_T_hours = " << DELTA_T_hours << std::endl;
    std::cout << " (std::cout) OutputDevice::realString(DELTA_T_hours, 9) = " << OutputDevice::realString(DELTA_T_hours, 9) << std::endl;
    printf(" (printf) DELTA_T_hours = %f8\n", DELTA_T_hours);
    std::cout << " (std::cout) Pbat * DELTA_T_hours = " << Pbat * DELTA_T_hours << std::endl;
    printf(" (printf) Pbat * DELTA_T_hours = %f8\n",Pbat * DELTA_T_hours);
    std::cout << "eff_mot = " << eff_mot << std::endl;    
    std::cout << "algo1 = " << algo1 << std::endl;  
    std::cout << "algo2 = " << algo2 << std::endl;
    std::cout << "slope = " << slope << std::endl;  
  }  
  if(oc.getSimulationVerbosity()>2)
    std::cout << "my_SOH = " << my_SOH << std::endl;
  my_curr_E = E; //Voltage.
  /*if (my_SOH >1)
   std::cout << "asldkjas";*/
  /*Only for debug purpose.
   If in file.cfg.sumo is set on the option <tripinfo-fev value="output.xml" />
   will be generated a new file with this information.
   */
  /* Print fev trip info tag for each notifyMove() loop? */
  if(OptionsCont::getOptions().isSet("tripinfo-fev"))
  {
    OutputDevice& os = OutputDevice::getDeviceByOption("tripinfo-fev");
    /*
    (os.openTag("fev") << " ID=\"" << myID << "\" timestep=\""
        << OutputDevice::realString(
            MSNet::getInstance()->getCurrentTimeStep(), 6)
        << "\" speedC-1=\""
        << OutputDevice::realString(my_last_speed * 3.6, 6)
        << "\" speedC=\""
        << OutputDevice::realString(my_curr_speed * 3.6, 6)
        << "\" Charge_removedC-1=\""
        << OutputDevice::realString(my_last_charge_rem, 6)
        << "\" Charge_removedC=\""
        << OutputDevice::realString(my_curr_charge_rem, 6)
        << "\" DoDC-1=\"" << OutputDevice::realString(my_last_DoD, 6)
        << "\" DoDC=\"" << OutputDevice::realString(my_curr_DoD, 6)
        << "\" myPositionOnLane=\"" << OutputDevice::realString(myHolder.getPositionOnLane(),6)
        << "\" myTraveledDistance=\"" << OutputDevice::realString(dynamic_cast < MSVehicle& > (myHolder).getCurrentTraveledDistance(),6) << "\""
        << " currentTimeStep=\"" << MSNet::getInstance()->getCurrentTimeStep() << "\""
    ).closeTag(true);
    */
    os.openTag("fev");
    os<< " ID=\"" << myID << "\""
      /* Just 1 decimal needed for timestep. Division by 1. turns potential int to float */
      << " timestep=\"" << OutputDevice::realString(MSNet::getInstance()->getCurrentTimeStep()/SECS_IN_CURENT_TIME_STEP/1., 1) << "\""
      << " speedC-1=\"" << OutputDevice::realString(my_last_speed * 3.6, 6) << "\""
      << " speedC=\"" << OutputDevice::realString(my_curr_speed * 3.6, 6) << "\""
      << " Charge_removedC-1=\"" << OutputDevice::realString(my_last_charge_rem, 6) << "\""
      << " Charge_removedC=\"" << OutputDevice::realString(my_curr_charge_rem, 6) << "\""
     <<" my_curr_energy_removed_a=\""<<OutputDevice::realString(my_curr_energy_removed_a, 6)<<"\""
     <<" my_curr_energy_removed_b=\""<<OutputDevice::realString(my_curr_energy_removed_b, 6)<<"\""
     <<" my_curr_energy_removed_c=\""<<OutputDevice::realString(my_curr_energy_removed_c, 6)<<"\""
     <<" my_curr_energy_removed_d=\""<<OutputDevice::realString(my_curr_energy_removed_d, 6)<<"\""
     <<" my_curr_E=\""<<OutputDevice::realString(my_curr_E, 6)<<"\""      
      << " energySpent=\"" << OutputDevice::realString(my_curr_energy_removed_c, 6) << "\""
      << " DoDC-1=\"" << OutputDevice::realString(my_last_DoD, 9) << "\""
      << " DoDC=\"" << OutputDevice::realString(my_curr_DoD, 9) << "\""
      << " myPositionOnLane=\"" << OutputDevice::realString(myHolder.getPositionOnLane(),6) << "\""
      //<< " myTraveledDistance=\"" << OutputDevice::realString(dynamic_cast < MSVehicle& > (myHolder).getCurrentTraveledDistance(),6) << "\""
      << " myTraveledDistance=\"" << OutputDevice::realString(static_cast < MSVehicle& > (myHolder).getCurrentTraveledDistance(),6) << "\""
      //<< " currentTimeStep=\"" << MSNet::getInstance()->getCurrentTimeStep() << "\""
    ;
    os.closeTag(true);
  }
  return true;
}

SUMOReal MSDevice_FEV::computeMaxSpeed(SUMOReal newSpeed)
{
  //if(newSpeed>2.8) // m/s
  //if(newSpeed>10) // m/s
  //  return 999;
  /* else we are at low velocity so do the calculation */
  SUMOReal Fhc, Fla, Frr, Pte, omega, Pbat, Pmot_out, Pmot_in, Torque, eff_mot,
    res, accel, Fad, slope;
  OptionsCont &oc = OptionsCont::getOptions();
  if(oc.getSimulationVerbosity()>1)
  {
    std::cout << "----> SUMOReal MSDevice_FEV::computeMaxSpeed(SUMOReal newSpeed="
              << newSpeed << ")" << std::endl;
    /*
    std::string message;
    message = "SUMOReal MSDevice_FEV::computeMaxSpeed(SUMOReal newSpeed=";
    message += newSpeed;
    message += ")";
    WRITE_MESSAGE(message);
    */
  }
  if(my_last_speed < 0)
    my_last_speed = 0;
  else
  {
    // Then which was my_last_speed?
    if(oc.getSimulationVerbosity()>2)
    {
      std::cout << "[100] SUMOReal MSDevice_FEV::computeMaxSpeed(SUMOReal newSpeed="
                << newSpeed << ") - my_last_speed=" << my_last_speed
                << std::endl;
    }
  }
  SUMOVehicle& veh = getHolder();
  res = 0;
  accel = (newSpeed - my_last_speed) / (1000.0 / DELTA_T);
  // Update Air Density
  //veh.getVehicleType().setAirDensWithTempAndHum(oc.getFloat("fiet"),oc.getFloat("fieh"));
  //SUMOReal fiet=oc.getFloat("fiet");
  //SUMOReal fieh=oc.getFloat("fieh");
  //const_cast<MSVehicleType&>(veh.getVehicleType()).setAirDensWithTempAndHum(fiet,fieh);
  const_cast<MSVehicleType&>(veh.getVehicleType()).updateAirDens(
   MSNet::getInstance()->getCurrentEnvTemp(),
   MSNet::getInstance()->getCurrentEnvHum());
  Fad = 0.5 * veh.getVehicleType().getAirDens()
            * veh.getVehicleType().getFrontalArea()
            * veh.getVehicleType().getDragCoefficient()
            * newSpeed * newSpeed;
  //slope = dynamic_cast<MSVehicle *>(&veh)->getMyLane()->getMySlope();
  slope = static_cast<MSVehicle *>(&veh)->getMyLane()->getMySlope();
  //slope = dynamic_cast<MSVehicle *>(&veh)->getMyLane()->
  if(slope == 0)
  {
    if(oc.getSimulationVerbosity()>2)
    {
      std::cout << "[150] SUMOReal MSDevice_FEV::computeMaxSpeed(SUMOReal newSpeed="
                << newSpeed << ") - zero slope=" << slope << std::endl;
    } 
  }
  else
  {
    if(oc.getSimulationVerbosity()>2)
    {
      std::cout << "[200] SUMOReal MSDevice_FEV::computeMaxSpeed(SUMOReal newSpeed="
                << newSpeed << ") - nonzero slope=" << slope << std::endl;
    }
  }
  if(slope>1)
  {
    if(oc.getSimulationVerbosity()>2)
      std::cout<<"> 1 slope detected: "<<slope<<" (setting to 1)"<<std::endl;
    slope = 1;
  }
  else if(slope<-1)
  {
    if(oc.getSimulationVerbosity()>2)
      std::cout<<"< -1 slope detected: "<<slope<<" (setting to -1)"<<std::endl;
    slope = -1;
  }
  else if(slope!=slope)
  {
    if(oc.getSimulationVerbosity()>2)
      std::cout<<"NaN slope detected: "<<slope<<" (setting to 0)"<<std::endl;
    slope = 0;
  }
  else
  {
    if(oc.getSimulationVerbosity()>2)
      std::cout<<"normal slope detected: "<<slope<<" (no action required)"<<std::endl;
  }
  Fhc = veh.getVehicleType().getMass() * stdGrav * sin(slope);
  Fla = 1.05 * veh.getVehicleType().getMass() * accel;
  Frr = veh.getVehicleType().getRollingRes()
      * veh.getVehicleType().getMass() * stdGrav;
  Pte = (Frr + Fad + Fhc + Fla) * newSpeed;
  omega = veh.getVehicleType().getGearRatio() * newSpeed;
  Pbat = 0;
  Pmot_out = 0;
  Pmot_in = 0;
  Torque = 0;
  eff_mot = 0;
  my_last_charge_rem = my_curr_charge_rem;
  my_last_DoD = my_curr_DoD;
  my_last_speed = my_curr_speed;
  my_curr_speed = newSpeed;
  /*
  doancea dprctd
  if (veh.getID().std::string::compare("t87") == 0
      && MSNet::getInstance()->getCurrentTimeStep() >= 493000)
    int a = 0;
  */
  if (omega == 0) {
    Pte = 0;
    Pmot_in = 0;
    Torque = 0;
    eff_mot = 0.5;
  } else {
    if (Pte < 0)
      Pte = veh.getVehicleType().getRegenRatio() * Pte;
    
    /*
    if (Pte >= 0)
      Pmot_out = Pte / veh.getVehicleType().getGearSysEffc();
    else
      Pmot_out = Pte * veh.getVehicleType().getGearSysEffc();
    */
    if (Pte >= 0)
      Pmot_out = Pte * veh.getVehicleType().getGearSysEffc();
    else
      Pmot_out = Pte / veh.getVehicleType().getGearSysEffc();    
    
    Torque = Pmot_out / omega;
    if (Torque > 0)
      eff_mot = (Torque * omega)
          / ((Torque * omega)
              + (pow(Torque, 2.0)
                  * veh.getVehicleType().getCooperLosses())
              + (omega * veh.getVehicleType().getIronLosses())
              + (pow(omega, 3.0)
                  * veh.getVehicleType().getWindageLosses())
              + veh.getVehicleType().getConstantLosses());
    else if (Torque < 0)
      eff_mot = 
        (-Torque * omega) /
        (-Torque * omega +
          pow(Torque, 2.0) * veh.getVehicleType().getCooperLosses() +
          omega * veh.getVehicleType().getIronLosses() +
          pow(omega, 3.0) * veh.getVehicleType().getWindageLosses() +
          veh.getVehicleType().getConstantLosses());
    if (Pmot_out >= 0)
      Pmot_in = Pmot_out / eff_mot;
    else
      Pmot_in = Pmot_out * eff_mot;
  }
  Pbat = Pmot_in + computeElectricAuxiliaries(); //Need to include accessories.
      //Needs further modifications in order to include lead acid batteries.
  double E = veh.getVehicleType().getNumCells() * (2.15 - 0.15 * my_last_DoD);
  double I = 0;
  //Calcultate internal resistence for lead acid.
  double Ri = (0.022 / veh.getVehicleType().getCapacity()) *
    veh.getVehicleType().getNumCells();
  /*my_last_charge_rem = my_curr_charge_rem;
   my_last_DoD = my_curr_DoD;
   my_last_speed = my_curr_speed;
   my_curr_speed = newSpeed;*/
  res = ((E * E) * eff_mot * veh.getVehicleType().getGearSysEffc()) /
    (4 * Ri * Pte);
  if(res < newSpeed)
  {
    if(oc.getSimulationVerbosity()>1)
      std::cout << "<---- SUMOReal MSDevice_FEV::computeMaxSpeed(SUMOReal newSp"
        <<"eed{"<<newSpeed<<"}) - returning res="<<res<<std::endl;
    return res;
  }
  else
  {
    if(oc.getSimulationVerbosity()>1)
      std::cout << "<---- SUMOReal MSDevice_FEV::computeMaxSpeed(SUMOReal newSp"
        <<"eed{"<< newSpeed << "}) - returning newSpeed="<<newSpeed<<std::endl;
    return newSpeed;
  }
}

/**
 * Added by HI-iberia.
 * Calculates the electric power consumption of the vehicle
 * auxiliaries such as: radio, heating, head lights,
 * fog lights and cockpit lights.
 */
SUMOReal MSDevice_FEV::computeElectricAuxiliaries()
{
  OptionsCont &oc = OptionsCont::getOptions();
  SUMOReal res, radio, heating_AC, head_lights, fog_lights, cockpit_lights;
  std::string aux_mask, light_mask;//, currentTimeString;
  bool night;
  SUMOVehicle &veh = getHolder();
  res = 0;
  radio = veh.getVehicleType().getRadioConsumption();
  heating_AC = veh.getVehicleType().getHeatingAcConsumption();
  head_lights = veh.getVehicleType().getVehicleHeadligthsConsumption();
  fog_lights = veh.getVehicleType().getFrontFogLightsConsumption();
  cockpit_lights = veh.getVehicleType().getCockpitLightsConsumption();
  /* ??? */
  aux_mask = veh.getVehicleType().getAuxMask();
  light_mask = veh.getVehicleType().getLightMask();
  //if(oc.isSet("rlas"))
  //{
    // Radio
    //if(rand()%5==0)
    res += radio * getCca();
    // Heating and air conditioner
    //if(MSNet::getInstance()->isWinter()||
    //   (MSNet::getInstance()->isSummer()&&
    //    (MSNet::getInstance()->getCity()=="cambiano"||
    //     MSNet::getInstance()->getCity()=="munchen"&&rand()%2==0)))
    res += heating_AC * (getCch() / 3.0);
    res += heating_AC * (getCac() / 3.0);
    // Update time string to catch night or day becoming
    //currentTimeString=MSNet::getInstance()->makeTimeString(
    // MSNet::getInstance()->getCurrentTimeStep());
    // Lights
    //if(MSNet::getInstance()->isNight(currentTimeString)||
    //   (!MSNet::getInstance()->isNight(currentTimeString))&&rand()%20==0)
    res += cockpit_lights * getCcl();
    res += head_lights * (getCvhl() / 3.0);
    res += fog_lights * getCffl();
  //}
  //else
  //{
  //  if(rand()%10==0)
  //    res+=radio;
  //  if(rand()%10==0)
  //    res+=heating_AC;
  //  if(rand()%10==0)
  //    res+=head_lights+fog_lights+cockpit_lights;
  //}
  // remove currentTimeString?
  return res;
}

void MSDevice_FEV::updateAuxiliaryServicesOnOffStatus()
{
  std::string currentTimeString;
  OptionsCont &oc = OptionsCont::getOptions();
  if(oc.getSafeBool("rlas") || oc.getSafeBool("ras"))
  {
    // Radio
    if(rand() % 5 == 0)
      setCca(1);
    // Heating and air conditioner
    if(MSNet::getInstance()->isWinter())
    {
      if(MSNet::getInstance()->getCity() == "munchen")
      {
        setCch(3);
        setCac(0);
      }
      else if(rand() % 2 == 0)
      {
        setCch(3);
        setCac(0);
      }
      else
      {
        setCch(2);
        setCac(0);
      }
    }        
    else if(MSNet::getInstance()->isSummer())
    {
      if(MSNet::getInstance()->getCity() == "cambiano")
      {
        if(rand() % 2 == 0)
        {
          setCch(0);
          setCac(3);
        }
        else
        {
          setCch(0);
          setCac(2);
        }
      }
      else if(MSNet::getInstance()->getCity() == "munchen")
      {
        if(rand() % 2 == 0)
        {
          setCch(0);
          setCac(2);
        }
        else
        {
          setCch(0);
          setCac(1);
        }
      }
    }
    else
    {
      if(rand() % 3 == 0)
      {
        setCch(1);
        setCac(0);
      }
      else if(rand() % 3 == 0)
      {
        setCch(0);
        setCac(1);
      }
      else
      {
        setCch(0);
        setCac(0);
      }
    }
    // Update time string to catch night or day becoming
    currentTimeString = MSNet::getInstance()->makeTimeString(
     MSNet::getInstance()->getCurrentTimeStep());
    // Lights
    if(MSNet::getInstance()->isNight(currentTimeString))
    {
      // Night
      // Headlights
      if(rand() % 10 == 0)
        setCvhl(3);
      else if(rand() % 80 == 0)
        setCvhl(1);
      else if(rand() % 90 == 0)
        setCvhl(0);
      else
        setCvhl(2);
      // Fog lights
      if(rand() % 20 == 0)
        setCffl(1);
      // Cockpit lights
      if(rand() % 50 == 0)
        setCcl(1);
    }
    else
    {
      // Daylight
      // Headlights
      if(rand() % 20 == 0)
        setCvhl(2);
      else if(rand() % 20 == 0)
        setCvhl(1);
      else
        setCvhl(0);
      // Fog lights
      if(MSNet::getInstance()->getCity() == "cambiano")
      {
        if(rand() % 5 == 0)
          setCffl(1);
        else
          setCffl(0);
      }
      else if(MSNet::getInstance()->getCity() == "munchen")
      {
        if(rand() % 5 == 0)
          setCffl(1);
        else
          setCffl(0);
      }
      // Cockpit lights
      setCcl(0);
    }
  }
  /*
  else
  {
    if(rand() % 10 == 0)
      setCac(1);
    else
      setCac(0);
    if(rand() % 10 == 0)
    {
      if(rand() % 3 == 0)
        setCac(1);
      else if(rand() % 3 == 0)
        setCac(3);
      else
        setCac(2);
    }
    else
      set
      
      seres+=heating_AC;
    }
    else
      setCch(0);
    if(rand
    
    if(rand() % 10 == 0)
      res+=head_lights+fog_lights+cockpit_lights;
  }
  */
  //remove currentTimeString?
  //return res;
}

void MSDevice_FEV::generateOutput() const
{
  /* THIS IS CALLED AT THE END OF THE SIMULATION */
  /*
  if(OptionsCont::getOptions().isSet("tripinfo-fev")){
    OutputDevice& os = OutputDevice::getDeviceByOption("tripinfo-fev");
    (os.openTag("fev") <<
    " ID=\"" << myID <<
    "\" timestep=\"" << OutputDevice::realString(MSNet::getInstance()->getCurrentTimeStep(), 6) <<
    "\" speedC-1=\"" << OutputDevice::realString(my_last_speed, 6) <<
    "\" speedC=\"" << OutputDevice::realString(my_curr_speed, 6) <<
    "\" Charge_removedC-1=\"" << OutputDevice::realString(my_last_charge_rem, 6) <<
    "\" Charge_removedC=\"" << OutputDevice::realString(my_curr_charge_rem, 6) <<
    "\" DoDC-1=\"" << OutputDevice::realString(my_last_DoD, 6) <<
    "\" DoDC=\"" << OutputDevice::realString(my_curr_DoD, 6) <<
    "\" myPositionOnLane=\"" << OutputDevice::realString(myHolder.getPositionOnLane(),6) << 
    "\" myTraveledDistance=\"" << OutputDevice::realString(dynamic_cast < MSVehicle& > (myHolder).getCurrentTraveledDistance(),6) <<
    "\"").closeTag(true);
  }
  */ 
  ;
}

double MSDevice_FEV::getMyCurrChargeRem() const
{
  return my_curr_charge_rem;
}

double MSDevice_FEV::getMyCurrDoD() const
{
  return my_curr_DoD;
}

double MSDevice_FEV::getMyCurrSpeed() const
{
  return my_curr_speed;
}

double MSDevice_FEV::getMyLastChargeRem() const
{
  return my_last_charge_rem;
}

double MSDevice_FEV::getMyLastDoD() const
{
  return my_last_DoD;
}

double MSDevice_FEV::getMyLastSpeed() const
{
  return my_last_speed;
}

/*
void MSDevice_FEV::setMyCurrChargeRem(double my_curr_charge_rem)
{
  this->my_curr_charge_rem = my_curr_charge_rem;
}
*/

void MSDevice_FEV::setMyCurrDoD(double my_curr_DoD)
{
  this->my_curr_DoD = my_curr_DoD;
}

void MSDevice_FEV::setMyCurrSpeed(double my_curr_speed)
{
  this->my_curr_speed = my_curr_speed;
}

/*
void MSDevice_FEV::setMyLastChargeRem(double my_last_charge_rem)
{
  this->my_last_charge_rem = my_last_charge_rem;
}
*/

void MSDevice_FEV::setMyLastDoD(double my_last_DoD)
{
  this->my_last_DoD = my_last_DoD;
}

void MSDevice_FEV::setMyLastSpeed(double my_last_speed)
{
  this->my_last_speed = my_last_speed;
}

double MSDevice_FEV::getMySoH() const
{
  return my_SOH;
}

/*
void MSDevice_FEV::setMySoH(double my_SOH)
{
  this->my_SOH = my_SOH;
}
*/

double MSDevice_FEV::getMyCurrE() const {
  return my_curr_E;
}

void MSDevice_FEV::setMyCurrE(double my_curr_E)
{
  this->my_curr_E = my_curr_E;
}
