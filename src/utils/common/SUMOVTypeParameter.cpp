/****************************************************************************/
/// @file    SUMOVTypeParameter.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    10.09.2009
/// @version $Id: SUMOVTypeParameter.cpp 11671 2012-01-07 20:14:30Z behrisch $
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

// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "additionalConfig.h"
#include <algorithm>
//#include <stdio.h>
//#include <string>
#include <cstring>
//#include <string.h>
#include "scenload/VehicleToTrack.h"
#include "utils/common/SUMOVTypeParameter.h"
#include "utils/common/ToString.h"
#include "utils/common/TplConvert.h"
#include "utils/common/MsgHandler.h"
#include "utils/iodevices/OutputDevice.h"
#include "utils/options/OptionsCont.h"
#include "utils/xml/SUMOXMLDefinitions.h"

#ifdef CHECK_MEMORY_LEAKS
#include "foreign/nvwa/debug_new.h"
#endif // CHECK_MEMORY_LEAKS

// ===========================================================================
// member method definitions
// ===========================================================================

#define FAAM_ECOMILE_LIGHT_CONSUMPTION 100

SUMOVTypeParameter::SUMOVTypeParameter(const std::string defVehTypeId)
#ifdef VEH_FAAM_ECOMILE
 :id(defVehTypeId), length(DEFAULT_VEH_LENGTH), minGap(DEFAULT_VEH_MINGAP),
  maxSpeed(80),
  defaultProbability(DEFAULT_VEH_PROB),
  speedFactor(DEFAULT_VEH_SPEEDFACTOR), speedDev(DEFAULT_VEH_SPEEDDEV),
  emissionClass(SVE_UNKNOWN), color(RGBColor::DEFAULT_COLOR),
  vehicleClass(SVC_UNKNOWN), width(DEFAULT_VEH_GUIWIDTH),
  shape(DEFAULT_VEH_SHAPE), cfModel(DEFAULT_VEH_FOLLOW_MODEL),
  lcModel(DEFAULT_VEH_LANE_CHANGE_MODEL), setParameter(0), saved(false),
  onlyReferenced(false),
#ifdef CALIBRATION_TEST
  mass(1245 /* current actual weight of the calibration test */),
#else
  mass(1000.0 /* 900 + 2 50kg. people */),
#endif
  rolling_res(0.0048), air_dens(1.225),
  frontal_area(/* 1.89 */ 2.4),
  drag_coefficient(0.19),
  Gear_ratio(8/**4*//* 37.0 */), /* old 37 was 9.25*4 ? */
  Gear_sys_effc(0.95), regen_ratio(0.5), motor_sys_effc(0.95),
  cooper_losses(0.3), iron_losses(0.1),
  windage_losses(0.00001),
  constant_losses(600.0), power_accs(250.0),
  num_cells(/* 30 */ 52 /*156*/), 
  capacity(150.0 /* 7800.0 */), /* per battery? */ /* 7800 = 150 * 52 */
  time_charge(8.0 /* hours? (4 with double charger) */),
  celsius(28.0), humidity(43),
  // FIXME This should be read from the scenario...
  aux_mask("00000000"),
  light_mask("00000000"),
  vehicle_headlights_consumption(FAAM_ECOMILE_LIGHT_CONSUMPTION * 0.65),
  front_fog_lights_consumption(FAAM_ECOMILE_LIGHT_CONSUMPTION * 0.30),
  cockpit_lights_consumption(FAAM_ECOMILE_LIGHT_CONSUMPTION * 0.05),
  radio_consumption(0 /* 52 */),
  heatingAC_consumption(489)
#endif
#ifdef VEH_HIB_ORIGINAL
 :id(defVehTypeId), length(DEFAULT_VEH_LENGTH), minGap(DEFAULT_VEH_MINGAP),
  maxSpeed(DEFAULT_VEH_MAXSPEED),
  defaultProbability(DEFAULT_VEH_PROB),
  speedFactor(DEFAULT_VEH_SPEEDFACTOR), speedDev(DEFAULT_VEH_SPEEDDEV),
  emissionClass(SVE_UNKNOWN), color(RGBColor::DEFAULT_COLOR),
  vehicleClass(SVC_UNKNOWN), width(DEFAULT_VEH_GUIWIDTH),
  shape(DEFAULT_VEH_SHAPE), cfModel(DEFAULT_VEH_FOLLOW_MODEL),
  lcModel(DEFAULT_VEH_LANE_CHANGE_MODEL), setParameter(0), saved(false),
  onlyReferenced(false),
  mass(1540.0),
  rolling_res(0.0048), air_dens(1.225),
  frontal_area(1.89),
  drag_coefficient(0.19),
  Gear_ratio(37.0),
  Gear_sys_effc(0.95), regen_ratio(0.5), motor_sys_effc(0.95),
  cooper_losses(0.3), iron_losses(0.1),
  windage_losses(0.000005),
  constant_losses(600.0), power_accs(250.0),
  num_cells(156.0), 
  capacity(60.0),
  time_charge(10.0),
  celsius(15.0), humidity(25.0),
  aux_mask("00000000"),
  light_mask("00000000"),
  vehicle_headlights_consumption(316.0 * 0.65),
  front_fog_lights_consumption(316.0 * 0.30),
  cockpit_lights_consumption(316 * 0.05),
  radio_consumption(52.0),
  heatingAC_consumption(489.0)
#endif
{
  OptionsCont &oc=OptionsCont::getOptions();
  if(oc.getAnyVerbosity()>2)
    std::cout<<"----> SUMOVTypeParameter::SUMOVTypeParameter(const std::string "
     <<"defVehTypeId{"<<defVehTypeId<<"})\n"<<std::endl;
  /*
  http://en.wikipedia.org/wiki/Density_of_air
  Rspecific is the specific gas constant for the dry air
  pressure = pzero*((1-((L*h)/Tzero))^(g*M)/(R*L))
  air_dens = pressure / (Rspecific * celsius);
  */
  /* Is this a vehicle to track? */  
  std::vector<VehicleToTrack*>* vehiclesList;
  vehiclesList=VehicleToTrack::getVehiclesToTrackWithCharacteristicsList(); // TODO TASK FIXME ALERT DANGER DEPRECATED REACTIVAR ESTO
  if(vehiclesList)
  {
    bool found=false;
    int i=0;
    std::cout<<"vehiclesList->size(){"<<vehiclesList->size()<<"}"<<std::endl;
    while(!found&&i<vehiclesList->size())
    {
      //if(!strcmp(vehiclesList->at(i)->getVehicleType().c_str(),defVehTypeId.c_str()))
      if(vehiclesList->at(i)->getVehicleType()!=defVehTypeId)
      {
        //if(!strcmp(vehiclesList->at(i)->getIdentifier()
        // Particular makes of vehicle
	std::cout<<"[SAM] TYPE at("<<i<<"): "<<vehiclesList->at(i)->getVehicleType().c_str()<<std::endl;
        if(!strcmp(vehiclesList->at(i)->getVehicleType().c_str(),"ecomile"))
        {
          std::cout<<"!strcmp(vehiclesList->at("<<i<<")->getVehicleType().c_str(),\"ecomile\")"<<std::endl;
          setMaxSpeed(80);
#ifdef CALIBRATION_TEST
          setMass(1245 /* current actual weight of the calibration test */);
#else
          setMass(1080.0 /* 900 + 2 90kg. people */);
#endif
          setRollingRes(0.0048);
          setAirDens(1.25);
          setFrontalArea(/* 1.89 */ 2.4);
          setDragCoefficient(0.19);
          setGearRatio(8/**4*//* 37.0 */); /* old 37 was 9.25*4 ? */
          setGearSysEffc(0.95);
          setRegenRatio(0.5);
          setMotorSysEffc(0.95);
          setCopperLosses(0.3);
          setIronLosses(0.1);
          setWindageLosses(0.00001);
          setConstantLosses(600.0);
          setPowerAccs(250.0);
          setNumCells(/* 30 */ 52 /*156*/);
          setCapacity(150.0 /* 7800.0 */); /* per battery? */ /* 7800 = 150 * 52 */
          setTimeCharge(8.0 /* hours? (4 with double charger) */);
          setCelsius(oc.getFloat("fiet"));  /* 28.0 */
          setHumidity(oc.getFloat("fieh")); /* 43 */
          // FIXME This should be read from the scenario...
          setAuxMask("00000000");
          setLightMask("00000000");
          setVehicleHeadlightsConsumption(FAAM_ECOMILE_LIGHT_CONSUMPTION * 0.65);
          setFrontFogLightsConsumption(FAAM_ECOMILE_LIGHT_CONSUMPTION * 0.30);
          setCockpitLightsConsumption(FAAM_ECOMILE_LIGHT_CONSUMPTION * 0.05);
          setRadioConsumption(0 /* 52 */);
          setHeatingACConsumption(489);
          found=true;
        }
        else if(!strcmp(vehiclesList->at(i)->getVehicleType().c_str(),"nido"))
        {
          std::cout<<"!strcmp(vehiclesList->at(i)->getVehicleType().c_str(),\"nido\")"<<std::endl;
          // Por omisión, mismos parametros que el EV1
          setMass(1000);
          //   It is known because of calibration tests that total battery
          // capacity is 28800, let us hope Ah (and not Wh). Not known the 
          // number of cells in Nido. Let us take, by default, the same battery
          // capacity per cell as with the EcoMile (150Ah). This way we are
          // guessing a number of cells of 28800/150=192
          setNumCells(28800/150);
          setCapacity(150);
          found=true;
        }
        else if(!strcmp(vehiclesList->at(i)->getVehicleType().c_str(),"miev"))
        {
          std::cout<<"!strcmp(vehiclesList->at(i)->getVehicleType().c_str(),\"miev\")"<<std::endl;
          setMass(1250);
          setNumCells(88);
          //setCapacity(48); /* 48 Ah (16 kWh) */
          //   According to common wellknown documentation, MiEV has 88 cells.
          // According to the calibration tests made by TEM and SOFT, total
          // capacity is 21500... (it's documented in some calibration tests as
          // Wh, but could possibly be really Ah...)
          // let each batt's capacity be then 21500/88=244.31818
          setCapacity(21500/88);
          setTimeCharge(6);
/*
#ifdef MIEV_ISTAMBUL_CALIBRATION
          setCelsius(11);
          setHumidity(97);          
#endif
*/
          found=true;
        }
        else if(!strcmp(vehiclesList->at(i)->getVehicleType().c_str(),"temsa"))
        {
          std::cout<<"!strcmp(vehiclesList->at(i)->getVehicleType().c_str(),\"temsa\")"<<std::endl;
          setFrontalArea(2.5*3);
          setMass(3500); // http://irisbusiveco.com/prod/euromidi-cc100.html
          found=true;
        }
        else if(!strcmp(vehiclesList->at(i)->getVehicleType().c_str(),"ducato"))
        {
          std::cout<<"!strcmp(vehiclesList->at(i)->getVehicleType().c_str(),\"ducato\")"<<std::endl;
	  // (smendez) Aerodynamic coeff??
	  setMass(2570);
	  setNumCells(432);
	  // setCapacity(118.45);
	  setNominalCapacity(118.45);
	  setTimeCharge(5);
	  setGearSysEffc(0.95);
	  setMotorSysEffc(0.90);
	  setRegenRatio(0.3);
	  
	  setHeatingACConsumption(1200);
	  found=true;
	}
        else if(!strcmp(vehiclesList->at(i)->getVehicleType().c_str(),"comarth"))
        {
          std::cout<<"!strcmp(vehiclesList->at(i)->getVehicleType().c_str(),\"comarth\")"<<std::endl;
	  // (smendez)
	  setMass(792);
	  setNominalCapacity(200);
	  setNumCells(23);
	  setTimeCharge(5);
	  setFrontalArea(1.75);
	  setRegenRatio(10.5);
	  found=true;
	}
        else if(!strcmp(vehiclesList->at(i)->getVehicleType().c_str(),"transit"))
        {
	  // (smendez)
	  setMass(1790.78);
	  setNominalCapacity(82);
	  setNumCells(192);
	  setFrontalArea(3.617);
	  setGearSysEffc(0.97);
	  setRegenRatio(8.28);
	  setTimeCharge(7);
	  found=true;
	}
        else if(!strcmp(vehiclesList->at(i)->getVehicleType().c_str(),"urbino"))
        {
          std::cout<<"!strcmp(vehiclesList->at(i)->getVehicleType().c_str(),\"urbino\")"<<std::endl;
	  // (smendez) -- Aerodynamic coeff??
	  setMass(12930);
	  // setCapacity(360);
	  setNominalCapacity(360);
	  setNumCells(624);
	  setTimeCharge(3);
	  setFrontalArea(7.274);
	  setMotorSysEffc(0.91);
	  setRollingRes(0.00989);
	  
	  setRadioConsumption(20);
	  setCockpitLightsConsumption(304);
	  setVehicleHeadlightsConsumption(110);
	  setHeatingACConsumption(34000);
	  found=true;
	}
        // Generic vehicles
        else if(!strcmp(vehiclesList->at(i)->getVehicleType().c_str(),"delivery"))
        {
          //std::cout<<"delivery"<<std::endl;
          // TODO PARAMETRIZATION ??? (como la EcoMile)
          found=true;
        }
        else if(!strcmp(vehiclesList->at(i)->getVehicleType().c_str(),"bus"))
        {
          //std::cout<<"bus"<<std::endl;
          setFrontalArea(2.5*3);
          setMass(3500);
          found=true;
        }    
        else if(!strcmp(vehiclesList->at(i)->getVehicleType().c_str(),"evehicle"))
        {
          /* Electric vehicle or EMPTY vehicle? */
          //std::cout<<"evehicle"<<std::endl;
          // TODO PARAMETRIZATION ??? ???
          found=true;
        }
        else
        {
          /*   Generic particular vehicle. Can be electric or not, attending to
           * its name prefix */
          found=true; // FIXME???
          ;
        }
      }
      else
        i++;
    }
  }
  else
    if(oc.getAnyVerbosity()>2)
      std::cout<<"[999] SUMOVTypeParameter::SUMOVTypeParameter(const std::string "
       <<"defVehTypeId{"<<defVehTypeId<<"})\n !vehiclesList"<<std::endl;
  updateAirDens(getCelsius(),getHumidity());
  // Save nominal capacity before updating it depending on envtemp
  //SUMOReal foaoo=getCapacity();
  //setNominalCapacity(foaoo);
  //setNominalCapacity(getCapacity()); // Seems not to work properly?
  setNominalCapacity(this->getCapacity());
  // Update capacity value as depending on envtemp
  updateCapacity(getCelsius());
}

void SUMOVTypeParameter::write(OutputDevice& dev) const {
	if (onlyReferenced) {
		return;
	}
	dev.openTag(SUMO_TAG_VTYPE);
	dev.writeAttr(SUMO_ATTR_ID, id);
	if (wasSet(VTYPEPARS_LENGTH_SET)) {
		dev.writeAttr(SUMO_ATTR_LENGTH, length);
	}
	if (wasSet(VTYPEPARS_MINGAP_SET)) {
		dev.writeAttr(SUMO_ATTR_MINGAP, minGap);
	}
	if (wasSet(VTYPEPARS_MAXSPEED_SET)) {
		dev.writeAttr(SUMO_ATTR_MAXSPEED, maxSpeed);
	}
	if (wasSet(VTYPEPARS_PROBABILITY_SET)) {
		dev.writeAttr(SUMO_ATTR_PROB, defaultProbability);
	}
	if (wasSet(VTYPEPARS_SPEEDFACTOR_SET)) {
		dev.writeAttr(SUMO_ATTR_SPEEDFACTOR, speedFactor);
	}
	if (wasSet(VTYPEPARS_SPEEDDEVIATION_SET)) {
		dev.writeAttr(SUMO_ATTR_SPEEDDEV, speedDev);
	}
	if (wasSet(VTYPEPARS_VEHICLECLASS_SET)) {
		dev.writeAttr(SUMO_ATTR_VCLASS, toString(vehicleClass));
	}
	if (wasSet(VTYPEPARS_EMISSIONCLASS_SET)) {
		dev.writeAttr(SUMO_ATTR_EMISSIONCLASS,
				getVehicleEmissionTypeName(emissionClass));
	}
	if (wasSet(VTYPEPARS_SHAPE_SET)) {
		dev.writeAttr(SUMO_ATTR_GUISHAPE, getVehicleShapeName(shape));
	}
	if (wasSet(VTYPEPARS_WIDTH_SET)) {
		dev.writeAttr(SUMO_ATTR_GUIWIDTH, width);
	}
	if (wasSet(VTYPEPARS_COLOR_SET)) {
		dev.writeAttr(SUMO_ATTR_COLOR, color);
	}

	if (cfParameter.size() != 0) {
		dev << ">\n";
		dev.openTag(cfModel);
		std::vector<SumoXMLAttr> attrs;
		for (CFParams::const_iterator i = cfParameter.begin();
				i != cfParameter.end(); ++i) {
			attrs.push_back(i->first);
		}
		std::sort(attrs.begin(), attrs.end());
		for (std::vector<SumoXMLAttr>::const_iterator i = attrs.begin();
				i != attrs.end(); ++i) {
			dev.writeAttr(*i, cfParameter.find(*i)->second);
		}
		dev.closeTag(true);
		dev.closeTag();
	} else {
		dev.closeTag(true);
	}
}

void SUMOVTypeParameter::setMaxSpeed(SUMOReal ms){
  maxSpeed=ms;
}

SUMOReal SUMOVTypeParameter::getMass() const {
  return mass;
}

void SUMOVTypeParameter::setMass(SUMOReal m) {
  mass=m;
}
  
void SUMOVTypeParameter::setRollingRes(SUMOReal rr){
  rolling_res=rr;
}
    
void SUMOVTypeParameter::updateAirDens(SUMOReal et,SUMOReal eh){
  SUMOReal ad1,ad2;
  ad1=1.30-.0042*et;
  if(et>=30){
    ad2=ad1-.02*(eh/100.);
  }else if(et>=20){
    ad2=ad1-.01*(eh/100.);
  }else if(et>=10){
    ad2=ad1-.006*(eh/100.);
  }else{
    ad2=ad1-.004*(eh/100.);
  }
  air_dens=ad2;
}

void SUMOVTypeParameter::setAirDens(SUMOReal ad){
  air_dens=ad;
}

void SUMOVTypeParameter::setFrontalArea(SUMOReal fa){
  frontal_area=fa;
}
    
void SUMOVTypeParameter::setDragCoefficient(SUMOReal dc){
  drag_coefficient=dc;
}
    
void SUMOVTypeParameter::setGearRatio(SUMOReal gr){
  Gear_ratio=gr;
}
        
void SUMOVTypeParameter::setGearSysEffc(SUMOReal gse){
  Gear_sys_effc=gse;
}

void SUMOVTypeParameter::setRegenRatio(SUMOReal rr){
  regen_ratio=rr;
}
        
void SUMOVTypeParameter::setMotorSysEffc(SUMOReal mse){
  motor_sys_effc=mse;
}
        
void SUMOVTypeParameter::setCopperLosses(SUMOReal cl){
  cooper_losses=cl;
}
        
void SUMOVTypeParameter::setIronLosses(SUMOReal il){
  iron_losses=il;
}
        
void SUMOVTypeParameter::setWindageLosses(SUMOReal wl){
  windage_losses=wl;
}
        
void SUMOVTypeParameter::setConstantLosses(SUMOReal cl){
  constant_losses=cl;
}
        
void SUMOVTypeParameter::setPowerAccs(SUMOReal pa){
  power_accs=pa;
}

void SUMOVTypeParameter::setNumCells(SUMOReal nc){
  num_cells=nc;
}

void SUMOVTypeParameter::updateCapacity(SUMOReal et)
{
  OptionsCont &oc=OptionsCont::getOptions();
  if(oc.getSimulationVerbosity()>1)
    std::cout<<"----> void SUMOVTypeParameter::updateCapacity(SUMOReal et)"<<std::endl;
  if(oc.getSimulationVerbosity()>2)
    std::cout<<" getCapacity(){"<<getCapacity()<<"}"<<std::endl;
  if(getCelsius()>50)
    setCapacity(getNominalCapacity()*1.12);
  else if(getCelsius()>45)
    setCapacity(getNominalCapacity()*1.1);
  else if(getCelsius()>40)
    setCapacity(getNominalCapacity()*1.08);
  else if(getCelsius()>35)
    setCapacity(getNominalCapacity()*1.06);  
  else if(getCelsius()>30)
    setCapacity(getNominalCapacity()*1.03);  
  else if(getCelsius()>25)
    //setCapacity(getNominalCapacity()*1);  
    ;
  else if(getCelsius()>20)
    setCapacity(getNominalCapacity()*.96);  
  else if(getCelsius()>15)
    setCapacity(getNominalCapacity()*.93);  
  else if(getCelsius()>10)
    setCapacity(getNominalCapacity()*.89);  
  else if(getCelsius()>5)
    setCapacity(getNominalCapacity()*.85);  
  else if(getCelsius()>0)
    setCapacity(getNominalCapacity()*.8);  
  else if(getCelsius()>-5)
    setCapacity(getNominalCapacity()*.75);  
  else if(getCelsius()>-10)
    setCapacity(getNominalCapacity()*.7);  
  else if(getCelsius()>-15)
    setCapacity(getNominalCapacity()*.65);  
  else if(getCelsius()>-20)
    setCapacity(getNominalCapacity()*.6);  
  else if(getCelsius()>-27)
    setCapacity(getNominalCapacity()*.5);
  if(oc.getSimulationVerbosity()>2)
    std::cout<<" getCapacity(){"<<getCapacity()<<"}"<<std::endl;
}
    
void SUMOVTypeParameter::setCapacity(SUMOReal c)
{
  capacity=c;
}

SUMOReal SUMOVTypeParameter::getCapacity()const
{
  return capacity;
}

void SUMOVTypeParameter::setNominalCapacity(SUMOReal nc)
{
  nominalCapacity=nc;
}

SUMOReal SUMOVTypeParameter::getNominalCapacity()const
{
  return nominalCapacity;
}

void SUMOVTypeParameter::setTimeCharge(double time_charge) {
  this->time_charge = time_charge;
}

SUMOReal SUMOVTypeParameter::getTimeCharge()const
{
  return time_charge;
}

void SUMOVTypeParameter::setCelsius(SUMOReal c){
  celsius=c;
}

double SUMOVTypeParameter::getCelsius(){return celsius;}
        
void SUMOVTypeParameter::setHumidity(SUMOReal h){
  humidity=h;
}

double SUMOVTypeParameter::getHumidity(){return humidity;}
        
void SUMOVTypeParameter::setAuxMask(std::string am){
  aux_mask=am;
}
        
void SUMOVTypeParameter::setLightMask(std::string lm){
  light_mask=lm;
}

void SUMOVTypeParameter::setVehicleHeadlightsConsumption(SUMOReal vhlc){
  vehicle_headlights_consumption=vhlc;
}
 
void SUMOVTypeParameter::setFrontFogLightsConsumption(SUMOReal fflc){
  front_fog_lights_consumption=fflc;
}
  
void SUMOVTypeParameter::setCockpitLightsConsumption(SUMOReal clc){
  cockpit_lights_consumption=clc;
}
    
void SUMOVTypeParameter::setRadioConsumption(SUMOReal rc){
  radio_consumption=rc;
}
    
void SUMOVTypeParameter::setHeatingACConsumption(SUMOReal hacc){
  heatingAC_consumption=hacc;
}    
    