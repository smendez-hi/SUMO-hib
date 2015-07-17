/****************************************************************************/
/// @file    MSVehicleType.cpp
/// @author  Christian Roessel
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Thimor Bohn
/// @author  Michael Behrisch
/// @date    Tue, 06 Mar 2001
/// @version $Id: MSVehicleType.cpp 11671 2012-01-07 20:14:30Z behrisch $
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

// ===========================================================================
// included modules
// ===========================================================================
#include <cassert>
#include <cstring>
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif
#include <utils/iodevices/BinaryInputDevice.h>
#include <utils/common/FileHelpers.h>
#include <utils/common/RandHelper.h>
#include <utils/common/SUMOVTypeParameter.h>
#include <utils/options/OptionsCont.h>
#include "MSNet.h"
#include "cfmodels/MSCFModel_IDM.h"
#include "cfmodels/MSCFModel_Kerner.h"
#include "cfmodels/MSCFModel_Krauss.h"
#include "cfmodels/MSCFModel_KraussOrig1.h"
#include "cfmodels/MSCFModel_PWag2009.h"
#include "cfmodels/MSCFModel_Wiedemann.h"
#include "MSVehicleType.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
MSVehicleType::MSVehicleType
 (const std::string& id, SUMOReal length, SUMOReal minGap, SUMOReal maxSpeed,
  SUMOReal prob, SUMOReal speedFactor, SUMOReal speedDev,
  SUMOVehicleClass vclass, SUMOEmissionClass emissionClass, SUMOReal guiWidth,
  SUMOVehicleShape shape, const std::string& lcModel, const RGBColor& c)
 :myID(id), myLength(length), myMinGap(minGap), myMaxSpeed(maxSpeed),
  myDefaultProbability(prob), mySpeedFactor(speedFactor), mySpeedDev(speedDev),
  myLaneChangeModel(lcModel), myEmissionClass(emissionClass), myColor(c),
  myVehicleClass(vclass), myWidth(guiWidth), myShape(shape), myOriginalType(0),
  mass(1540.0), rolling_res(0.0048), air_dens(1.25), frontal_area(1.89),
  drag_coefficient(0.19), Gear_ratio(37.0), Gear_sys_effc(0.95),
  regen_ratio(0.5), motor_sys_effc(0.95), cooper_losses(0.3), iron_losses(0.1),
  windage_losses(0.000005), constant_losses(600.0), power_accs(250.0),
  num_cells(156.0), capacity(/*60.*/30.0), time_charge(10), celsius(15.0),
  humidity(25), aux_mask("00000000"), light_mask("00000000"),
  vehicle_headlights_consumption(316.0 * 0.65),
  front_fog_lights_consumption(316.0 * 0.30),
  cockpit_lights_consumption(316 * 0.05), radio_consumption(52),
  heatingAC_consumption(489),oc(OptionsCont::getOptions())
{
  if(oc.getAnyVerbosity()>1)
    std::cout<<"MSVehicleType::MSVehicleType(const std::string& id{"<<id<<"}..."
     <<"(bigConstructorNonFevAlike))"<<std::endl;
  assert(myLength > 0);
  assert(getMaxSpeed() > 0);
}

MSVehicleType::MSVehicleType
 (const std::string& id, SUMOReal length, SUMOReal minGap, SUMOReal maxSpeed,
  SUMOReal prob, SUMOReal speedFactor, SUMOReal speedDev,
  SUMOVehicleClass vclass, SUMOEmissionClass emissionClass, SUMOReal guiWidth,
  SUMOVehicleShape shape, const std::string& lcModel, const RGBColor& c,
  SUMOReal m, SUMOReal r_res, SUMOReal air_r, SUMOReal frontal, SUMOReal drag_c,
  SUMOReal gear_r, SUMOReal gear_s_e, SUMOReal reg_rat, SUMOReal motor_s_e,
  SUMOReal coop_l, SUMOReal iron_l, SUMOReal windage_l, SUMOReal constant_l,
  SUMOReal power_a, SUMOReal num_c, SUMOReal capac, SUMOReal time,
  SUMOReal temp, SUMOReal hum, std::string aux_m, std::string ligth_m,
  SUMOReal vehicle_lights, SUMOReal front_lights, SUMOReal cockpit_lights,
  SUMOReal radio, SUMOReal heating)
 :myID(id), myLength(length), myMinGap(minGap), myMaxSpeed(maxSpeed),
  myDefaultProbability(prob), mySpeedFactor(speedFactor), mySpeedDev(speedDev),
  myLaneChangeModel(lcModel), myEmissionClass(emissionClass), myColor(c),
  myVehicleClass(vclass), myWidth(guiWidth), myShape(shape), myOriginalType(0),
  mass(m), rolling_res(r_res), air_dens(air_r), frontal_area(frontal),
  drag_coefficient(drag_c), Gear_ratio(gear_r), Gear_sys_effc(gear_s_e),
  regen_ratio(reg_rat), motor_sys_effc(motor_s_e), cooper_losses(coop_l),
  iron_losses(iron_l), windage_losses(windage_l), constant_losses(constant_l),
  power_accs(power_a), num_cells(num_c), capacity(capac), time_charge(time),
  celsius(temp), humidity(hum), aux_mask(aux_m), light_mask(ligth_m),
  vehicle_headlights_consumption(vehicle_lights),
  front_fog_lights_consumption(front_lights),
  cockpit_lights_consumption(cockpit_lights), radio_consumption(radio), 
  heatingAC_consumption(heating),oc(OptionsCont::getOptions())
{
  if(oc.getAnyVerbosity()>1)
    std::cout<<"MSVehicleType::MSVehicleType(const std::string& id{"<<id<<"}..."
     <<"(biggerConstructorFevAlike))"<<std::endl;
  assert(myLength>0);
  assert(getMaxSpeed()>0);
  /*
  if(!strcmp(id.c_str(),"ecomile"))
  {
    setMass(1245);
    // ...
  }
  else if(!strcmp(id.c_str(),"nido"))
  {
    setMass(900); // better be 1000
    // ...
  }
  else if(!strcmp(id.c_str(),"miev"))
  {
    setMass(1400); // better be 1250
    // ...
  }
  */
  setNominalCapacity(getCapacity());
}

/*
MSVehicleType::MSVehicleType
 (MSVehicleType *msVType, VehicleToTrack *vtt)
 :myID(msVType->getID()), myLength(msVType->getLength()), myMinGap(msVType->getMinGap()), myMaxSpeed(msVType->getMaxSpeed()),
  myDefaultProbability(msVType->getDefaultProbability()), mySpeedFactor(msVType->getSpeedFactor()), mySpeedDev(msVType->getSpeedDeviation()),
  myLaneChangeModel(msVType->getL), myEmissionClass(emissionClass), myColor(c),
  myVehicleClass(vclass), myWidth(guiWidth), myShape(shape), myOriginalType(0),
  mass(m), rolling_res(r_res), air_dens(air_r), frontal_area(frontal),
  drag_coefficient(drag_c), Gear_ratio(gear_r), Gear_sys_effc(gear_s_e),
  regen_ratio(reg_rat), motor_sys_effc(motor_s_e), cooper_losses(coop_l),
  iron_losses(iron_l), windage_losses(windage_l), constant_losses(constant_l),
  power_accs(power_a), num_cells(num_c), capacity(capac), time_charge(time),
  celsius(temp), humidity(hum), aux_mask(aux_m), light_mask(ligth_m),
  vehicle_headlights_consumption(vehicle_lights),
  front_fog_lights_consumption(front_lights),
  cockpit_lights_consumption(cockpit_lights), radio_consumption(radio), 
  heatingAC_consumption(heating)
{
  
}
*/

MSVehicleType::~MSVehicleType() {
    delete myCarFollowModel;
}

void MSVehicleType::saveState(std::ostream& os) {
    FileHelpers::writeString(os, myID);
    FileHelpers::writeFloat(os, myLength);
    FileHelpers::writeFloat(os, myMinGap);
    FileHelpers::writeFloat(os, getMaxSpeed());
    FileHelpers::writeInt(os, (int) myVehicleClass);
    FileHelpers::writeInt(os, (int) myEmissionClass);
    FileHelpers::writeInt(os, (int) myShape);
    FileHelpers::writeFloat(os, myWidth);
    FileHelpers::writeFloat(os, myDefaultProbability);
    FileHelpers::writeFloat(os, mySpeedFactor);
    FileHelpers::writeFloat(os, mySpeedDev);
    FileHelpers::writeFloat(os, myColor.red());
    FileHelpers::writeFloat(os, myColor.green());
    FileHelpers::writeFloat(os, myColor.blue());
    FileHelpers::writeInt(os, myCarFollowModel->getModelID());
    FileHelpers::writeString(os, myLaneChangeModel);
    //myCarFollowModel->saveState(os);
}

// ------------ Setter methods
void MSVehicleType::setLength(const SUMOReal& length) {
    assert(myOriginalType != 0);
    if (length < 0) {
        myLength = myOriginalType->myLength;
    } else {
        myLength = length;
    }
}

void MSVehicleType::setMinGap(const SUMOReal& minGap) {
    assert(myOriginalType != 0);
    if (minGap < 0) {
        myMinGap = myOriginalType->myMinGap;
    } else {
        myMinGap = minGap;
    }
}

void MSVehicleType::setMaxSpeed(const SUMOReal& maxSpeed) {
	assert(myOriginalType != 0);
	if (maxSpeed < 0) {
		myMaxSpeed = myOriginalType->myMaxSpeed;
	} else {
		myMaxSpeed = maxSpeed;
	}
}

void MSVehicleType::setVClass(SUMOVehicleClass vclass) {
	myVehicleClass = vclass;
}

void MSVehicleType::setDefaultProbability(const SUMOReal& prob) {
	assert(myOriginalType != 0);
	if (prob < 0) {
		myDefaultProbability = myOriginalType->myDefaultProbability;
	} else {
		myDefaultProbability = prob;
	}
}

void MSVehicleType::setSpeedFactor(const SUMOReal& factor) {
	assert(myOriginalType != 0);
	if (factor < 0) {
		mySpeedFactor = myOriginalType->mySpeedFactor;
	} else {
		mySpeedFactor = factor;
	}
}

void MSVehicleType::setSpeedDeviation(const SUMOReal& dev) {
	assert(myOriginalType != 0);
	if (dev < 0) {
		mySpeedDev = myOriginalType->mySpeedDev;
	} else {
		mySpeedDev = dev;
	}
}

void MSVehicleType::setEmissionClass(SUMOEmissionClass eclass) {
	myEmissionClass = eclass;
}

void MSVehicleType::setColor(const RGBColor& color) {
	myColor = color;
}

void MSVehicleType::setWidth(const SUMOReal& width) {
	assert(myOriginalType != 0);
	if (width < 0) {
		myWidth = myOriginalType->myWidth;
	} else {
		myWidth = width;
	}
}

void MSVehicleType::setShape(SUMOVehicleShape shape) {
	myShape = shape;
}

// ------------ Static methods for building vehicle types
SUMOReal MSVehicleType::get(const SUMOVTypeParameter::CFParams& from,
		SumoXMLAttr attr, SUMOReal defaultValue) {
	if (from.count(attr)) {
		return from.find(attr)->second;
	} else {
		return defaultValue;
	}
}

MSVehicleType* MSVehicleType::build(SUMOVTypeParameter& from)
{
  if(OptionsCont::getOptions().getSimulationVerbosity()>1||
    OptionsCont::getOptions().getLoadVerbosity()>1)
  {
    std::cout<<"MSVehicleType* MSVehicleType::build(...)"<<std::endl;
  }
  // Old was..
  /*
  MSVehicleType* vtype = new MSVehicleType(from.id, from.length, from.minGap,
      from.maxSpeed, from.defaultProbability, from.speedFactor,
      from.speedDev, from.vehicleClass, from.emissionClass, from.width,
      from.shape, from.lcModel, from.color, from.mass, from.rolling_res,
      from.air_dens, from.frontal_area, from.drag_coefficient,
      from.Gear_ratio, from.Gear_sys_effc, from.regen_ratio,
      from.motor_sys_effc, from.cooper_losses, from.iron_losses,
      from.windage_losses, from.constant_losses, from.power_accs,
      from.num_cells, from.capacity, from.time_charge, from.celsius,
      from.humidity, from.aux_mask, from.light_mask,
      from.vehicle_headligths_consumption, from.front_fog_ligths_consumption,
      from.cockpit_ligths_consumption, from.radio_consumption,
      from.heatingAC_consumption);
  */
  
  // This is the new try...
  MSVehicleType* vtype;
  if(from.id==DEFAULT_VTYPE_ID)
  {
    vtype = new MSVehicleType
     (from.id, from.length, from.minGap, from.maxSpeed, from.defaultProbability,
      from.speedFactor, from.speedDev, from.vehicleClass, from.emissionClass,
      from.width, from.shape, from.lcModel, from.color);
  }
  else // from.vehicleclass should be equal to DEFAULT_ELECVTYPE_ID
  {
    /* build the default FEV MSVehicleType instance and the particular
     * MSVehicleType instances for the vehicles to track */
    vtype = new MSVehicleType
     (from.id, from.length, from.minGap, from.maxSpeed, from.defaultProbability,
      from.speedFactor,
      from.speedDev, from.vehicleClass, from.emissionClass, from.width,
      from.shape, from.lcModel, from.color, from.mass, from.rolling_res,
      from.air_dens, from.frontal_area, from.drag_coefficient,
      from.Gear_ratio, from.Gear_sys_effc, from.regen_ratio,
      from.motor_sys_effc, from.cooper_losses, from.iron_losses,
      from.windage_losses, from.constant_losses, from.power_accs,
      from.num_cells, from.capacity, from.time_charge, from.celsius,
      from.humidity, from.aux_mask, from.light_mask,
      from.vehicle_headlights_consumption, from.front_fog_lights_consumption,
      from.cockpit_lights_consumption, from.radio_consumption,
      from.heatingAC_consumption);
  }

	MSCFModel* model = 0;
	switch (from.cfModel) {
	case SUMO_TAG_CF_IDM:
		model = new MSCFModel_IDM(vtype,
				get(from.cfParameter, SUMO_ATTR_ACCEL, DEFAULT_VEH_ACCEL),
				get(from.cfParameter, SUMO_ATTR_DECEL, DEFAULT_VEH_DECEL),
				get(from.cfParameter, SUMO_ATTR_TAU, DEFAULT_VEH_TAU),
				get(from.cfParameter, SUMO_ATTR_CF_IDM_DELTA, 4.),
				get(from.cfParameter, SUMO_ATTR_CF_IDM_STEPPING, .25));
		break;
	case SUMO_TAG_CF_IDMM:
		model = new MSCFModel_IDM(vtype,
				get(from.cfParameter, SUMO_ATTR_ACCEL, DEFAULT_VEH_ACCEL),
				get(from.cfParameter, SUMO_ATTR_DECEL, DEFAULT_VEH_DECEL),
				get(from.cfParameter, SUMO_ATTR_TAU, DEFAULT_VEH_TAU),
				get(from.cfParameter, SUMO_ATTR_CF_IDMM_ADAPT_FACTOR, 1.8),
				get(from.cfParameter, SUMO_ATTR_CF_IDMM_ADAPT_TIME, 600.),
				get(from.cfParameter, SUMO_ATTR_CF_IDM_STEPPING, .25));
		break;
	case SUMO_TAG_CF_BKERNER:
		model = new MSCFModel_Kerner(vtype,
				get(from.cfParameter, SUMO_ATTR_ACCEL, DEFAULT_VEH_ACCEL),
				get(from.cfParameter, SUMO_ATTR_DECEL, DEFAULT_VEH_DECEL),
				get(from.cfParameter, SUMO_ATTR_TAU, DEFAULT_VEH_TAU),
				get(from.cfParameter, SUMO_ATTR_K, .5),
				get(from.cfParameter, SUMO_ATTR_CF_KERNER_PHI, 5.));
		break;
	case SUMO_TAG_CF_KRAUSS_ORIG1:
		model = new MSCFModel_KraussOrig1(vtype,
				get(from.cfParameter, SUMO_ATTR_ACCEL, DEFAULT_VEH_ACCEL),
				get(from.cfParameter, SUMO_ATTR_DECEL, DEFAULT_VEH_DECEL),
				get(from.cfParameter, SUMO_ATTR_SIGMA, DEFAULT_VEH_SIGMA),
				get(from.cfParameter, SUMO_ATTR_TAU, DEFAULT_VEH_TAU));
		break;
	case SUMO_TAG_CF_PWAGNER2009:
		model = new MSCFModel_PWag2009(vtype,
				get(from.cfParameter, SUMO_ATTR_ACCEL, DEFAULT_VEH_ACCEL),
				get(from.cfParameter, SUMO_ATTR_DECEL, DEFAULT_VEH_DECEL),
				get(from.cfParameter, SUMO_ATTR_SIGMA, DEFAULT_VEH_SIGMA),
				get(from.cfParameter, SUMO_ATTR_TAU, DEFAULT_VEH_TAU),
				get(from.cfParameter, SUMO_ATTR_CF_PWAGNER2009_TAULAST, 0.3),
				get(from.cfParameter, SUMO_ATTR_CF_PWAGNER2009_APPROB, 0.5));
		break;
	case SUMO_TAG_CF_WIEDEMANN:
		model = new MSCFModel_Wiedemann(vtype,
				get(from.cfParameter, SUMO_ATTR_ACCEL, DEFAULT_VEH_ACCEL),
				get(from.cfParameter, SUMO_ATTR_DECEL, DEFAULT_VEH_DECEL),
				get(from.cfParameter, SUMO_ATTR_CF_WIEDEMANN_SECURITY, 0.5),
				get(from.cfParameter, SUMO_ATTR_CF_WIEDEMANN_ESTIMATION, 0.5));
		break;
	case SUMO_TAG_CF_KRAUSS:
	default:
		model = new MSCFModel_Krauss(vtype,
				get(from.cfParameter, SUMO_ATTR_ACCEL, DEFAULT_VEH_ACCEL),
				get(from.cfParameter, SUMO_ATTR_DECEL, DEFAULT_VEH_DECEL),
				get(from.cfParameter, SUMO_ATTR_SIGMA, DEFAULT_VEH_SIGMA),
				get(from.cfParameter, SUMO_ATTR_TAU, DEFAULT_VEH_TAU));
		break;
	}
	vtype->myCarFollowModel = model;
	return vtype;
}

MSVehicleType*
MSVehicleType::build(const std::string& id, const MSVehicleType* from) {
	MSVehicleType* vtype = new MSVehicleType(id, from->myLength, from->myMinGap,
			from->myMaxSpeed, from->myDefaultProbability, from->mySpeedFactor,
			from->mySpeedDev, from->myVehicleClass, from->myEmissionClass,
			from->myWidth, from->myShape, from->myLaneChangeModel,
			from->myColor);
	vtype->myCarFollowModel = from->myCarFollowModel->duplicate(vtype);
	vtype->myOriginalType =
			from->myOriginalType != 0 ? from->myOriginalType : from;
	return vtype;
}

SUMOReal MSVehicleType::getAirDens() const {
	return air_dens;
}

SUMOReal MSVehicleType::getCapacity()const
{
  return capacity;
}

SUMOReal MSVehicleType::getNominalCapacity()const
{
  return nominalCapacity;
}

SUMOReal MSVehicleType::getConstantLosses() const {
	return constant_losses;
}

SUMOReal MSVehicleType::getCooperLosses() const {
	return cooper_losses;
}

SUMOReal MSVehicleType::getDragCoefficient() const {
	return drag_coefficient;
}

SUMOReal MSVehicleType::getFrontalArea() const {
	return frontal_area;
}

SUMOReal MSVehicleType::getGearRatio() const {
	return Gear_ratio;
}

SUMOReal MSVehicleType::getGearSysEffc() const {
	return Gear_sys_effc;
}

SUMOReal MSVehicleType::getInternalResis() const {
	return internal_resis;
}

SUMOReal MSVehicleType::getIronLosses() const {
	return iron_losses;
}

SUMOReal MSVehicleType::getMass() const {
	return mass;
}

SUMOReal MSVehicleType::getMotorSysEffc() const {
	return motor_sys_effc;
}

SUMOReal MSVehicleType::getNumCells() const {
	return num_cells;
}

SUMOReal MSVehicleType::getPowerAccs() const {
	return power_accs;
}

SUMOReal MSVehicleType::getRegenRatio() const {
	return regen_ratio;
}

SUMOReal MSVehicleType::getRollingRes() const {
	return rolling_res;
}

SUMOReal MSVehicleType::getWindageLosses() const {
	return windage_losses;
}

void MSVehicleType::setAirDens(SUMOReal ad) {
  air_dens = ad;
}

void MSVehicleType::updateAirDens
 (SUMOReal envtemp,SUMOReal envhum/*, MSVehicle msv, MSDevice_FEV msdf*/)
{
  SUMOReal ad1,ad2;
  ad1=1.30-.0042*envtemp;
  if(envtemp>30){
    ad2=ad1-.02*(envhum/100.);
  }else if(envtemp>20){
    ad2=ad1-.01*(envhum/100.);
  }else if(envtemp>10){
    ad2=ad1-.006*(envhum/100.);
  }else{
    ad2=ad1-.004*(envhum/100.);
  }
  air_dens=ad2;
  //setAirDens(ad2);
}

void MSVehicleType::updateCapacity
 (SUMOReal envtemp/*, MSVehicle msv, MSDevice_FEV msdf*/)
{
  OptionsCont &oc=OptionsCont::getOptions();
  MSNet *net = MSNet::getInstance();
  if(oc.getSimulationVerbosity()>1)
    std::cout<<"----> void MSVehicleType::updateCapacity(SUMOReal et)"<<std::endl;
  if(oc.getSimulationVerbosity()>2)
    std::cout<<" getCapacity(){"<<getCapacity()<<"}"<<std::endl;
  if(net->getCurrentEnvTemp()>50)
    setCapacity(getNominalCapacity()*1.12);
  else if(net->getCurrentEnvTemp()>45)
    setCapacity(getNominalCapacity()*1.1);
  else if(net->getCurrentEnvTemp() >40)
    setCapacity(getNominalCapacity()*1.08);
  else if(net->getCurrentEnvTemp() >35)
    setCapacity(getNominalCapacity()*1.06);  
  else if(net->getCurrentEnvTemp() >30)
    setCapacity(getNominalCapacity()*1.03);  
  else if(net->getCurrentEnvTemp() >25)
    //setCapacity(getNominalCapacity()*1);  
    ;
  else if(net->getCurrentEnvTemp() >20)
    setCapacity(getNominalCapacity()*.96);  
  else if(net->getCurrentEnvTemp() >15)
    setCapacity(getNominalCapacity()*.93);  
  else if(net->getCurrentEnvTemp() >10)
    setCapacity(getNominalCapacity()*.89);  
  else if(net->getCurrentEnvTemp() >5)
    setCapacity(getNominalCapacity()*.85);  
  else if(net->getCurrentEnvTemp() >0)
    setCapacity(getNominalCapacity()*.8);  
  else if(net->getCurrentEnvTemp() >-5)
    setCapacity(getNominalCapacity()*.75);  
  else if(net->getCurrentEnvTemp() >-10)
    setCapacity(getNominalCapacity()*.7);  
  else if(net->getCurrentEnvTemp() >-15)
    setCapacity(getNominalCapacity()*.65);  
  else if(net->getCurrentEnvTemp() >-20)
    setCapacity(getNominalCapacity()*.6);  
  else if(net->getCurrentEnvTemp()>-27)
    setCapacity(getNominalCapacity()*.5);
  if(oc.getSimulationVerbosity()>2)
    std::cout<<" getCapacity(){"<<getCapacity()<<"}"<<std::endl;
}

void MSVehicleType::setCapacity(SUMOReal capacity)
{
  this->capacity = capacity;
}

void MSVehicleType::setNominalCapacity(SUMOReal nc)
{
  this->nominalCapacity=nc;
}

void MSVehicleType::setConstantLosses(SUMOReal constant_losses) {
	this->constant_losses = constant_losses;
}

void MSVehicleType::setCooperLosses(SUMOReal cooper_losses) {
	this->cooper_losses = cooper_losses;
}

void MSVehicleType::setDragCoefficient(SUMOReal drag_coefficient) {
	this->drag_coefficient = drag_coefficient;
}

void MSVehicleType::setFrontalArea(SUMOReal frontal_area) {
	this->frontal_area = frontal_area;
}

void MSVehicleType::setGearRatio(SUMOReal Gear_ratio) {
	this->Gear_ratio = Gear_ratio;
}

void MSVehicleType::setGearSysEffc(SUMOReal Gear_sys_effc) {
	this->Gear_sys_effc = Gear_sys_effc;
}

void MSVehicleType::setInternalResis(SUMOReal internal_resis) {
	this->internal_resis = internal_resis;
}

void MSVehicleType::setIronLosses(SUMOReal iron_losses) {
	this->iron_losses = iron_losses;
}

void MSVehicleType::setMass(SUMOReal mass) {
	this->mass = mass;
}

void MSVehicleType::setMotorSysEffc(SUMOReal motor_sys_effc) {
	this->motor_sys_effc = motor_sys_effc;
}

void MSVehicleType::setNumCells(SUMOReal num_cells) {
	this->num_cells = num_cells;
}

void MSVehicleType::setPowerAccs(SUMOReal power_accs) {
	this->power_accs = power_accs;
}

void MSVehicleType::setRegenRatio(SUMOReal regen_ratio) {
	this->regen_ratio = regen_ratio;
}

void MSVehicleType::setRollingRes(SUMOReal rolling_res) {
	this->rolling_res = rolling_res;
}

void MSVehicleType::setWindageLosses(SUMOReal windage_losses) {
	this->windage_losses = windage_losses;
}

std::string MSVehicleType::getAuxMask() const {
	return aux_mask;
}

SUMOReal MSVehicleType::getCelsius() const {
	return celsius;
}

SUMOReal MSVehicleType::getHumidity() const {
	return humidity;
}

std::string MSVehicleType::getLightMask() const {
	return light_mask;
}

void MSVehicleType::setAuxMask(std::string aux_mask) {
	this->aux_mask = aux_mask;
}

void MSVehicleType::setCelsius(SUMOReal celsius) {
	this->celsius = celsius;
}

void MSVehicleType::setHumidity(SUMOReal humidity) {
	this->humidity = humidity;
}

void MSVehicleType::setLightMask(std::string light_mask) {
	this->light_mask = light_mask;
}

SUMOReal MSVehicleType::getTimeCharge() const {
	return time_charge;
}

void MSVehicleType::setTimeCharge(SUMOReal time_charge) {
	this->time_charge = time_charge;
}

SUMOReal MSVehicleType::getCockpitLightsConsumption() const
{
    return cockpit_lights_consumption;
}

SUMOReal MSVehicleType::getFrontFogLightsConsumption() const
{
    return front_fog_lights_consumption;
}

SUMOReal MSVehicleType::getHeatingAcConsumption() const
{
    return heatingAC_consumption;
}

SUMOReal MSVehicleType::getRadioConsumption() const
{
    return radio_consumption;
}

SUMOReal MSVehicleType::getVehicleHeadligthsConsumption() const
{
    return vehicle_headlights_consumption;
}

void MSVehicleType::setCockpitLigthsConsumption(SUMOReal cockpit_ligths_consumption)
{
    this->cockpit_lights_consumption = cockpit_ligths_consumption;
}

void MSVehicleType::setFrontFogLigthsConsumption(SUMOReal front_fog_ligths_consumption)
{
    this->front_fog_lights_consumption = front_fog_ligths_consumption;
}

void MSVehicleType::setHeatingAcConsumption(SUMOReal heatingAC_consumption)
{
    this->heatingAC_consumption = heatingAC_consumption;
}

void MSVehicleType::setRadioConsumption(SUMOReal radio_consumption)
{
    this->radio_consumption = radio_consumption;
}

void MSVehicleType::setVehicleHeadligthsConsumption(SUMOReal vehicle_headligths_consumption__)
{
    this->vehicle_headlights_consumption = vehicle_headligths_consumption__;
}
