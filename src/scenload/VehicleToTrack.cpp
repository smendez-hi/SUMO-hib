/*
 * VehicleToTrack.cpp
 *
 *  Created on: May 18, 2012
 *      Author: doancea
 */

#ifdef _MSC_VER
  #include <windows_config.h>
#else
  #include <config.h>
#endif

#include <iostream>
#include "VehicleToTrack.h"
#include <utils/options/OptionsCont.h>


static std::vector<VehicleToTrack*> *vehiclesToTrackWithCharacteristicsList;

static int maxVehicles = 8;
static int allowedVehicles = 0;

VehicleToTrack::VehicleToTrack()
{ 
  OptionsCont &oc = OptionsCont::getOptions();
  if(oc.getLoadVerbosity()>1||
     oc.getBuildVerbosity()>1)
    std::cout << "----> VehicleToTrack::VehicleToTrack()\n";
  /* VehicleToTrack::vehiclesToTrackWithCharacteristicsList.push_back(this); */
  if(!this->getVehiclesToTrackWithCharacteristicsList())
    vehiclesToTrackWithCharacteristicsList=new std::vector<VehicleToTrack*>();
  if(allowedVehicles<maxVehicles)
  {
    vehiclesToTrackWithCharacteristicsList->push_back(this);
    allowedVehicles++;
  }
}

VehicleToTrack::~VehicleToTrack() {
  OptionsCont &oc = OptionsCont::getOptions();
  if(oc.getLoadVerbosity()>1||
    oc.getBuildVerbosity()>1)
  {
    std::cout<<getIdentifier()<<std::endl;
    std::cout<<"<---- VehicleToTrack::~VehicleToTrack()\n";
  }
}

/*void VehicleToTrack::toString(const VehicleToTrack& vehicle) {
		std::cout << "VehicleToTrack {\n"
				" Identifier=" << vehicle.getIdentifier() << ";\n"
				" InitialBattery=" << vehicle.getInitialBattery() << ";\n"
				" InitialPoint=" << vehicle.getInitialPoint() << ";\n"
				" FinalPoint=" << vehicle.getFinalPoint() << ";\n"
				" VehicleType=" << vehicle.getVehicleType() << ";\n"
				" Behaviour=" << vehicle.getBehaviour() << ";\n"
				" Overload=" << vehicle.getOverlioad() << ";\n" << "}\n";
	}*/

std::ostream& operator<<(std::ostream& out, const VehicleToTrack& vehicle)
{
  out << "VehicleToTrack\n"
    "{\n"
    " Identifier=" << vehicle.getIdentifier() << ";\n"
    " InitialBattery=" << vehicle.getInitialBattery() << ";\n"
    " InitialPoint=" << vehicle.getInitialPoint() << ";\n"
    " FinalPoint=" << vehicle.getFinalPoint() << ";\n"
    " VehicleType=" << vehicle.getVehicleType() << ";\n"
    " Behaviour=" << vehicle.getBehaviour() << ";\n" <<
    " Overload=" << vehicle.getOverload() << ";\n" <<
    " AsConsumption=" << vehicle.getAsConsumption() << ";\n" <<
    " CoefDrag=" << vehicle.getCoefDrag() << ";\n" <<
    "}";
  return out;
}

std::string VehicleToTrack::getAsConsumption() const {
	return as_consumption;
}

void VehicleToTrack::setAsConsumption(std::string asConsumption) {
	as_consumption = asConsumption;
}

std::string VehicleToTrack::getCoefDrag() const {
	return coef_drag;
}

void VehicleToTrack::setCoefDrag(std::string coefDrag) {
	coef_drag = coefDrag;
}

std::string VehicleToTrack::getCoefRollingResistance() const {
	return coef_rolling_resistance;
}

void VehicleToTrack::setCoefRollingResistance(std::string coefRollingResistance) {
	coef_rolling_resistance = coefRollingResistance;
}

std::string VehicleToTrack::getConstantLoss() const {
	return constant_loss;
}

void VehicleToTrack::setConstantLoss(std::string constantLoss) {
	constant_loss = constantLoss;
}

std::string VehicleToTrack::getCooperLossCoef() const {
	return cooper_loss_coef;
}

void VehicleToTrack::setCooperLossCoef(std::string cooperLossCoef) {
	cooper_loss_coef = cooperLossCoef;
}

std::string VehicleToTrack::getFrontalArea() const {
	return frontal_area;
}

void VehicleToTrack::setFrontalArea(std::string frontalArea) {
	frontal_area = frontalArea;
}

std::string VehicleToTrack::getGearRatio() const {
	return gear_ratio;
}

void VehicleToTrack::setGearRatio(std::string gearRatio) {
	gear_ratio = gearRatio;
}

std::string VehicleToTrack::getGearSystemEff() const {
	return gear_system_eff;
}

void VehicleToTrack::setGearSystemEff(std::string gearSystemEff) {
	gear_system_eff = gearSystemEff;
}

std::string VehicleToTrack::getHeatingConsumption() const {
	return heating_consumption;
}

void VehicleToTrack::setHeatingConsumption(std::string heatingConsumption) {
	heating_consumption = heatingConsumption;
}

std::string VehicleToTrack::getIronLossCoef() const {
	return iron_loss_coef;
}

void VehicleToTrack::setIronLossCoef(std::string ironLossCoef) {
	iron_loss_coef = ironLossCoef;
}

std::string VehicleToTrack::getLights() const {
	return lights;
}

void VehicleToTrack::setLights(std::string lights) {
	this->lights = lights;
}

std::string VehicleToTrack::getMass() const {
	return mass;
}

void VehicleToTrack::setMass(std::string mass) {
	this->mass = mass;
}

std::string VehicleToTrack::getMaxPower() const {
	return max_power;
}

void VehicleToTrack::setMaxPower(std::string maxPower) {
	max_power = maxPower;
}

std::string VehicleToTrack::getMotorEff() const {
	return motor_eff;
}

void VehicleToTrack::setMotorEff(std::string motorEff) {
	motor_eff = motorEff;
}

std::string VehicleToTrack::getNominalCapacity() const {
	return nominal_capacity;
}

void VehicleToTrack::setNominalCapacity(std::string nominalCapacity) {
	nominal_capacity = nominalCapacity;
}

std::string VehicleToTrack::getNumberCells() const {
	return number_cells;
}

void VehicleToTrack::setNumberCells(std::string numberCells) {
	number_cells = numberCells;
}

std::string VehicleToTrack::getPeukertCoeff() const {
	return peukert_coeff;
}

void VehicleToTrack::setPeukertCoeff(std::string peukertCoeff) {
	peukert_coeff = peukertCoeff;
}

std::string VehicleToTrack::getRadioConsumption() const {
	return radio_consumption;
}

void VehicleToTrack::setRadioConsumption(std::string radioConsumption) {
	radio_consumption = radioConsumption;
}

std::string VehicleToTrack::getRegenRatio() const {
	return regen_ratio;
}

void VehicleToTrack::setRegenRatio(std::string regenRatio) {
	regen_ratio = regenRatio;
}

std::string VehicleToTrack::getTimeCharge() const {
	return time_charge;
}

void VehicleToTrack::setTimeCharge(std::string timeCharge) {
	time_charge = timeCharge;
}

std::string VehicleToTrack::getWidescreenCons() const {
	return widescreen_cons;
}

void VehicleToTrack::setWidescreenCons(std::string widescreenCons) {
	widescreen_cons = widescreenCons;
}

std::string VehicleToTrack::getWindageLossCoef() const {
	return windage_loss_coef;
}

void VehicleToTrack::setWindageLossCoef(std::string windageLossCoef) {
	windage_loss_coef = windageLossCoef;
}


std::vector<VehicleToTrack*>*
 VehicleToTrack::getVehiclesToTrackWithCharacteristicsList()
{
  //return VehicleToTrack::vehiclesToTrackWithCharacteristicsList;
  return vehiclesToTrackWithCharacteristicsList;
}

