/*
 * VehicleToTrack.h
 *
 *  Created on: May 18, 2012
 *      Author: doancea
 */

#ifndef VEHICLETOTRACK_H_
#define VEHICLETOTRACK_H_

#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <iostream>
#include <string>
#include <vector>
#include "../utils/options/OptionsCont.h"

class VehicleToTrack
{
 public:  
  VehicleToTrack();
  virtual ~VehicleToTrack();

  std::string getBehaviour() const
  {
    return behaviour;
  }

  void setBehaviour(std::string behaviour)
  {
    this->behaviour = behaviour;
  }

  std::string getFinalPoint() const
  {
      return finalPoint;
  }

  void setFinalPoint(std::string finalPoint)
  {
    OptionsCont &oc=OptionsCont::getOptions();
    if(oc.getAnyVerbosity()>2)
    {
      std::cout<<"VehicleToTrack::setFinalPoint(std::string finalPoint{"
       <<finalPoint<<"})"<<std::endl;
    }
    this->finalPoint = finalPoint;
  }

  std::string getIdentifier() const
  {
    std::string returning;
    try{
      returning=identifier;
    }catch(...){
      std::cout<<"CRITICAL std::string VehicleToTrack::getIdentifier()"<<std::endl;
      returning="";
    }
    return returning;
  }

  void setIdentifier(std::string identifier)
  {
      this->identifier = identifier;
  }

  std::string getInitialBattery() const
  {
      return initialBattery;
  }

  void setInitialBattery(std::string initialBattery)
  {
      this->initialBattery = initialBattery;
  }

  std::string getInitialPoint() const
  {
      return initialPoint;
  }

  void setInitialPoint(std::string initialPoint)
  {
      this->initialPoint = initialPoint;
  }

  std::string getOverload() const
  {
      return overload;
  }

  void setOverload(std::string overlioad)
  {
      this->overload = overlioad;
  }

  std::string getVehicleType() const
  {
      return vehicleType;
  }

  void setVehicleType(std::string vehicleType)
  {
      this->vehicleType = vehicleType;
  }

  void toString(const VehicleToTrack& vehicle);

  friend std::ostream& operator<<(std::ostream& out, const VehicleToTrack& vehicle);

  std::string getAsConsumption() const;

  void setAsConsumption(std::string asConsumption);

	std::string getCoefDrag() const;

	void setCoefDrag(std::string coefDrag);

	std::string getCoefRollingResistance() const;

	void setCoefRollingResistance(std::string coefRollingResistance);
	std::string getConstantLoss() const;

	void setConstantLoss(std::string constantLoss);

	std::string getCooperLossCoef() const;

	void setCooperLossCoef(std::string cooperLossCoef);
	std::string getFrontalArea() const;

	void setFrontalArea(std::string frontalArea);

	std::string getGearRatio() const;

	void setGearRatio(std::string gearRatio);

	std::string getGearSystemEff() const;

	void setGearSystemEff(std::string gearSystemEff);

	std::string getHeatingConsumption() const;

	void setHeatingConsumption(std::string heatingConsumption);

	std::string getIronLossCoef() const;

	void setIronLossCoef(std::string ironLossCoef);

	std::string getLights() const;

	void setLights(std::string lights);

	std::string getMass() const;

	void setMass(std::string mass);

	std::string getMaxPower() const;

  void setMaxPower(std::string maxPower);

  std::string getMotorEff() const;

  void setMotorEff(std::string motorEff);

  std::string getNominalCapacity() const;

  void setNominalCapacity(std::string nominalCapacity);

  std::string getNumberCells() const;

  void setNumberCells(std::string numberCells);

  std::string getPeukertCoeff() const;

  void setPeukertCoeff(std::string peukertCoeff);

  std::string getRadioConsumption() const;

  void setRadioConsumption(std::string radioConsumption);

  std::string getRegenRatio() const;

  void setRegenRatio(std::string regenRatio);

  std::string getTimeCharge() const;

  void setTimeCharge(std::string timeCharge);

  std::string getWidescreenCons() const;

  void setWidescreenCons(std::string widescreenCons);

  std::string getWindageLossCoef() const;

  void setWindageLossCoef(std::string windageLossCoef);
  
  
  static std::vector<VehicleToTrack *> *
   getVehiclesToTrackWithCharacteristicsList();
  
  
  /*
  static std::vector<VehicleToTrack *> *
   getVehiclesToTrackWithCharacteristicsList()
  {
    return vehiclesToTrackWithCharacteristicsList;
  }
  */

  std::string getAdasWebserviceUrl(){return adasWebserviceUrl;}
 
  void setAdasWebserviceUrl(std::string awu){adasWebserviceUrl=awu;}

 private:
  std::string identifier;
  std::string initialBattery;
  std::string initialPoint;
  std::string finalPoint;
  std::string vehicleType;
  std::string behaviour;
  std::string overload;
  //Electrical characteristics.
  std::string mass;
  std::string coef_rolling_resistance;
	std::string frontal_area;
	std::string coef_drag;
	std::string motor_eff;
	std::string gear_ratio;
	std::string gear_system_eff;
	std::string regen_ratio;
	std::string cooper_loss_coef;
	std::string iron_loss_coef;
	std::string windage_loss_coef;
	std::string constant_loss;
	std::string heating_consumption;
	std::string radio_consumption;
	std::string lights;
	std::string number_cells;
	std::string peukert_coeff;
	std::string nominal_capacity;
	std::string time_charge;
	std::string widescreen_cons;
	std::string as_consumption;
	std::string max_power;
  std::string adasWebserviceUrl;
  
  /* 
  static std::vector<VehicleToTrack *> *vehiclesToTrackWithCharacteristicsList;
  */
};
#endif /* VEHICLETOTRACK_H_ */
