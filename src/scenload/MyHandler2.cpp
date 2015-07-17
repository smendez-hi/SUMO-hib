/*
 * MyHandler2.cpp
 *
 *  Created on: Abril 9, 2012
 *      Author: doancea
 */

#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#define elif else if

#include <algorithm>
#if FALSE
#include <ctime>
#else
#include <time.h>
#endif
#include <iomanip>
#include <iostream>
#include <map>
#include <vector>
#include <sstream>
// No needed
//#include <xercesc/framework/XMLAttr.hpp>
#include <xercesc/sax2/Attributes.hpp>
#include <xercesc/util/XMLString.hpp>
#include "MyHandler2.h"
#include "VehicleToTrack.h"
#include "utils/options/OptionsCont.h"

using namespace xercesc;

static time_t *initialTime;

static std::string initialTimeString;

MyHandler2::MyHandler2()
{
  OptionsCont &oc = OptionsCont::getOptions();
  if(oc.getLoadVerbosity()>1)
    std::cout << "----> MyHandler2::MyHandler2()\n";
}

int MyHandler2::find
 (std::vector<VehicleToTrack>& _vec, const std::string& vehicle)
{
  OptionsCont &oc = OptionsCont::getOptions();
  if(oc.getLoadVerbosity()>2)
  {
    std::cout << "The vector size is:  " << _vec.size() << "\n";
    std::cout << "vehicle="<<vehicle<<std::endl;
  }
  /* The Unknown New One */
  /*
  std::vector<VehicleToTrack>::const_iterator iter;
  for(iter = _vec.begin(); iter != _vec.end(); ++iter)
  {
    if(iter->getIdentifier() == vehicle)
    {
      return iter.???();
    }
  }
  */
  /* The Old Safe One */
  
  for(int i = 0; i<_vec.size(); i++)
  {
    if(_vec.at(i).getIdentifier() == vehicle)
    {
      if(oc.getLoadVerbosity()>2)
        std::cout<<"found..."<<std::endl;
      return i;
    }
    else
    {
      if(oc.getLoadVerbosity()>2)
        std::cout << "_vec.at(i).getIdentifier()="<<_vec.at(i).getIdentifier()<<std::endl;
    }
  }
  
  /* Or not found */
  return -1;
}

void MyHandler2::processAttribute
 (VehicleToTrack &vehicle, const Attributes &attrs)
{
  int length;
  OptionsCont &oc = OptionsCont::getOptions();
  /*VehicleToTrack *veh = &vehicle;*/
  if(oc.getLoadVerbosity()>1)
    std::cout << "----> MyHandler2::processAttribute(...)" << std::endl;
  //std::ostream& out;
  if(oc.getLoadVerbosity()>2)
    std::cout<<" attrs.getLength()="<<attrs.getLength()<<std::endl;
  //Gets the number of elements.
  length = attrs.getLength();
  //Process each attribute.
  for(int var = 0; var < length; ++var)
  {
    const XMLCh* attrName;
    attrName = attrs.getQName(var);
    const XMLCh* attrValue;
    attrValue = attrs.getValue(var);
    char* _name = xercesc::XMLString::transcode(attrName);
    char* _value = xercesc::XMLString::transcode(attrValue);
    std::cout << "_name=" << _name << ", _value= " << _value << std::endl;
    std::string id = "id";
    //dynamics.
    std::string mass = "mass";
    std::string coef_rolling_resistance = "coef_rolling_resistance";
    std::string frontal_area = "frontal_area";
    std::string coef_drag = "coef_drag";
    //motor
    std::string motor_eff = "motor_eff";
    std::string gear_ratio = "gear_ratio";
    std::string gear_system_eff = "gear_system_eff";
    std::string regen_ratio = "regen_ratio";
    //battery.
    std::string cooper_loss_coef = "cooper_loss_coef";
    std::string iron_loss_coef = "iron_loss_coef";
    std::string windage_loss_coef = "windage_loss_coef";
    std::string constant_loss = "constant_loss";
    std::string heating_consumption = "heating_consumption";
    std::string radio_consumption = "radio_consumption";
    std::string lights = "lights";
    std::string number_cells = "number_cells";
    std::string peukert_coeff = "peukert_coeff";
    std::string nominal_capacity = "nominal_capacity";
    std::string time_charge = "time_charge";
    std::string widescreen_cons = "widescreen_cons";
    std::string as_consumption = "A_C_consumption";
    std::string max_power = "max_power";
    if(_name == id)
    {
      if(oc.getLoadVerbosity()>2)
      {
        std::cout << " (_name==id)" << std::endl;
        std::cout << " (_value==" << _value << std::endl;
      }
      vehicle.setIdentifier(_value);
      if(oc.getLoadVerbosity()>2)
      {
        std::cout << " (_name==id)" << std::endl;
        std::cout << " (_value==" << _value << std::endl;
      }
    }
    /* Dynamics. */
    elif(_name == mass)
    {
      if(oc.getLoadVerbosity()>2)
      {
        std::cout << " (_name==mass)" << std::endl;
        std::cout << " (_value==" << _value << std::endl;
      }     
      vehicle.setMass(_value);
      if(oc.getLoadVerbosity()>2)
      {
        std::cout << " (_name==mass)" << std::endl;
        std::cout << " (_value==" << _value << std::endl;
      }
    }
    elif(_name == coef_rolling_resistance)
    {
      vehicle.setCoefRollingResistance(_value);
    }
    elif(_name == frontal_area)
    {
      vehicle.setFrontalArea(_value);
    }
    elif(_name == coef_drag)
    {
      vehicle.setCoefDrag(_value);
    }
    /* Motor. */
    elif(_name == motor_eff)
    {
      if(oc.getLoadVerbosity()>2)
      {
        std::cout << " (_name==motor_eff)" << std::endl;
        std::cout << " (_value==" << _value << std::endl;
      }      
      vehicle.setMotorEff(_value);
      if(oc.getLoadVerbosity()>2)
      {
        std::cout << " (_name==motor_eff)" << std::endl;
        std::cout << " (_value==" << _value << std::endl;
      }      
    }
    elif (_name == gear_ratio) {
      vehicle.setGearRatio(_value);
    }
    else if (_name == gear_system_eff) {
      vehicle.setGearSystemEff(_value);
    }
    else if (_name == regen_ratio) {
      vehicle.setRegenRatio(_value);
    }
    /* Battery. */
    elif(_name == cooper_loss_coef)
    {
      if(oc.getLoadVerbosity()>2)
      {
        std::cout << " (_name==cooper_loss_coef)" << std::endl;
        std::cout << " (_value==" << _value << std::endl;
      }      
      vehicle.setCooperLossCoef(_value);
      if(oc.getLoadVerbosity()>2)
      {
        std::cout << " (_name==cooper_loss_coef)" << std::endl;
        std::cout << " (_value==" << _value << std::endl;
      }  
    }
    else if (_name == iron_loss_coef) {
      vehicle.setIronLossCoef(_value);
    }
    else if (_name == windage_loss_coef) {
      vehicle.setWindageLossCoef(_value);
    }
    else if (_name == constant_loss) {
      vehicle.setConstantLoss(_value);
    }
    else if (_name == heating_consumption) {
      vehicle.setHeatingConsumption(_value);
    }
    else if (_name == radio_consumption) {
      vehicle.setRadioConsumption(_value);
    }
    else if (_name == lights) {
      vehicle.setLights(_value);
    }
    elif(_name == number_cells)
    {
      if(oc.getLoadVerbosity()>2)
      {
        std::cout << " (_name==number_cells" << std::endl;
        std::cout << " (_value==" << _value << std::endl;
      }
      vehicle.setNumberCells(_value);
      if(oc.getLoadVerbosity()>2)
      {
        std::cout << " (_name==number_cells" << std::endl;
        std::cout << " (_value==" << _value << std::endl;
      }
    }
    else if (_name == peukert_coeff) {
      vehicle.setPeukertCoeff(_value);
    }
    else if (_name == nominal_capacity) {
      vehicle.setNominalCapacity(_value);
    }
    else if (_name == time_charge) {
      vehicle.setTimeCharge(_value);
    }
    else if (_name == widescreen_cons) {
      vehicle.setWidescreenCons(_value);
    }
    else if (_name == as_consumption) {
      vehicle.setAsConsumption(_value);
    }
    else if (_name == max_power) {
      vehicle.setMaxPower(_value);
    }
    XMLString::release(&_name);
    XMLString::release(&_value);
  }
  if(oc.getLoadVerbosity()>2)
    std::cout << "MyHandler2::processAttribute(...) [400] " << std::endl
              << vehicle << std::endl;    
  std::cout<<"vehicles.size()="<<vehicles.size()<<std::endl;
  vehicles.push_back(vehicle);
  std::cout<<"vehicles.size()="<<vehicles.size()<<std::endl;
  if(oc.getLoadVerbosity()>2)
    std::cout << "MyHandler2::processAttribute(...) [500] " << std::endl
              << vehicle << std::endl;    
  if(oc.getLoadVerbosity()>1)
    std::cout << "<---- MyHandler2::processAttribute(...)" << std::endl;    
}

void MyHandler2::startElement
 (const XMLCh* const uri, const XMLCh* const localname,
  const XMLCh* const qname, const Attributes& attrs)
 {
  VehicleToTrack *vehicle;
  int i, length;
  std::string vehId;
  const XMLCh *attrName, *attrValue, *innerAttrName, *innerAttrValue;
  char *name, *value, *innerName, *innerValue;
  OptionsCont &oc = OptionsCont::getOptions();
  if(oc.getLoadVerbosity()>1)
    std::cout<<"----> void MyHandler2::startElement(...)"<<std::endl;
  if(oc.getLoadVerbosity()>2)
    std::cout<<"MyHandler2::startElement(..."<<"uri{"<<uri<< "},localname{"
     <<localname<<"},qname{"<<qname
     //<< ", attrs=" << attrs // Does not have operator<<
     <<"})"<<std::endl;    
  
  //std::vector<VehicleToTrack> vehicles;
  
  //VehicleToTrack vehicle;
  char* message = xercesc::XMLString::transcode(localname);
    
  if(oc.getLoadVerbosity()>2)
    std::cout << "message=" << message << std::endl;

  // several important tags
  std::string vehicleToTrack = "vehicleToTrack";
  std::string dynamics = "dynamics";
  std::string motor = "motor";
  std::string battery = "battery";
  std::string VehicleCharacteristic = "VehicleCharacteristic";
  
  // tag and attribute for timestamp catching
  std::string general = "general";
  std::string start = "start";  
  
  // dynamics attrs
  std::string mass = "mass";
  std::string coef_rolling_resistance = "coef_rolling_resistance";
  std::string frontal_area = "frontal_area";
  std::string coef_drag = "coef_drag";
  // motor attrs
  std::string motor_eff = "motor_eff";
  std::string gear_ratio = "gear_ratio";
  std::string gear_system_eff = "gear_system_eff";
  std::string regen_ratio = "regen_ratio";
  // battery attrs
  std::string cooper_loss_coef = "cooper_loss_coef";
  std::string iron_loss_coef = "iron_loss_coef";
  std::string windage_loss_coef = "windage_loss_coef";
  std::string constant_loss = "constant_loss";
  std::string heating_consumption = "heating_consumption";
  std::string radio_consumption = "radio_consumption";
  std::string lights = "lights";
  std::string number_cells = "number_cells";
  std::string peukert_coeff = "peukert_coeff";
  std::string nominal_capacity = "nominal_capacity";
  std::string time_charge = "time_charge";
  std::string widescreen_cons = "widescreen_cons";
  std::string as_consumption = "A_C_consumption";
  std::string max_power = "max_power";

  //if(message == general)
  if(message == vehicleToTrack)
  {
    MyHandler2::depth = 0;
    if(oc.getLoadVerbosity()>2)
      std::cout << "---Found a " << vehicleToTrack << " element---" << std::endl;
    processVehicleToTrackAttributes(vehicle, attrs);
  }
  if(message == VehicleCharacteristic)
  {
    MyHandler2::depth = 1;
    if(oc.getLoadVerbosity()>2)
      std::cout << "---Found a " << VehicleCharacteristic << " element---" << std::endl;
		vehId = "id";
		length = attrs.getLength();
		for (int var = 0; var < length; ++var) {
			attrName = attrs.getQName(var);
			attrValue = attrs.getValue(var);
			char* name = xercesc::XMLString::transcode(attrName);
      std::cout<<" name="<<name;
			char* value = xercesc::XMLString::transcode(attrValue);
      std::cout<<" value="<<value;
      if(name == vehId)
      {
        if(oc.getLoadVerbosity()>2)
          std::cout << "---check1---" << std::endl;
				std::stringstream ss;
				std::string s;
				ss << value;
				ss >> s;
        i = find(vehicles,s);
        if(-1!=i)
        {
          /*
          processVehicleToTrackAttributes(vehicles.at(i),attrs);
          */
          /* Save the vehicle id for latter characteristics processing */
          MyHandler2::current = vehicles.at(i).getIdentifier();
          if(oc.getLoadVerbosity()>2)
            std::cout << "---check11--- current=" <<current<<std::endl;
					//The vehicle Id exists in vector.
					//std::cout <<"VehicleCharacteristic is already in...!" << std::endl;
					std::vector<VehicleToTrack>::iterator iter;
          for(iter = vehicles.begin(); iter != vehicles.end(); ++iter)
          {
            if(oc.getLoadVerbosity()>2)
              std::cout << "---check111---" << std::endl;
            if (iter->getIdentifier() == s) {
              if(oc.getLoadVerbosity()>2)
              {
              std::cout << "---Found the element with id " << s << " !" << std::endl;
              std::cout << "The element " << message << " | " << dynamics << std::endl;              
              /*
              innerAttrName = attrs.getQName(var);
              const XMLCh* attrValue = attrs.getValue(var);
              char* name = xercesc::XMLString::transcode(attrName);
              char* value = xercesc::XMLString::transcode(attrValue);
              xercesc::XMLString::transcode(attrName);
              char* value = xercesc::XMLString::transcode(attrValue);
              */              
              std::cout << "The element " << message << " | " << dynamics << std::endl;
              }
              if (message == dynamics) {
                if(oc.getLoadVerbosity()>2)
                  std::cout << "---Found the " << dynamics << " element---!" << std::endl;
                //processAttribute(*iter.base(), attrs);
                ;
              }
              if (message == motor) {
                if(oc.getLoadVerbosity()>2)
                  std::cout << "---Found the " << motor << " element---!" << std::endl;
                //processAttribute(*iter.base(), attrs);
              }
              if (message == battery) {
                if(oc.getLoadVerbosity()>2)
                  std::cout << "---Found the " << battery << " element---!" << std::endl;
                //processAttribute(*iter.base(), attrs);
              }
            }
          }
        }
        else
        {
          /* Got some vehicle characteristics for an unknown vehicle */
          /*
          vehicle.setIdentifier(value);
          */
            if(oc.getLoadVerbosity()>2)
              std::cout << "got some vehicle characteristics for an unknown '"
                << s << "' vehicle << std::endl" << std::endl;
        }
			}
			XMLString::release(&name);
			XMLString::release(&value);
		}
  }
  else if(message == dynamics)
  {
    MyHandler2::depth = 3;
    if(oc.getLoadVerbosity()>2)
      std::cout << "---Found a " << dynamics << " element---" << std::endl;
    length = attrs.getLength();
    for (int var = 0; var < length; ++var) {
      attrName = attrs.getQName(var);
      attrValue = attrs.getValue(var);
      char* name = xercesc::XMLString::transcode(attrName);
      if(oc.getLoadVerbosity()>2)
        std::cout<<" name="<<name<<std::endl;
      char* value = xercesc::XMLString::transcode(attrValue);
      if(oc.getLoadVerbosity()>2)
        std::cout<<" value="<<value<<std::endl;
      if(name == mass)
      {
        i = find(vehicles, current);
        if(-1 != i)
        {
          vehicles.at(i).setMass(value);
          if(oc.getLoadVerbosity()>2)
            std::cout<<" vehicles.at(i).getMass()="<<vehicles.at(i).getMass()<<std::endl;
        }
      }
      else if(name == coef_rolling_resistance)
      {
        i = find(vehicles, current);
        if(-1 != i)
        {
          vehicles.at(i).setCoefRollingResistance(value);
          if(oc.getLoadVerbosity()>2)
            std::cout<<" vehicles.at(i).getCoefRollingResistance()="<<vehicles.at(i).getCoefRollingResistance()<<std::endl;
        }
      }
      else if(name == frontal_area)
      {
        i = find(vehicles, current);
        if(-1 != i)
        {
          vehicles.at(i).setFrontalArea(value);          
        }
      }
      else if(name == coef_drag)
      {
        i = find(vehicles, current);
        if(-1 != i)
        {
          vehicles.at(i).setCoefDrag(value);
          if(oc.getLoadVerbosity()>2)
            std::cout<<" vehicles.at(i).getCoefDrag()="<<vehicles.at(i).getCoefDrag()<<std::endl;
        }
        else
        {
          /* Vehicle not found? */
          ;
        }
      }
      XMLString::release(&name);
      XMLString::release(&value);
    }
  }
  else if (message == motor)
  {    
    MyHandler2::depth = 3;
    if(oc.getLoadVerbosity()>2)
      std::cout << "---Found a " << motor << " element---" << std::endl;
    length = attrs.getLength();
    for (int var = 0; var < length; ++var) {
      attrName = attrs.getQName(var);
      attrValue = attrs.getValue(var);
      char* name = xercesc::XMLString::transcode(attrName);
      if(oc.getLoadVerbosity()>2)
        std::cout<<" name="<<name<<std::endl;
      char* value = xercesc::XMLString::transcode(attrValue);
      if(oc.getLoadVerbosity()>2)
        std::cout<<" value="<<value<<std::endl;
      if(name == motor_eff)
      {
        i = find(vehicles, current);
        if(-1 != i)
        {
          vehicles.at(i).setMotorEff(value);
          /*if(OptionsCont::getOptions().getInt("loadVerbosity"))
            std::cout<<" vehicles.at(i).getMass()="<<vehicles.at(i).getMass()<<std::endl;*/
        }
      }
      else if(name == gear_ratio)
      {
        i = find(vehicles, current);
        if(-1 != i)
        {
          vehicles.at(i).setGearRatio(value);
          /*if(OptionsCont::getOptions().getInt("loadVerbosity"))
            std::cout<<" vehicles.at(i).getCoefRollingResistance()="<<vehicles.at(i).getCoefRollingResistance()<<std::endl;*/
        }
      }
      else if(name == gear_system_eff)
      {
        i = find(vehicles, current);
        if(-1 != i)
        {
          vehicles.at(i).setGearSystemEff(value);          
        }
      }
      else if(name == regen_ratio)
      {
        i = find(vehicles, current);
        if(-1 != i)
        {
          vehicles.at(i).setRegenRatio(value);
          /*if(OptionsCont::getOptions().getInt("loadVerbosity"))
            std::cout<<" vehicles.at(i).getCoefDrag()="<<vehicles.at(i).getCoefDrag()<<std::endl;*/
        }
        else
        {
          /* Vehicle not found? */
          ;
        }
      }
      XMLString::release(&name);
      XMLString::release(&value);
    }    
  }
  else if (message == battery)  
  /*
  std::string cooper_loss_coef = "cooper_loss_coef";
  std::string iron_loss_coef = "iron_loss_coef";
  std::string windage_loss_coef = "windage_loss_coef";
  std::string constant_loss = "constant_loss";
  std::string heating_consumption = "heating_consumption";
  std::string radio_consumption = "radio_consumption";
  std::string lights = "lights";
  std::string number_cells = "number_cells";
  std::string peukert_coeff = "peukert_coeff";
  std::string nominal_capacity = "nominal_capacity";
  std::string time_charge = "time_charge";
  std::string widescreen_cons = "widescreen_cons";
  std::string as_consumption = "A_C_consumption";
  std::string max_power = "max_power";
  */ 
  {    
    MyHandler2::depth = 3;
    if(oc.getLoadVerbosity()>2)
      std::cout << "---Found a " << motor << " element---" << std::endl;
    length = attrs.getLength();
    for (int var = 0; var < length; ++var) {
      attrName = attrs.getQName(var);
      attrValue = attrs.getValue(var);
      char* name = xercesc::XMLString::transcode(attrName);
      if(oc.getLoadVerbosity()>2)
        std::cout<<" name="<<name<<std::endl;
      char* value = xercesc::XMLString::transcode(attrValue);
      if(oc.getLoadVerbosity()>2)
        std::cout<<" value="<<value<<std::endl;
      if(name == constant_loss)
      {
        i = find(vehicles, current);
        if(-1 != i)
        {
          vehicles.at(i).setConstantLoss(value);
          /*if(OptionsCont::getOptions().getInt("loadVerbosity"))
            std::cout<<" vehicles.at(i).getMass()="<<vehicles.at(i).getMass()<<std::endl;*/
        }
      }
      else if(name == cooper_loss_coef)
      {
        i = find(vehicles, current);
        if(-1 != i)
        {
          vehicles.at(i).setCooperLossCoef(value);
          /*if(OptionsCont::getOptions().getInt("loadVerbosity"))
            std::cout<<" vehicles.at(i).getCoefRollingResistance()="<<vehicles.at(i).getCoefRollingResistance()<<std::endl;*/
        }
      }
      else if(name == iron_loss_coef)
      {
        i = find(vehicles, current);
        if(-1 != i)
        {
          vehicles.at(i).setIronLossCoef(value);          
        }
      }
      else if(name == windage_loss_coef)
      {
        i = find(vehicles, current);
        if(-1 != i)
        {
          vehicles.at(i).setWindageLossCoef(value);
          /*if(OptionsCont::getOptions().getInt("loadVerbosity"))
            std::cout<<" vehicles.at(i).getCoefDrag()="<<vehicles.at(i).getCoefDrag()<<std::endl;*/
        }
        else
        {
          /* Vehicle not found? */
          ;
        }
      }
      else if(name == max_power)
      {
        i = find(vehicles, current);
        if(-1 != i)
        {
          vehicles.at(i).setMaxPower(value);
          /*if(OptionsCont::getOptions().getInt("loadVerbosity"))
            std::cout<<" vehicles.at(i).getMass()="<<vehicles.at(i).getMass()<<std::endl;*/
        }
      }
      else if(name == heating_consumption)
      {
        i = find(vehicles, current);
        if(-1 != i)
        {
          vehicles.at(i).setHeatingConsumption(value);
          /*if(OptionsCont::getOptions().getInt("loadVerbosity"))
            std::cout<<" vehicles.at(i).getCoefRollingResistance()="<<vehicles.at(i).getCoefRollingResistance()<<std::endl;*/
        }
      }
      else if(name == radio_consumption)
      {
        i = find(vehicles, current);
        if(-1 != i)
        {
          vehicles.at(i).setRadioConsumption(value);          
        }
      }
      else if(name == lights)
      {
        i = find(vehicles, current);
        if(-1 != i)
        {
          vehicles.at(i).setLights(value);
          /*if(OptionsCont::getOptions().getInt("loadVerbosity"))
            std::cout<<" vehicles.at(i).getCoefDrag()="<<vehicles.at(i).getCoefDrag()<<std::endl;*/
        }
        else
        {
          /* Vehicle not found? */
          ;
        }
      }
      else if(name == number_cells)
      {
        i = find(vehicles, current);
        if(-1 != i)
        {
          vehicles.at(i).setNumberCells(value);
          /*if(OptionsCont::getOptions().getInt("loadVerbosity"))
            std::cout<<" vehicles.at(i).getMass()="<<vehicles.at(i).getMass()<<std::endl;*/
        }
      }
      else if(name == peukert_coeff)
      {
        i = find(vehicles, current);
        if(-1 != i)
        {
          vehicles.at(i).setPeukertCoeff(value);
          /*if(OptionsCont::getOptions().getInt("loadVerbosity"))
            std::cout<<" vehicles.at(i).getCoefRollingResistance()="<<vehicles.at(i).getCoefRollingResistance()<<std::endl;*/
        }
      }
      else if(name == nominal_capacity)
      {
        i = find(vehicles, current);
        if(-1 != i)
        {
          vehicles.at(i).setNominalCapacity(value);          
        }
      }
      else if(name == time_charge)
      {
        i = find(vehicles, current);
        if(-1 != i)
        {
          vehicles.at(i).setTimeCharge(value);
          /*if(OptionsCont::getOptions().getInt("loadVerbosity"))
            std::cout<<" vehicles.at(i).getCoefDrag()="<<vehicles.at(i).getCoefDrag()<<std::endl;*/
        }
        else
        {
          /* Vehicle not found? */
          ;
        }
      }
      else if(name == widescreen_cons)
      {
        i = find(vehicles, current);
        if(-1 != i)
        {
          vehicles.at(i).setWidescreenCons(value);
          /*if(OptionsCont::getOptions().getInt("loadVerbosity"))
            std::cout<<" vehicles.at(i).getMass()="<<vehicles.at(i).getMass()<<std::endl;*/
        }
      }
      else if(name == as_consumption)
      {
        i = find(vehicles, current);
        if(-1 != i)
        {
          vehicles.at(i).setAsConsumption(value);
          /*if(OptionsCont::getOptions().getInt("loadVerbosity"))
            std::cout<<" vehicles.at(i).getCoefRollingResistance()="<<vehicles.at(i).getCoefRollingResistance()<<std::endl;*/
        }
      }
      XMLString::release(&name);
      XMLString::release(&value);
    }    
  }
  else if(message==general)
  {
    /* There are several 'general' tags */
    length = attrs.getLength();
    for(int var = 0; var < length; var++)
    {
      attrName = attrs.getQName(var);
      attrValue = attrs.getValue(var);
      char* name = xercesc::XMLString::transcode(attrName);
      if(oc.getLoadVerbosity()>2)
        std::cout<<" name="<<name<<std::endl;
      char* value = xercesc::XMLString::transcode(attrValue);
      if(oc.getLoadVerbosity()>2)
        std::cout<<" value="<<value<<std::endl;
      if(name == start)
      {
        /* Then this is the general which has the start time, so we save it */
        std::string initialTimeString = value;
        struct tm *startTime = new struct tm; 
        /* UTC respects 2012-09-12T09:45:46
         *              0 2 4 6 8  11 14 17
         *                          
         */
        startTime->tm_year = atoi(initialTimeString.substr(0, 4).c_str())-1900;
        startTime->tm_mon  = atoi(initialTimeString.substr(5, 2).c_str())-1;
        startTime->tm_mday = atoi(initialTimeString.substr(8, 2).c_str());
        startTime->tm_hour = atoi(initialTimeString.substr(11,2).c_str());
        startTime->tm_min  = atoi(initialTimeString.substr(14,2).c_str());
        startTime->tm_sec  = atoi(initialTimeString.substr(17,2).c_str());
        time_t *aux = new time_t;
        
        //*aux = mktime(time);
        //aux = &mktime(time);
        //time(aux);
        if(oc.isSet("ARFTmode"))
          // If Augmented Reality, use real time
          *aux = time(NULL);
        else
          // We keep the time indicated in the input summary file
          *aux = mktime(startTime);
        ;
        
        std::cout<<"aux="<<aux<<std::endl;
        std::cout<<"*aux="<<*aux<<std::endl;
        
        if(*aux == -1)
        {
          std::cout<<"CRITICAL 1 mktime return value in startElement"<<std::endl;
          exit(0x03);
        }
        else if(*aux == 0)
        {
          std::cout<<"CRITICAL 2 mktime return value in startElement"<<std::endl;
          exit(0x03);
        }
        setInitialTime(aux);
        setInitialTimeString(initialTimeString);
        //delete aux; // This is an error
        //delete startTime; // This is an error
      }
      XMLString::release(&name);
      XMLString::release(&value);
    }    
  }
  XMLString::release(&message);
  //XMLString::release(&_val);
}

//!!!!Here you need to add another argument (vehicle) to set information to vehicle.
void MyHandler2::processVehicleToTrackAttributes
 (VehicleToTrack* veh, const Attributes& attrs)
{
  int length;
  OptionsCont &oc = OptionsCont::getOptions();
  if(oc.getLoadVerbosity()>1)
    std::cout << "----> MyHandler2::processAttribute(...)" << std::endl;
  // Actually create the vehicle
  veh = new VehicleToTrack();
  //std::ostream& out;
  if(oc.getLoadVerbosity()>2)
    std::cout<<"attrs.getLength()="<<attrs.getLength()<<std::endl;
  //Gets the number of elements.
  length = attrs.getLength();
  //Process each attribute.
  for(int var = 0; var < length; ++var)
  {
    if(oc.getLoadVerbosity()>2)
      std::cout << "[100] MyHandler2::processAttribute(...)" << std::endl;
    const XMLCh* attrName;
    attrName = attrs.getQName(var);
    const XMLCh* attrValue;
    attrValue = attrs.getValue(var);
    char* _name = xercesc::XMLString::transcode(attrName);
    char* _value = xercesc::XMLString::transcode(attrValue);
    //std::cout << _name << " : " << _value << std::endl;
    std::string id = "identifier";
    std::string initBattery = "initialBattery";
    std::string initialPoint = "initialPoint";
    std::string finalPoint = "FinalPoint";
    std::string vehicleType = "vehicleType";
    std::string behaviour = "behaviour";
    std::string overload = "overlioad";
    std::string vehId = "id";
    if(_name == id)
    {
      if(oc.getLoadVerbosity()>2)
      {
        std::cout << " (_name==id)" << std::endl;
        std::cout << " (_value==" << _value << std::endl;
      }
      veh->setIdentifier(_value);
    }
#define elif else if
    elif(_name == initBattery)
    {
      if(oc.getLoadVerbosity()>2)
      {
        std::cout << " (_name==initBattery)" << std::endl;
        std::cout << " (_value==" << _value << std::endl;
      }
      veh->setInitialBattery(_value);
    }
    elif(_name == initialPoint)
    {
      if(oc.getLoadVerbosity()>2)
      {
        std::cout << " (_name==initialPoint)" << std::endl;
        std::cout << " (_value==" << _value << std::endl;
      }
      veh->setInitialPoint(_value);
    }
    elif(_name == finalPoint)
    {
      veh->setFinalPoint(_value);
    }
    elif(_name == vehicleType)
    {
      std::cout<<_value<<" vehicle type parsed"<<std::endl;
      veh->setVehicleType(_value);
    }
    elif(_name == behaviour)
    {
      veh->setBehaviour(_value);
    }
    elif(_name == overload)
    {
      veh->setOverload(_value);
    }
    // These other are dummy?
    /*
    elif(_name == widescreen_cons)
    {
      veh->setWidescreenCons(_value);
    }
    elif (_name == as_consumption)
    {
      veh->setAsConsumption(_value);
    }
    elif (_name == max_power)
    {
      veh->setMaxPower(_value);
    }
    */
    XMLString::release(&_name);
    XMLString::release(&_value);
  }
  vehicles.push_back(*veh);
  if(oc.getLoadVerbosity()>2)
    std::cout << "[500] MyHandler2::processAttribute(...)" << veh << std::endl;
  if(oc.getLoadVerbosity()>1)
    std::cout << "<---- MyHandler2::processAttribute(...)" << std::endl;    
}

/*   (doancea) - Here you need to add another argument (vehicle) to set the 
 * information to the vehicle.
 *   (uprego) - Now not because of 'current' vehicle hack */
/*void MyHandler2::processVehicleCharacteristicsAttributes
 (VehicleToTrack& veh, const Attributes& attrs)
{
  int length;
  if(OptionsCont::getOptions().getInt("loadVerbosity"))
    std::cout << "MyHandler2::processVehicleCharacteristicsAttributes(...) ----> " << std::endl;
  //std::ostream& out;
  if(OptionsCont::getOptions().getInt("loadVerbosity"))
    std::cout<<"attrs.getLength()="<<attrs.getLength()<<std::endl;
  //Gets the number of elements.
  length = attrs.getLength();
  //Process each attribute.
  for(int var = 0; var < length; ++var)
  {
    const XMLCh* attrName;
    attrName = attrs.getQName(var);
    const XMLCh* attrValue;
    attrValue = attrs.getValue(var);
    char* _name = xercesc::XMLString::transcode(attrName);
    char* _value = xercesc::XMLString::transcode(attrValue);
    //std::cout << _name << " : " << _value << std::endl;
    std::string id = "id";
    //dynamics.
    std::string mass = "mass";
    std::string coef_rolling_resistance = "coef_rolling_resistance";*/
//     std::string frontal_area = "frontal_area";
//     std::string coef_drag = "coef_drag";
//     //motor
//     std::string motor_eff = "motor_eff";
//     std::string gear_ratio = "gear_ratio";
//     std::string gear_system_eff = "gear_system_eff";
//     std::string regen_ratio = "regen_ratio";
//     //battery.
//     std::string cooper_loss_coef = "cooper_loss_coef";
//     std::string iron_loss_coef = "iron_loss_coef";
//     std::string windage_loss_coef = "windage_loss_coef";
//     std::string constant_loss = "constant_loss";
//     std::string heating_consumption = "heating_consumption";
//     std::string radio_consumption = "radio_consumption";
//     std::string lights = "lights";
//     std::string number_cells = "number_cells";
//     std::string peukert_coeff = "peukert_coeff";
//     std::string nominal_capacity = "nominal_capacity";
//     std::string time_charge = "time_charge";
//     std::string widescreen_cons = "widescreen_cons";
//     std::string as_consumption = "A_C_consumption";
//     std::string max_power = "max_power";
//     if(_name == id)
//     {
//       if(OptionsCont::getOptions().getInt("loadVerbosity"))
//       {
//         std::cout << " (_name==id)" << std::endl;
//         std::cout << " (_value==" << _value << std::endl;
//       }
//       veh.setIdentifier(_value);
//     }
    /* Dynamics. */
//     elif(_name == mass)
//     {
//       if(OptionsCont::getOptions().getInt("loadVerbosity")){
//         std::cout << " (_name==mass)" << std::endl;
//         std::cout << " (_value==" << _value << std::endl;
//       }     
//       veh.setMass(_value);
//     }
//     else if (_name == coef_rolling_resistance) {
//       veh.setCoefRollingResistance(_value);
//     }
//     else if (_name == frontal_area) {
//       veh.setFrontalArea(_value);
//     }
//     else if (_name == coef_drag) {
//       veh.setCoefDrag(_value);
//     }
    /* Motor. */
//     elif(_name == motor_eff)
//     {
//       if(OptionsCont::getOptions().getInt("loadVerbosity"))
//       {
//         std::cout << " (_name==motor_eff)" << std::endl;
//         std::cout << " (_value==" << _value << std::endl;
//       }      
//       veh.setMotorEff(_value);
//     }
//     else if (_name == gear_ratio) {
//       veh.setGearRatio(_value);
//     }
//     else if (_name == gear_system_eff) {
//       veh.setGearSystemEff(_value);
//     }
//     else if (_name == regen_ratio) {
//       veh.setRegenRatio(_value);
//     }
//     /* Battery. */
//     elif(_name == cooper_loss_coef)
//     {
//       if(OptionsCont::getOptions().getInt("loadVerbosity"))
//       {
//         std::cout << " (_name==cooper_loss_coef)" << std::endl;
//         std::cout << " (_value==" << _value << std::endl;
//       }      
//       veh.setCooperLossCoef(_value);
//     }
//     else if (_name == iron_loss_coef) {
//       veh.setIronLossCoef(_value);
//     }
//     else if (_name == windage_loss_coef) {
//       veh.setWindageLossCoef(_value);
//     }
//     else if (_name == constant_loss) {
//       veh.setConstantLoss(_value);
//     }
//     else if (_name == heating_consumption) {
//       veh.setHeatingConsumption(_value);
//     }
//     else if (_name == radio_consumption) {
//       veh.setRadioConsumption(_value);
//     }
//     else if (_name == lights) {
//       veh.setLights(_value);
//     }
//     elif(_name == number_cells)
//     {
//       if(OptionsCont::getOptions().getInt("loadVerbosity"))
//       {
//         std::cout << " (_name==number_cells" << std::endl;
//         std::cout << " (_value==" << _value << std::endl;
//       }
//       veh.setNumberCells(_value);
//     }
//     else if (_name == peukert_coeff) {
//       veh.setPeukertCoeff(_value);
//     }
//     else if (_name == nominal_capacity) {
//       veh.setNominalCapacity(_value);
//     }
//     else if (_name == time_charge) {
//       veh.setTimeCharge(_value);
//     }
//     else if (_name == widescreen_cons) {
//       veh.setWidescreenCons(_value);
//     }
//     else if (_name == as_consumption) {
//       veh.setAsConsumption(_value);
//     }
//     else if (_name == max_power) {
//       veh.setMaxPower(_value);
//     }
//     XMLString::release(&_name);
//     XMLString::release(&_value);
//   }
//   vehicles.push_back(veh);
//   if(OptionsCont::getOptions().getInt("loadVerbosity"))  
//     std::cout << "MyHandler2::processAttribute(...) [500] "
//               << "vehicle=" << veh
//               << std::endl;    
//   if(OptionsCont::getOptions().getInt("loadVerbosity"))
//     std::cout << "MyHandler2::processAttribute(...) <---- " << std::endl;    
// }

void MyHandler2::fatalError(const SAXParseException& exception)
{
  char* message = XMLString::transcode(exception.getMessage());
  std::cout << "Fatal Error: " << message << " at line: "
    << exception.getLineNumber() << std::endl;
  XMLString::release(&message);
}

void MyHandler2::showVehiclesToTrack(std::vector<VehicleToTrack>& vehicles)
{
  std::cout << "There are " << vehicles.size() << " vehicles\n"
    << "------Id--------Behaviour------\n";
  std::vector<VehicleToTrack>::const_iterator iter;
  for(iter = vehicles.begin(); iter != vehicles.end(); ++iter)
  {
    std::cout << (iter)->getIdentifier() << " | " <<(iter)->getBehaviour() << "\n";
  }
}

time_t *MyHandler2::getInitialTime(){return initialTime;}
  
void MyHandler2::setInitialTime(time_t *t)
{
  OptionsCont &oc = OptionsCont::getOptions();
  if(oc.getSimulationVerbosity()>1)
    std::cout<<"----> void MyHandler2::setInitialTime(...)"<<std::endl;
  initialTime = t;
}

std::string MyHandler2::getInitialTimeString(){return initialTimeString;}
  
void MyHandler2::setInitialTimeString(std::string its)
{
  initialTimeString=its;
}
