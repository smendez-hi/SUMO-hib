/*
 * MyHandler2.h
 *
 *  Created on: Abril 9, 2012
 *      Author: doancea
 */

#ifndef MYHANDLER2_H_
#define MYHANDLER2_H_

#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <xercesc/sax2/DefaultHandler.hpp>
#include <xercesc/sax2/Attributes.hpp>
#include <vector>
#include "VehicleToTrack.h"

using namespace xercesc;

class MyHandler2: public DefaultHandler
{
 public:
  MyHandler2();

  void startElement
   (const XMLCh* const uri, const XMLCh* const localname,
    const XMLCh* const qname, const Attributes& attrs);
   
  void fatalError(const SAXParseException&);

  std::vector<VehicleToTrack> vehicles;

  void showVehiclesToTrack(std::vector<VehicleToTrack>& vehicles);
  
  static time_t *getInitialTime();
  
  static void setInitialTime(time_t *t);
  
  static std::string getInitialTimeString();
  
  static void setInitialTimeString(std::string its);
  
 private:
  void processVehicleToTrackAttributes
   (VehicleToTrack* vehicle, const Attributes& attrs);
  
  /*
  void processVehicleCharacteristicsAttributes
   (VehicleToTrack& vehicle, const Attributes& attrs);
  */
  
  void processAttribute(VehicleToTrack& vehicle, const Attributes& attrs);

  int find(std::vector<VehicleToTrack>& vec, const std::string& vehicle);
  
  int depth; /* hack to detect the different vehicle characteristics */
  
  std::string current; /* hack identifier of the current vehicle */
};
#endif /* MYHANDLER2_H_ */