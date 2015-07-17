/*
 * ScenParser.cpp
 *
 *  Created on: May 21, 2012
 *      Author: doancea
 */

#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <iostream>
#include <xercesc/sax2/SAX2XMLReader.hpp>
#include <xercesc/sax2/XMLReaderFactory.hpp>
#include <xercesc/sax2/DefaultHandler.hpp>
#include <xercesc/util/XMLString.hpp>
#include "utils/options/OptionsCont.h"
#include "ScenParser.h"
#include "MyHandler2.h"

/*
ScenParser::ScenParser
 ()
{}
*/

ScenParser::ScenParser
 (const char* filePath)
{
  
  if(OptionsCont::getOptions().getInt("loadVerbosity") > 0)
    std::cout << "----> ScenParser::ScenParser(filePath=" << filePath << ")\n";
  try
  {
    xercesc::XMLPlatformUtils::Initialize();
  }
  catch(const xercesc::XMLException& toCatch)
  {
    char* message = xercesc::XMLString::transcode(toCatch.getMessage());
    std::cout << " Error during initialization! :\n";
    std::cout << " Exception message is: \n" << message << "\n";
    xercesc::XMLString::release(&message);
  }
  SAX2XMLReader* parser = XMLReaderFactory::createXMLReader();
  parser->setFeature(xercesc::XMLUni::fgSAX2CoreValidation, true);
  parser->setFeature(xercesc::XMLUni::fgSAX2CoreNameSpaces, true); // optional
  MyHandler2* defaultHandler = new MyHandler2();
  parser->setContentHandler(defaultHandler);
  parser->setErrorHandler(defaultHandler);
  try
  {
    if(OptionsCont::getOptions().getInt("loadVerbosity") > 0)
      std::cout << "[010] ScenParser::ScenParser(filePath=" << filePath <<
                   ")\n";
    parser->parse(filePath);        
    std::vector<VehicleToTrack> result;
    if(OptionsCont::getOptions().getInt("loadVerbosity") > 0)
      std::cout << "[020] ScenParser::ScenParser(filePath=" << filePath <<
                   ")\n  result = defaultHandler->vehicles;" << std::endl;
    // ALERT (UXIO) LINE WAS COMMENTED IN ORIGINAL
    result = defaultHandler->vehicles;
    if(OptionsCont::getOptions().getInt("loadVerbosity") > 0)
    {
      int i=0;
      while(i<result.size())
      {
        std::cout << result.at(i);
        i++;
      }
    }
    if(OptionsCont::getOptions().getInt("loadVerbosity") > 0)
      std::cout << "[030] ScenParser::ScenParser(filePath=" << filePath <<
                   ")\n  result = defaultHandler->showVehiclesToTrack(result);"
                << std::endl;      
    // ALERT (UXIO) LINE WAS COMMENTED IN ORIGINAL
    defaultHandler->showVehiclesToTrack(result);        
  }
  catch(const xercesc::XMLException& toCatch)
  {
    char* message = xercesc::XMLString::transcode(toCatch.getMessage());
    std::cout << "Exception message is: \n" << message << "\n";
    xercesc::XMLString::release(&message);
  }
  catch (const xercesc::SAXParseException& toCatch)
  {
    char* message = xercesc::XMLString::transcode(toCatch.getMessage());
    std::cout << "Exception message is: \n" << message << "\n";
    xercesc::XMLString::release(&message);
  }
  catch(...)
  {
    std::cout << "Unexpected Exception \n";
  }
  delete parser;
  delete defaultHandler;
  //delete defaultHandler;    
  if(OptionsCont::getOptions().getInt("loadVerbosity") > 0)
    std::cout << "<---- ScenParser::ScenParser(filePath=" << filePath <<
                   ")" << std::endl;
  
}

ScenParser::~ScenParser(){}