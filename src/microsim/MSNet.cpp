/****************************************************************************/
/// @file    MSNet.cpp
/// @author  Christian Roessel
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Clemens Honomichl
/// @author  Eric Nicolay
/// @author  Michael Behrisch
/// @date    Tue, 06 Mar 2001
/// @version $Id: MSNet.cpp 11747 2012-01-20 08:29:46Z namdre $
///
// The simulated network and simulation perfomer
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

#ifdef HAVE_VERSION_H
#include <version.h>
#endif

#include <iostream>
#include <sstream>
#include <string.h>
#include <typeinfo>
#include <algorithm>
#include <cassert>
#include <vector>
#include <sstream>
#include <utils/common/UtilExceptions.h>
#include "MSNet.h"
#include "MSPersonControl.h"
#include "MSEdgeControl.h"
#include "MSJunctionControl.h"
#include "MSInsertionControl.h"
#include "MSEventControl.h"
#include "MSEdge.h"
#include "MSJunction.h"
#include "MSJunctionLogic.h"
#include "MSLane.h"
#include "MSVehicleTransfer.h"
#include "MSRoute.h"
#include "MSRouteLoaderControl.h"
#include "traffic_lights/MSTLLogicControl.h"
#include "MSVehicleControl.h"
#include <utils/common/MsgHandler.h>
#include <utils/common/ToString.h>
#include <microsim/output/MSDetectorControl.h>
#include <microsim/MSVehicleTransfer.h>
#include "traffic_lights/MSTrafficLightLogic.h"
#include <utils/shapes/Polygon.h>
#include <utils/shapes/ShapeContainer.h>
#include "output/MSXMLRawOut.h"
#include <utils/iodevices/OutputDevice.h>
#include <utils/common/SysUtils.h>
#include <utils/common/WrappingCommand.h>
#include <utils/options/OptionsCont.h>
#include "MSGlobals.h"
#include <utils/geom/GeoConvHelper.h>
#include <ctime>
#include "scenload/MyHandler2.h"
#include "MSPerson.h"
#include "MSEdgeWeightsStorage.h"

#ifdef _MESSAGES
#include "MSMessageEmitter.h"
#endif

#ifdef HAVE_MESOSIM
#include <mesosim/MELoop.h>
#include <utils/iodevices/BinaryInputDevice.h>
#endif

#ifndef NO_TRACI
#include <traci-server/TraCIServer.h>
#endif

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS

// ===========================================================================
// static member definitions
// ===========================================================================
MSNet* MSNet::myInstance = 0;

// ===========================================================================
// member method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// MSNet::EdgeWeightsProxi - methods
// ---------------------------------------------------------------------------
SUMOReal
MSNet::EdgeWeightsProxi::getEffort(const MSEdge* const e,
                                   const SUMOVehicle* const v,
                                   SUMOReal t) const {
    SUMOReal value;
    if (myVehicleKnowledge.retrieveExistingEffort(e, v, t, value)) {
        return value;
    }
    if (myNetKnowledge.retrieveExistingEffort(e, v, t, value)) {
        return value;
    }
    return 0;
}

SUMOReal
MSNet::EdgeWeightsProxi::getTravelTime(const MSEdge* const e,
                                       const SUMOVehicle* const v,
                                       SUMOReal t) const {
    SUMOReal value;
    if (myVehicleKnowledge.retrieveExistingTravelTime(e, v, t, value)) {
        return value;
    }
    if (myNetKnowledge.retrieveExistingTravelTime(e, v, t, value)) {
        return value;
    }
    const MSLane* const l = e->getLanes()[0];
    return l->getLength() / l->getMaxSpeed();
}

// ---------------------------------------------------------------------------
// MSNet - methods
// ---------------------------------------------------------------------------
MSNet*
MSNet::getInstance(void) {
    if (myInstance != 0) {
        return myInstance;
    }
    throw ProcessError("A network was not yet constructed.");
}

MSNet::MSNet(MSVehicleControl* vc, MSEventControl* beginOfTimestepEvents,
             MSEventControl* endOfTimestepEvents, MSEventControl* insertionEvents,
             ShapeContainer* shapeCont) {
    if (myInstance != 0) {
        throw ProcessError("A network was already constructed.");
    }
    OptionsCont& oc = OptionsCont::getOptions();
    myStep = string2time(oc.getString("begin"));
    myLogExecutionTime = !oc.getBool("no-duration-log");
    myLogStepNumber = !oc.getBool("no-step-log");
    myTooManyVehicles = oc.getInt("max-num-vehicles");
    myInserter = new MSInsertionControl(*vc, string2time(oc.getString("max-depart-delay")), oc.getBool("sloppy-insert"));
    myVehicleControl = vc;
    myDetectorControl = new MSDetectorControl();
    myEdges = 0;
    myJunctions = 0;
    myRouteLoaders = 0;
    myLogics = 0;
    myPersonControl = 0;
    myEdgeWeights = 0;
    myShapeContainer = shapeCont == 0 ? new ShapeContainer() : shapeCont;

    myBeginOfTimestepEvents = beginOfTimestepEvents;
    myEndOfTimestepEvents = endOfTimestepEvents;
    myInsertionEvents = insertionEvents;

#ifdef HAVE_MESOSIM
    if (MSGlobals::gUseMesoSim) {
        MSGlobals::gMesoNet = new MELoop(string2time(oc.getString("meso-recheck")));
    }
#endif
    myInstance = this;
    cityWasSet=false;
    srand((unsigned)time(NULL));
}

void MSNet::closeBuilding(MSEdgeControl* edges, MSJunctionControl* junctions,
                     MSRouteLoaderControl* routeLoaders,
                     MSTLLogicControl* tlc,
                     std::vector<SUMOTime> stateDumpTimes,
                     std::vector<std::string> stateDumpFiles) {
    myEdges = edges;
    myJunctions = junctions;
    myRouteLoaders = routeLoaders;
    myLogics = tlc;
    // save the time the network state shall be saved at
    myStateDumpTimes = stateDumpTimes;
    myStateDumpFiles = stateDumpFiles;

    // set requests/responses
    myJunctions->postloadInitContainer();

    // initialise performance computation
    if (myLogExecutionTime) {
        mySimBeginMillis = SysUtils::getCurrentMillis();
    }
}

MSNet::~MSNet() {
    // delete controls
    delete myJunctions;
    delete myDetectorControl;
    // delete mean data
    delete myEdges;
    delete myInserter;
    delete myLogics;
    delete myRouteLoaders;
    delete myVehicleControl;
    if (myPersonControl != 0) {
        delete myPersonControl;
    }
    delete myShapeContainer;
#ifdef _MESSAGES
    myMsgEmitter.clear();
    msgEmitVec.clear();
#endif
#ifdef HAVE_MESOSIM
    if (MSGlobals::gUseMesoSim) {
        delete MSGlobals::gMesoNet;
    }
#endif
    delete myBeginOfTimestepEvents;
    delete myEndOfTimestepEvents;
    delete myInsertionEvents;
    delete myEdgeWeights;
    clearAll();
    myInstance = 0;
}

int MSNet::extractYear(std::string dateTimeString)
{
  // 2006-05-04T18:13:52 alike
  return atoi(dateTimeString.substr(0,4).c_str());
}

int MSNet::extractMonth(std::string dateTimeString)
{
  // 2006-05-04T18:13:52 alike
  return atoi(dateTimeString.substr(5,2).c_str());
}

int MSNet::extractDay(std::string dateTimeString)
{
  // 2006-05-04T18:13:52 alike
  return atoi(dateTimeString.substr(8,2).c_str());
}

int MSNet::extractHour(std::string dateTimeString)
{
  // 2006-05-04T18:13:52 alike
  return atoi(dateTimeString.substr(11,2).c_str());
}

int MSNet::extractMinutes(std::string dateTimeString)
{
  // 2006-05-04T18:13:52 alike
  atoi(dateTimeString.substr(14,2).c_str());
}

int MSNet::extractSeconds(std::string dateTimeString)
{
  // 2006-05-04T18:13:52 alike
  return atoi(dateTimeString.substr(17,2).c_str());
}

bool MSNet::isNight(std::string dateTimeString)
{
  int month=extractMonth(dateTimeString),
   hour=extractHour(dateTimeString);
  if(month<3||month>11)
  {
    // December, January or February
    return((hour>=18)&&(hour<8));
  }
  else if(month<6)
  {
    // March, April or May
    return((hour>=19)&&(hour<7));
  }
  else if(month<9)
  {
    // June, July or August
    return((hour>=20)&&(hour<7));
  }
  else
  {
    // September, October or November
    return((hour>=19)&&(hour<7));
  }
}

bool MSNet::isWinter()
{
  OptionsCont &oc=OptionsCont::getOptions();
  std::string dateTimeString=MyHandler2::getInitialTimeString();
  int month=extractMonth(dateTimeString);
  if(month<3||month>11)
  {
    // December, January or February
    return true;
  }
  else if(month<6)
  {
    // March, April or May
    return false;
  }
  else if(month<9)
  {
    // June, July or August
    return true;
  }
  else
  {
    // September, October or November
    return false;
  }
}

bool MSNet::isSummer()
{
  return !isWinter();
}

void MSNet::updateWeather()
{
  OptionsCont &oc = OptionsCont::getOptions();
  std::string currentTimeString, initialTimeString =
   MyHandler2::getInitialTimeString();
  MSNet *net = MSNet::getInstance();
  if(oc.getAnyVerbosity()>1)
    std::cout<<"----> void MSNet::updateWeather()"<<std::endl;
  if(oc.getAnyVerbosity()>2)
    std::cout<<"initialTimeString{"<<initialTimeString<<"}"<<std::endl;
  int tempTendingTo;
  if(oc.getSafeBool("rlwc"))
  {
    // Update time string to catch night or day becoming
    currentTimeString = net->makeTimeString(net->getCurrentTimeStep());
    if(getCity()=="cambiano")
    {
      // Real Life simulation envtemp update
      // Month depending analysis
      if(extractMonth(initialTimeString) < 3 ||
       extractMonth(initialTimeString) > 11)
      {
        // December, January or February
        if(net->isNight(currentTimeString))
          tempTendingTo = 0;
        else
          tempTendingTo = 5;
      }
      else if(extractMonth(initialTimeString)<6)
      {
        // March, April or May
        if(net->isNight(currentTimeString))
          tempTendingTo = 10;
        else
          tempTendingTo = 15;
      }
      else if(extractMonth(initialTimeString)<9)
      {
        // June, July or August
        if(net->isNight(currentTimeString))
          tempTendingTo = 15;
        else
          tempTendingTo = 20;
      }
      else
      {
        // September, October or November
        if(net->isNight(currentTimeString))
          tempTendingTo = 5;
        else
          tempTendingTo = 10;
      }
      // Real Life simulation envhum update
      if(rand()%10==0)
      {
        rand()%2==0?
        setCurrentEnvHum(getCurrentEnvHum() + ((float)rand()) / RAND_MAX) :
        setCurrentEnvHum(getCurrentEnvHum() - ((float)rand()) / RAND_MAX);
      }
    }
    else if(getCity()=="munchen")
    {
      // Real Life simulation envtemp update
      // Month depending analysis
      if((extractMonth(initialTimeString)<3)||
          extractMonth(initialTimeString)>11)
      {
        // December, January or February
        if(net->isNight(currentTimeString))
          tempTendingTo = 0;
        else
          tempTendingTo = 5;
      }
      else if(extractMonth(initialTimeString)<6)
      {
        // March, April or May
        if(net->isNight(currentTimeString))
          tempTendingTo = 10;
        else
          tempTendingTo = 15;
      }
      else if(extractMonth(initialTimeString)<9)
      {
        // June, July or August
        if(net->isNight(currentTimeString))
          tempTendingTo = 15;
        else
          tempTendingTo = 20;
      }
      else
      {
        // September, October or November
        if(net->isNight(currentTimeString))
          tempTendingTo = 5;
        else
          tempTendingTo = 10;
      }
      // Real Life simulation envhum update
      if(rand() % 10 == 0)
      {
        (rand() % 2 == 0) ?
        setCurrentEnvHum(getCurrentEnvHum() + ((float)rand()) / RAND_MAX) :
        setCurrentEnvHum(getCurrentEnvHum() - ((float)rand()) / RAND_MAX);
      }
    }
    // Envtemp update
    if(rand() % 10 == 0)
    {
    if(rand() % 2 == 0)
    {
      // Adding
      if(getCurrentEnvTemp() > tempTendingTo)
      {
        rand() % 3 == 0 ?
        setCurrentEnvTemp(getCurrentEnvTemp() + ((float)rand()) / RAND_MAX) :
        setCurrentEnvTemp(getCurrentEnvTemp() - ((float)rand()) / RAND_MAX);
      }
      else
      {
        rand() % 2 == 0 ?
        setCurrentEnvTemp(getCurrentEnvTemp() + ((float)rand()) / RAND_MAX) :
        setCurrentEnvTemp(getCurrentEnvTemp() - ((float)rand()) / RAND_MAX);
      }
    }
    else
    {
      // Substracting
      if(getCurrentEnvTemp() > tempTendingTo)
      {
        rand() % 2 == 0 ?
        setCurrentEnvTemp(getCurrentEnvTemp() + ((float)rand()) / RAND_MAX) :
        setCurrentEnvTemp(getCurrentEnvTemp() - ((float)rand()) / RAND_MAX);
      }
      else
      {
        rand() % 3 == 0 ?
        setCurrentEnvTemp(getCurrentEnvTemp() - ((float)rand()) / RAND_MAX) :
        setCurrentEnvTemp(getCurrentEnvTemp() + ((float)rand()) / RAND_MAX);
      }
    }
    }    
  }
  else if((oc.getSafeBool("rwc") ||
          (oc.getSafeBool("rlwc") && (getCity() == "unknown"))))
  {
   tag:
    // Random envtemp update
    if(rand() % 10 == 0)
    {
      (rand() % 2 == 0) ?
      setCurrentEnvTemp(getCurrentEnvTemp() + ((float)rand())/RAND_MAX) :
      setCurrentEnvTemp(getCurrentEnvTemp() - ((float)rand())/RAND_MAX);
    }
    // Random envhum update
    if(rand()%10==0)
    {
      (rand()%2==0)?
      setCurrentEnvHum(getCurrentEnvHum()+((float)rand())/RAND_MAX) :
      setCurrentEnvHum(getCurrentEnvHum()-((float)rand())/RAND_MAX);
    }
  }
  // else weather is constant
}

int MSNet::simulate(SUMOTime start, SUMOTime stop)
{
  OptionsCont &oc = OptionsCont::getOptions();
  if(oc.getSimulationVerbosity()>1)
    std::cout<<"----> void MSNet::simulate(...)"<<std::endl;  
  //if((oc.getString("net-file").find("munchen"))||
  // (oc.getString("net-file").find("munich")))
  if(oc.getSafeBool("ger"))
    setCity("munchen");
  //else if(oc.getString("net-file").find("cambiano"))
  else if(oc.getSafeBool("ita"))
    setCity("cambiano");
  else
    setCity("unknown");
  // the simulation loop
  MSNet::SimulationState state = SIMSTATE_RUNNING;
  myStep = start;
#ifndef NO_TRACI
#ifdef HAVE_PYTHON
  if(OptionsCont::getOptions().isSet("python-script")) {
    traci::TraCIServer::runEmbedded(OptionsCont::getOptions().getString("python-script"));
    WRITE_MESSAGE("Simulation End: Script ended");
    closeSimulation(start);
    return 0;
  }
#endif
#endif
  // Initialize weather conditions
  if(oc.isSet("fiet"))
    setCurrentEnvTemp(oc.getFloat("fiet"));
  else
    setCurrentEnvTemp(5.3);
  if(oc.isSet("fieh"))
    setCurrentEnvHum(oc.getFloat("fieh"));
  else
    setCurrentEnvHum(67.8);
  while(state == SIMSTATE_RUNNING)
  {
    if(myLogStepNumber)
        preSimStepOutput();
    simulationStep();
    // FIXME Custom wait
    //for(int i=0;i++<0x00003fff;)for(int j=0;j++<0x00000fff;);
    if(oc.getSafeBool("rwc") || oc.getSafeBool("rlwc"))
      updateWeather();
    if(myLogStepNumber)
        postSimStepOutput();
    state = simulationState(stop);
#ifndef NO_TRACI
    if(state != SIMSTATE_RUNNING)
    {
      if(OptionsCont::getOptions().getInt("remote-port") != 0 && !traci::TraCIServer::wasClosed())
        state = SIMSTATE_RUNNING;
    }
#endif
  }
  WRITE_MESSAGE("Simulation End: " + getStateMessage(state));
  // exit simulation loop
  closeSimulation(start);
  return 0;
}

void MSNet::closeSimulation(SUMOTime start)
{
    if (myLogExecutionTime) {
        long duration = SysUtils::getCurrentMillis() - mySimBeginMillis;
        std::ostringstream msg;
        msg << "Performance: " << "\n" << " Duration: " << duration << " ms" << "\n";
        if (duration != 0) {
            msg << " Real time factor: " << (STEPS2TIME(myStep - start) * 1000. / (SUMOReal)duration) << "\n";
            msg.setf(std::ios::fixed , std::ios::floatfield);    // use decimal format
            msg.setf(std::ios::showpoint);    // print decimal point
            msg << " UPS: " << ((SUMOReal) myVehiclesMoved * 1000. / (SUMOReal) duration) << "\n";
        }
        const std::string scaleNotice = ((myVehicleControl->getLoadedVehicleNo() != myVehicleControl->getDepartedVehicleNo()) ?
                " (Loaded: " + toString(myVehicleControl->getLoadedVehicleNo()) + ")" : "");
        msg << "Vehicles: " << "\n"
            << " Emitted: " << myVehicleControl->getDepartedVehicleNo() << scaleNotice << "\n"
            << " Running: " << myVehicleControl->getRunningVehicleNo() << "\n"
            << " Waiting: " << myInserter->getWaitingVehicleNo() << "\n";
        WRITE_MESSAGE(msg.str());
    }
    myDetectorControl->close(myStep);
#ifndef NO_TRACI
    traci::TraCIServer::close();
#endif
}

void MSNet::simulationStep()
{
  OptionsCont &oc = OptionsCont::getOptions();
  if(oc.getSimulationVerbosity()>1)
  {
    std::cout<<"----> void MSNet::simulationStep()"<<std::endl;
    //std::cout<<"std::string::max_size()="<<std::string::max_size()<<std::endl; // Why doesn't work?
  }
#ifndef NO_TRACI
    traci::TraCIServer::processCommandsUntilSimStep(myStep);
#endif
    // execute beginOfTimestepEvents
    if (myLogExecutionTime) {
        mySimStepBegin = SysUtils::getCurrentMillis();
    }
#ifdef HAVE_MESOSIM
    // netstate output
    std::vector<SUMOTime>::iterator timeIt = find(myStateDumpTimes.begin(), myStateDumpTimes.end(), myStep);
    if (timeIt != myStateDumpTimes.end()) {
        const int dist = distance(myStateDumpTimes.begin(), timeIt);
        std::ofstream strm(myStateDumpFiles[dist].c_str(), std::fstream::out | std::fstream::binary);
        saveState(strm);
    }
#endif
    myBeginOfTimestepEvents->execute(myStep);
    if (MSGlobals::gCheck4Accidents) {
        myEdges->detectCollisions(myStep);
    }
    // check whether the tls programs need to be switched
    myLogics->check2Switch(myStep);
    // set the signals
    myLogics->setTrafficLightSignals(myStep);

#ifdef HAVE_MESOSIM
    if (MSGlobals::gUseMesoSim) {
        MSGlobals::gMesoNet->simulate(myStep);
    } else {
#endif

        // assure all lanes with vehicles are 'active'
        myEdges->patchActiveLanes();

        // move vehicles
        //  precompute possible positions for vehicles that do interact with
        //   their lane's end
        myEdges->moveCritical(myStep);

        // move vehicles which do interact with their lane's end
        //  (it is now known whether they may drive
        myEdges->moveFirst(myStep);
        if (MSGlobals::gCheck4Accidents) {
            myEdges->detectCollisions(myStep);
        }

        // Vehicles change Lanes (maybe)
        myEdges->changeLanes(myStep);

        if (MSGlobals::gCheck4Accidents) {
            myEdges->detectCollisions(myStep);
        }
#ifdef HAVE_MESOSIM
    }
#endif
    // load routes
    myRouteLoaders->loadNext(myStep);

    // persons
    if (myPersonControl != 0) {
        myPersonControl->checkArrivedPersons(this, myStep);
    }
    // emit Vehicles
    myInsertionEvents->execute(myStep);
    myInserter->emitVehicles(myStep);
    if (MSGlobals::gCheck4Accidents) {
        myEdges->detectCollisions(myStep);
    }
    MSVehicleTransfer::getInstance()->checkInsertions(myStep);

    // execute endOfTimestepEvents
    myEndOfTimestepEvents->execute(myStep);

    // update and write (if needed) detector values
    writeOutput();

    if (myLogExecutionTime) {
        mySimStepEnd = SysUtils::getCurrentMillis();
        mySimStepDuration = mySimStepEnd - mySimStepBegin;
        myVehiclesMoved += myVehicleControl->getRunningVehicleNo();
    }
    myStep += DELTA_T;
}


MSNet::SimulationState
MSNet::simulationState(SUMOTime stopTime) const {
    if (myTooManyVehicles > 0 && (int) myVehicleControl->getRunningVehicleNo() > myTooManyVehicles) {
        return SIMSTATE_TOO_MANY_VEHICLES;
    }
#ifndef NO_TRACI
    if (traci::TraCIServer::wasClosed()) {
        return SIMSTATE_CONNECTION_CLOSED;
    }
    if (stopTime < 0 && OptionsCont::getOptions().getInt("remote-port") == 0) {
#else
    if (stopTime < 0) {
#endif
        if (myInsertionEvents->isEmpty()
                && (myVehicleControl->getActiveVehicleCount() == 0)
                && (myInserter->getPendingFlowCount() == 0)
                && (myPersonControl == 0 || !myPersonControl->hasPedestrians())) {
            if (myPersonControl) {
                myPersonControl->abortWaiting();
            }
            myVehicleControl->abortWaiting();
            return SIMSTATE_NO_FURTHER_VEHICLES;
        }
    }
    if (stopTime >= 0 && myStep >= stopTime) {
        return SIMSTATE_END_STEP_REACHED;
    }
    return SIMSTATE_RUNNING;
}

std::string MSNet::getStateMessage(MSNet::SimulationState state)
{
  switch (state)
  {
   case MSNet::SIMSTATE_RUNNING:
    return "";
   case MSNet::SIMSTATE_END_STEP_REACHED:
    return "The final simulation step has been reached.";
   case MSNet::SIMSTATE_NO_FURTHER_VEHICLES:
    return "All vehicles have left the simulation.";
   case MSNet::SIMSTATE_CONNECTION_CLOSED:
    return "TraCI requested termination.";
   case MSNet::SIMSTATE_ERROR_IN_SIM:
    return "An error occured (see log).";
   case MSNet::SIMSTATE_TOO_MANY_VEHICLES:
    return "Too many vehicles.";
   default:
    return "Unknown reason.";
  }
}

void MSNet::clearAll()
{
  // clear container
  MSEdge::clear();
  MSLane::clear();
  MSRoute::clear();
  delete MSVehicleTransfer::getInstance();
}

struct tm *timeStringToTm(std::string timeString)
{
  struct tm *res=(struct tm *)malloc(sizeof(struct tm));
  res->tm_sec=MSNet::getInstance()->extractSeconds(timeString);
  res->tm_min=MSNet::getInstance()->extractMinutes(timeString);
  res->tm_hour=MSNet::getInstance()->extractHour(timeString);
  res->tm_mday=MSNet::getInstance()->extractDay(timeString);
  res->tm_mon=MSNet::getInstance()->extractMonth(timeString);
  res->tm_year=MSNet::getInstance()->extractYear(timeString);
  return res;
}

std::string MSNet::makeTimeString(SUMOTime timeStep)
{
  /*
  REVISAR ESTA FUNCION, PORQUE DA ESTE FALLO:
  ==10363== 
  ==10363== 
  ==10363== 28751 errors in context 14 of 17:
  ==10363== Conditional jump or move depends on uninitialised value(s)
  ==10363==    at 0x60B6C4E: __mktime_internal (mktime.c:428)
  ==10363==    by 0x497F5D: MSNet::makeTimeString(int) (MSNet.cpp:676)
  ==10363==    by 0x51F6DB: MSDevice_FEV::computeElectricAuxiliaries() (MSDevice_FEV.cpp:1128)
  ==10363==    by 0x51BDD1: MSDevice_FEV::notifyMove(SUMOVehicle&, double, double, double) (MSDevice_FEV.cpp:362)
  ==10363==    by 0x4C6CC7: MSVehicle::workOnMoveReminders(double, double, double) (MSVehicle.cpp:465)
  ==10363==    by 0x4C9BA6: MSVehicle::moveChecked() (MSVehicle.cpp:1029)
  ==10363==    by 0x476979: MSLane::setCritical(int, std::vector<MSLane*, std::allocator<MSLane*> >&) (MSLane.cpp:732)
  ==10363==    by 0x456F4A: MSEdgeControl::moveFirst(int) (MSEdgeControl.cpp:120)
  ==10363==    by 0x4975CC: MSNet::simulationStep() (MSNet.cpp:552)
  ==10363==    by 0x496B8F: MSNet::simulate(int, int) (MSNet.cpp:454)
  ==10363==    by 0x40BA16: main (sumo_main.cpp:197)
  ==10363== 
  ==10363== 
  ==10363== 29266 errors in context 15 of 17:
  ==10363== Invalid free() / delete / delete[]
  ==10363==    at 0x4C240FD: free (vg_replace_malloc.c:366)
  ==10363==    by 0x497FB8: MSNet::makeTimeString(int) (MSNet.cpp:681)
  ==10363==    by 0x51F6DB: MSDevice_FEV::computeElectricAuxiliaries() (MSDevice_FEV.cpp:1128)
  ==10363==    by 0x51BDD1: MSDevice_FEV::notifyMove(SUMOVehicle&, double, double, double) (MSDevice_FEV.cpp:362)
  ==10363==    by 0x4C6CC7: MSVehicle::workOnMoveReminders(double, double, double) (MSVehicle.cpp:465)
  ==10363==    by 0x4C9BA6: MSVehicle::moveChecked() (MSVehicle.cpp:1029)
  ==10363==    by 0x476979: MSLane::setCritical(int, std::vector<MSLane*, std::allocator<MSLane*> >&) (MSLane.cpp:732)
  ==10363==    by 0x456F4A: MSEdgeControl::moveFirst(int) (MSEdgeControl.cpp:120)
  ==10363==    by 0x4975CC: MSNet::simulationStep() (MSNet.cpp:552)
  ==10363==    by 0x496B8F: MSNet::simulate(int, int) (MSNet.cpp:454)
  ==10363==    by 0x40BA16: main (sumo_main.cpp:197)
  ==10363==  Address 0x6386380 is 0 bytes inside data symbol "_tmbuf"
  ==10363== 
  ==10363== 
  */
  // Format used is 2006-05-04T18:13:52
  std::string returning, initialTimeString=MyHandler2::getInitialTimeString();
  char *returningCharArray=(char *)malloc(128);
  struct tm *tm1, *tm2=(struct tm *)malloc(sizeof(struct tm));
  time_t timeT;
  tm1=timeStringToTm(initialTimeString);
  memset(&timeT,0,sizeof(time_t));
  memset(tm1,0,sizeof(struct tm));
  memset(tm2,0,sizeof(struct tm));
  timeT=mktime(tm1);
  timeT+=timeStep;
  tm2=gmtime(&timeT);
  strftime(returningCharArray,128,"%Y-%m-%dT%H:%M:%S",tm2);
  free(tm1);
  //   tm2 no se libera, porque gmtime(...) y localtime(...) devuelven una es-
  // tructura que tienen interna y que usan todas las llamadas a esta funcion.
  //free(tm2);
  returning = strdup(returningCharArray);
  free(returningCharArray); // FIXME TODO
  return returning;
}

SUMOTime MSNet::getCurrentTimeStep()const
{
  return myStep;
}

void
MSNet::writeOutput() {
    // update detector values
    myDetectorControl->updateDetectors(myStep);
    // check state dumps

    // If you uncomment this, you dirt the file with the netstate-dump info (UXIO)
    /*if (OptionsCont::getOptions().isSet("netstate-dump-fev")) {
            MSXMLRawOut::writeFev(OutputDevice::getDeviceByOption("netstate-dump-fev"), *myEdges, myStep);
    }*/ 
    
    // FIXME (UXIO) THIS BLOCK WAS COMMENTED IN THE ORIGINAL FOR NOT MAKING A NETSTATE DUMP AT ALL
    if (OptionsCont::getOptions().isSet("netstate-dump")) {
        MSXMLRawOut::write(OutputDevice::getDeviceByOption("netstate-dump"), *myEdges, myStep);
    }
    // emission output
    if (OptionsCont::getOptions().isSet("summary-output")) {
        OutputDevice& od = OutputDevice::getDeviceByOption("summary");
        od << "    <step time=\"" << time2string(myStep) << "\" "
           << "loaded=\"" << myVehicleControl->getLoadedVehicleNo() << "\" "
           << "emitted=\"" << myVehicleControl->getDepartedVehicleNo() << "\" "
           << "running=\"" << myVehicleControl->getRunningVehicleNo() << "\" "
           << "waiting=\"" << myInserter->getWaitingVehicleNo() << "\" "
           << "ended=\"" << myVehicleControl->getEndedVehicleNo() << "\" "
           << "meanWaitingTime=\"";
        myVehicleControl->printMeanWaitingTime(od);
        od << "\" meanTravelTime=\"";
        myVehicleControl->printMeanTravelTime(od);
        od << "\" ";
        if (myLogExecutionTime) {
            od << "duration=\"" << mySimStepDuration << "\" ";
        }
        od << "/>\n";
    }
    // write detector values
    myDetectorControl->writeOutput(myStep + DELTA_T, false);
}


bool
MSNet::logSimulationDuration() const {
    return myLogExecutionTime;
}


#ifdef HAVE_MESOSIM
void
MSNet::saveState(std::ostream& os) {
    FileHelpers::writeString(os, VERSION_STRING);
    FileHelpers::writeUInt(os, sizeof(size_t));
    FileHelpers::writeUInt(os, sizeof(SUMOReal));
    FileHelpers::writeUInt(os, MSEdge::dictSize());
    FileHelpers::writeUInt(os, myStep);
    MSRoute::dict_saveState(os);
    myVehicleControl->saveState(os);
    if (MSGlobals::gUseMesoSim) {
        MSGlobals::gMesoNet->saveState(os);
    }
}


unsigned int
MSNet::loadState(BinaryInputDevice& bis) {
    std::string version;
    unsigned int sizeT, fpSize, numEdges, step;
    bis >> version;
    bis >> sizeT;
    bis >> fpSize;
    bis >> numEdges;
    bis >> step;
    if (version != VERSION_STRING) {
        WRITE_WARNING("State was written with sumo version " + version + " (present: " + VERSION_STRING + ")!");
    }
    if (sizeT != sizeof(size_t)) {
        WRITE_WARNING("State was written on a different platform (32bit vs. 64bit)!");
    }
    if (fpSize != sizeof(SUMOReal)) {
        WRITE_WARNING("State was written with a different precision for SUMOReal!");
    }
    if (numEdges != MSEdge::dictSize()) {
        WRITE_WARNING("State was written for a different net!");
    }
    const SUMOTime offset = string2time(OptionsCont::getOptions().getString("load-state.offset"));
    MSRoute::dict_loadState(bis);
    myVehicleControl->loadState(bis, offset);
    if (MSGlobals::gUseMesoSim) {
        MSGlobals::gMesoNet->loadState(bis, *myVehicleControl, offset);
    }
    return step;
}
#endif


MSPersonControl& MSNet::getPersonControl()
{
    if (myPersonControl == 0) {
        myPersonControl = new MSPersonControl();
    }
    return *myPersonControl;
}

MSEdgeWeightsStorage& MSNet::getWeightsStorage()
{
    if (myEdgeWeights == 0) {
        myEdgeWeights = new MSEdgeWeightsStorage();
    }
    return *myEdgeWeights;
}

void MSNet::preSimStepOutput() const
{
  OptionsCont &oc = OptionsCont::getOptions();
  if(oc.getSimulationVerbosity()>1)
    std::cout<<"----> void MSNet::preSimStepOutput() const"<<std::endl;
  std::cout << std::setprecision(OUTPUT_ACCURACY);
  std::cout << "Step #" << time2string(myStep);
}

void MSNet::postSimStepOutput() const
{
  OptionsCont &oc = OptionsCont::getOptions();
  if(oc.getSimulationVerbosity()>1)
    std::cout<<"----> void MSNet::postSimStepOutput() const"<<std::endl;
  if (myLogExecutionTime) {
        std::string msg;
        std::ostringstream oss;
        oss.setf(std::ios::fixed , std::ios::floatfield);    // use decimal format
        oss.setf(std::ios::showpoint);    // print decimal point
        oss << std::setprecision(OUTPUT_ACCURACY);
        if (mySimStepDuration != 0) {
            oss << " (" << mySimStepDuration << "ms ~= "
                << (1000. / (SUMOReal) mySimStepDuration) << "*RT, ~"
                << ((SUMOReal) myVehicleControl->getRunningVehicleNo() / (SUMOReal) mySimStepDuration * 1000.);
        } else {
            oss << " (0ms ?*RT. ?";
        }
        oss << "UPS, vehicles"
            << " TOT " << myVehicleControl->getDepartedVehicleNo()
            << " ACT " << myVehicleControl->getRunningVehicleNo()
            << ")                                              ";
        msg = oss.str();
        std::string prev = "Step #" + time2string(myStep - DELTA_T);
        msg = msg.substr(0, 78 - prev.length());
        std::cout << msg;
  }
  std::cout << (char) 13;
}

void MSNet::addVehicleStateListener(VehicleStateListener* listener) {
    if (find(myVehicleStateListeners.begin(), myVehicleStateListeners.end(), listener) == myVehicleStateListeners.end()) {
        myVehicleStateListeners.push_back(listener);
    }
}

void MSNet::removeVehicleStateListener(VehicleStateListener* listener) {
    std::vector<VehicleStateListener*>::iterator i = find(myVehicleStateListeners.begin(), myVehicleStateListeners.end(), listener);
    if (i != myVehicleStateListeners.end()) {
        myVehicleStateListeners.erase(i);
    }
}

void MSNet::informVehicleStateListener(const SUMOVehicle* const vehicle, VehicleState to) {
    for (std::vector<VehicleStateListener*>::iterator i = myVehicleStateListeners.begin(); i != myVehicleStateListeners.end(); ++i) {
        (*i)->vehicleStateChanged(vehicle, to);
    }
}

bool MSNet::checkSlopesSet()
{
  int i, positiveSlopes, negativeSlopes, zeroSlopes, zeroInternalSlopes, nanSlopes;
  std::vector < MSEdge * > edges;
  OptionsCont &oc = OptionsCont::getOptions();
  edges = getEdgeControl().getEdges();
  if(oc.getBuildVerbosity()>1)
    std::cout<<"----> bool MSNet::checkSlopesSet()"<<std::endl;
  positiveSlopes = negativeSlopes = zeroSlopes = zeroInternalSlopes = nanSlopes = 0;
  for(i=0;i<edges.size();i++)
  {
    if(!edges.at(i)->getSlopeSetStatus())
    {
      if(oc.getBuildVerbosity()>2)
        std::cout<<"Edge "<<edges.at(i)->getID()<<" was not set any slope at all"<<std::endl;
    }
    else
    {
      if(oc.getBuildVerbosity()>2)
        std::cout<<"Edge "<<edges.at(i)->getID()<<" has been set a slope as it has to be"<<std::endl;
    }
    if(edges.at(i)->getSlope() > 0)
      positiveSlopes++;
    else if(edges.at(i)->getSlope() < 0)
      negativeSlopes++;
    else if(edges.at(i)->getSlope() == std::numeric_limits<double>::quiet_NaN/*infinity*/())
      nanSlopes++;
    else
    {
      zeroSlopes++;
      if(edges.at(i)->getID().at(0)==':')
        zeroInternalSlopes++;
    }
    /*
    std::cout<<"CRITICAL whilein bool MSNet::checkSlopesSet()"<<std::endl;
    exit(0xfffffff);
    */
    //std::cout<<"[100] edges.at(i)->getSlope()="<<edges.at(i)->getSlope()<<std::endl;
  }
  std::cout<<"Net checked:"<<std::endl;
  std::cout<<" got "<<positiveSlopes<<" positive sloped edges"<<std::endl;
  std::cout<<" got "<<zeroSlopes<<" zero sloped edges"<<std::endl;
  std::cout<<"  got "<<zeroInternalSlopes<<" internal zero sloped edges"<<std::endl;
  std::cout<<"  got "<<zeroSlopes-zeroInternalSlopes<<" noninternal zero sloped edges"<<std::endl;
  std::cout<<" got "<<negativeSlopes<<" negative sloped edges"<<std::endl;
  std::cout<<" got "<<nanSlopes<<" NaN (unsetted) sloped edges"<<std::endl;  
}
  
SUMOReal MSNet::getCurrentEnvHum(){return currentEnvHum;}

void MSNet::setCurrentEnvHum(SUMOReal ceh)
{
  if(ceh < 0)
    currentEnvHum = 0;
  else if(ceh > 100)
    currentEnvHum = 100;
  else
    currentEnvHum=ceh;
}

SUMOReal MSNet::getCurrentEnvTemp(){return currentEnvTemp;}

void MSNet::setCurrentEnvTemp(SUMOReal cet){currentEnvTemp=cet;}

std::string MSNet::getCity()
{
  if(cityWasSet)return city;
  else
  {
    std::cout<<"CRITICAL in std::string MSNet::getcity()"<<std::endl;
    std::cout<<"  (city was not set)"<<std::endl;
    exit(1);
  }
}

void MSNet::setCity(const char* c){city=c;cityWasSet=true;}

// ------ Insertion and retrieval of bus stops ------
bool MSNet::addBusStop(MSBusStop* busStop) {
  return myBusStopDict.add(busStop->getID(), busStop);
}

MSBusStop* MSNet::getBusStop(const std::string& id) const {
  return myBusStopDict.get(id);
}

std::string MSNet::getBusStopID(const MSLane* lane, const SUMOReal pos) const {
  const std::map<std::string, MSBusStop*> &vals = myBusStopDict.getMyMap();
  for(std::map<std::string, MSBusStop*>::const_iterator it = vals.begin(); it != vals.end(); ++it){
    MSBusStop* stop = it->second;
    if(&stop->getLane() == lane && fabs(stop->getEndLanePosition() - pos) < POSITION_EPS){
      return stop->getID();
    }
  }
  return "";
}

#ifdef _MESSAGES
MSMessageEmitter* MSNet::getMsgEmitter(const std::string& whatemit) {
  msgEmitVec.clear();
  msgEmitVec = myMsgEmitter.buildAndGetStaticVector();
  MSMessageEmitter* msgEmitter = 0;
  for(int i = 0; i < msgEmitVec.size(); ++i){
    if(msgEmitVec.at(i)->getEventsEnabled(whatemit)){
      msgEmitter = msgEmitVec.at(i);
      break;
    }
  }
  // returns 0 if the requested MessageEmitter is not in the map
  return msgEmitter;
}

void MSNet::createMsgEmitter
 (std::string& id, std::string& file, const std::string& base,
  std::string& whatemit, bool reverse, bool table, bool xy, SUMOReal step)
{
  MSMessageEmitter* msgEmitter = new MSMessageEmitter(file, base, whatemit, reverse, table, xy, step);
  myMsgEmitter.add(id, msgEmitter);
}
#endif
