/****************************************************************************/
/// @file    MSXMLRawOut.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Sascha Krieg
/// @author  Bjoern Hendriks
/// @author  Michael Behrisch
/// @date    Mon, 10.05.2004
/// @version $Id: MSXMLRawOut.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// Realises dumping the complete network state
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

#include <cassert>
#include <microsim/MSEdgeControl.h>
#include <microsim/MSEdge.h>
#include <microsim/MSLane.h>
#include <microsim/MSGlobals.h>
#include <microsim/devices/MSDevice_FEV.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/common/SUMOTime.h>
#include "MSXMLRawOut.h"

#ifdef HAVE_MESOSIM
#include <mesosim/MELoop.h>
#include <mesosim/MESegment.h>
#endif

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
void
MSXMLRawOut::write(OutputDevice& of, const MSEdgeControl& ec,
                   SUMOTime timestep) {
    of.openTag("timestep") << " time=\"" << time2string(timestep) << "\">\n";
    const std::vector<MSEdge*> &edges = ec.getEdges();
    for (std::vector<MSEdge*>::const_iterator e = edges.begin(); e != edges.end(); ++e) {
        writeEdge(of, **e);
    }
    of.closeTag();
}


void
MSXMLRawOut::writeEdge(OutputDevice& of, const MSEdge& edge) {
    //en
    bool dump = !MSGlobals::gOmitEmptyEdgesOnDump;
    if (!dump) {
#ifdef HAVE_MESOSIM
        if (MSGlobals::gUseMesoSim) {
            MESegment* seg = MSGlobals::gMesoNet->getSegmentForEdge(edge);
            while (seg != 0) {
                if (seg->getCarNumber() != 0) {
                    dump = true;
                    break;
                }
                seg = seg->getNextSegment();
            }
        } else {
#endif
            const std::vector<MSLane*> &lanes = edge.getLanes();
            for (std::vector<MSLane*>::const_iterator lane = lanes.begin(); lane != lanes.end(); ++lane) {
                if (((**lane).getVehicleNumber() != 0)) {
                    dump = true;
                    break;
                }
            }
#ifdef HAVE_MESOSIM
        }
#endif
    }
    //en
    if (dump) {
        of.openTag("edge") << " id=\"" << edge.getID() << "\">\n";
#ifdef HAVE_MESOSIM
        if (MSGlobals::gUseMesoSim) {
            MESegment* seg = MSGlobals::gMesoNet->getSegmentForEdge(edge);
            while (seg != 0) {
                seg->writeVehicles(of);
                seg = seg->getNextSegment();
            }
        } else {
#endif
            const std::vector<MSLane*> &lanes = edge.getLanes();
            for (std::vector<MSLane*>::const_iterator lane = lanes.begin(); lane != lanes.end(); ++lane) {
                writeLane(of, **lane);
            }
#ifdef HAVE_MESOSIM
        }
#endif
        of.closeTag();
    }
}


void
MSXMLRawOut::writeLane(OutputDevice& of, const MSLane& lane) {
    of.openTag("lane") << " id=\"" << lane.myID << "\"";
    if (lane.getVehicleNumber() != 0) {
        of << ">\n";
        for (std::vector<MSVehicle*>::const_iterator veh = lane.myVehBuffer.begin();
                veh != lane.myVehBuffer.end(); ++veh) {
            writeVehicle(of, **veh);
        }
        for (MSLane::VehCont::const_iterator veh = lane.myVehicles.begin();
                veh != lane.myVehicles.end(); ++veh) {
            writeVehicle(of, **veh);
        }
    }
    of.closeTag(lane.getVehicleNumber() == 0);
}


void
MSXMLRawOut::writeVehicle(OutputDevice& of, const MSVehicle& veh) {

    if (veh.isOnRoad()) {

    	//SUMOTime time_on_lane = MSNet::getInstance()->getCurrentTimeStep()-veh.myState.enterLane;


    	SUMOTime time_on_lane = MSNet::getInstance()->getCurrentTimeStep()-veh.getMyState().getEnterLane();
    	double time = time_on_lane;
    	//if(time!=0){
    		time = time/1000.0;
    	double avg_speed = (veh.getPositionOnLane()- veh.getMyState().getMyPosOnLane())/time;
        of.openTag("vehicle") << " ida=\"" << veh.getID() << "\" pos=\""
                              << veh.getPositionOnLane() << "\" speed=\"" << veh.getSpeed() <<"\""
                              << " enter_time_on_lane=\"" << time2string(veh.getMyState().getEnterLane()) <<"\""
        					  << " time_on_lane =\"" << time2string(time_on_lane) << "\""
        					  << " enter_pos =\"" << veh.getMyState().getMyPosOnLane() <<"\""
        					  << " curr_pos =\"" << veh.getPositionOnLane() <<"\""
        					  << " avg_speed =\"" << avg_speed <<"\"";
        of.closeTag(true);
    	//}
    }
}



/****************************************************************************/

