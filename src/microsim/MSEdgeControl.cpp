/****************************************************************************/
/// @file    MSEdgeControl.cpp
/// @author  Christian Roessel
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Mon, 09 Apr 2001
/// @version $Id: MSEdgeControl.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// Stores edges and lanes, performs moving of vehicle
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

#include "utils/options/OptionsCont.h"
#include "MSEdgeControl.h"
#include "MSEdge.h"
#include "MSLane.h"
#include "MSJunctionControl.h"
#include "MSLogicJunction.h"
#include "MSNoLogicJunction.h"
#include "MSRightOfWayJunction.h"
#include <iostream>
#include <cmath>
#include <vector>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// member method definitions
// ===========================================================================
MSEdgeControl::MSEdgeControl(const std::vector< MSEdge* > &edges)
    : myEdges(edges),
      myLanes(MSLane::dictSize()),
      myLastLaneChange(MSEdge::dictSize()) {
    // build the usage definitions for lanes
    for (std::vector< MSEdge* >::const_iterator i = myEdges.begin(); i != myEdges.end(); ++i) {
        const std::vector<MSLane*> &lanes = (*i)->getLanes();
        if (lanes.size() == 1) {
            size_t pos = (*lanes.begin())->getNumericalID();
            myLanes[pos].lane = *(lanes.begin());
            myLanes[pos].firstNeigh = lanes.end();
            myLanes[pos].lastNeigh = lanes.end();
            myLanes[pos].amActive = false;
            myLanes[pos].haveNeighbors = false;
        } else {
            for (std::vector<MSLane*>::const_iterator j = lanes.begin(); j != lanes.end(); ++j) {
                size_t pos = (*j)->getNumericalID();
                myLanes[pos].lane = *j;
                myLanes[pos].firstNeigh = (j + 1);
                myLanes[pos].lastNeigh = lanes.end();
                myLanes[pos].amActive = false;
                myLanes[pos].haveNeighbors = true;
            }
        }
        size_t pos = (*i)->getNumericalID();
        myLastLaneChange[pos] = -1;
    }
}


MSEdgeControl::~MSEdgeControl() {
}


void
MSEdgeControl::patchActiveLanes() {
    for (std::set<MSLane*, Named::ComparatorIdLess>::iterator i = myChangedStateLanes.begin(); i != myChangedStateLanes.end(); ++i) {
        LaneUsage& lu = myLanes[(*i)->getNumericalID()];
        // if the lane was inactive but is now...
        if (!lu.amActive && (*i)->getVehicleNumber() > 0) {
            // ... add to active lanes and mark as such
            if (lu.haveNeighbors) {
                myActiveLanes.push_front(*i);
            } else {
                myActiveLanes.push_back(*i);
            }
            lu.amActive = true;
        }
    }
    myChangedStateLanes.clear();
}

void MSEdgeControl::moveCritical(SUMOTime t) {
  for (std::list<MSLane*>::iterator i = myActiveLanes.begin(); i != myActiveLanes.end();) {
    if ((*i)->getVehicleNumber() == 0 || (*i)->moveCritical(t)) {
      myLanes[(*i)->getNumericalID()].amActive = false;
      i = myActiveLanes.erase(i);
    } else {
      ++i;
    }
  }
}

void MSEdgeControl::moveFirst(SUMOTime t) {
  myWithVehicles2Integrate.clear();
  for (std::list<MSLane*>::iterator i = myActiveLanes.begin(); i != myActiveLanes.end();) {
    if ((*i)->getVehicleNumber() == 0 || (*i)->setCritical(t, myWithVehicles2Integrate)) {
      myLanes[(*i)->getNumericalID()].amActive = false;
      i = myActiveLanes.erase(i);
    } else {
      ++i;
    }
  }
  for (std::vector<MSLane*>::iterator i = myWithVehicles2Integrate.begin(); i != myWithVehicles2Integrate.end(); ++i) {
    if ((*i)->integrateNewVehicle(t)) {
      LaneUsage& lu = myLanes[(*i)->getNumericalID()];
      if (!lu.amActive) {
        if (lu.haveNeighbors) {
          myActiveLanes.push_front(*i);
        } else {
          myActiveLanes.push_back(*i);
        }
        lu.amActive = true;
      }
    }
  }
}

void MSEdgeControl::changeLanes(SUMOTime t) {
  std::vector<MSLane*> toAdd;
  for (std::list<MSLane*>::iterator i = myActiveLanes.begin(); i != myActiveLanes.end();) {
    LaneUsage& lu = myLanes[(*i)->getNumericalID()];
    if (lu.haveNeighbors) {
      MSEdge& edge = (*i)->getEdge();
      if (myLastLaneChange[edge.getNumericalID()] != t) {
        myLastLaneChange[edge.getNumericalID()] = t;
        edge.changeLanes(t);
        const std::vector<MSLane*> &lanes = edge.getLanes();
        for (std::vector<MSLane*>::const_iterator i = lanes.begin(); i != lanes.end(); ++i) {
          LaneUsage& lu = myLanes[(*i)->getNumericalID()];
          if ((*i)->getVehicleNumber() > 0 && !lu.amActive) {
            toAdd.push_back(*i);
            lu.amActive = true;
          }
        }
      }
      ++i;
    } else {
      i = myActiveLanes.end();
    }
  }
  for (std::vector<MSLane*>::iterator i = toAdd.begin(); i != toAdd.end(); ++i) {
    myActiveLanes.push_front(*i);
  }
}

void MSEdgeControl::detectCollisions(SUMOTime timestep)
{
    // Detections is made by the edge's lanes, therefore hand over.
    for (std::list<MSLane*>::iterator i = myActiveLanes.begin(); i != myActiveLanes.end(); ++i) {
        (*i)->detectCollisions(timestep);
    }
}

std::vector<std::string> MSEdgeControl::getEdgeNames() const
{
    std::vector<std::string> ret;
    for (std::vector<MSEdge*>::const_iterator i = myEdges.begin(); i != myEdges.end(); ++i) {
        ret.push_back((*i)->getID());
    }
    return ret;
}

void MSEdgeControl::gotActive(MSLane* l) {
    myChangedStateLanes.insert(l);
}

void MSEdgeControl::patchEdgeFromJunctions(MSJunctionControl *junctions)
{
  return; // ALERT
  if(OptionsCont::getOptions().getInt("buildVerbosity") > 2)
    std::cout << "----> MSEdgeControl::patchEdgeFromJunctions(MSJunctionControl *junctions)" << std::endl;
  int i, j, k;
  MSEdge *edgeI, *edgeJ;
  MSJunction *edgeIToJunction;
  if(junctions->size() > 0)
  {
    for(int i = 0; i < getEdges().size(); i++)
    {
      /*if(OptionsCont::getOptions().getInt("buildVerbosity") > 2)
        std::cout << "[100] MSEdgeControl::patchEdgeFromJunctions(MSJunctionControl *junctions)" << std::endl;*/
      edgeI = getEdges().at(i);
      if(OptionsCont::getOptions().getInt("buildVerbosity") > 2)
      {
        std::cout << "[110] MSEdgeControl::patchEdgeFromJunctions(MSJunctionControl *junctions)" << std::endl;
      }
      if(edgeI->getToJunction())
      {
        if(OptionsCont::getOptions().getInt("buildVerbosity") > 2)
        {
          std::cout << "[115] MSEdgeControl::patchEdgeFromJunctions(MSJunctionControl *junctions)" << std::endl;
        }
        std::cout << "edgeI->getToJunction()=" << edgeI->getToJunction() << std::endl;
        std::cout << "edgeI->getToJunction()->getNonReferencedId()=" << edgeI->getToJunction()->getNonReferencedId() << std::endl;
        if(OptionsCont::getOptions().getInt("buildVerbosity") > 2)
        {
          std::cout << "[120] MSEdgeControl::patchEdgeFromJunctions(MSJunctionControl *junctions)" << std::endl;
        }
        if(junctions->get(edgeI->getToJunction()->getNonReferencedId()))
          edgeIToJunction = junctions->get(edgeI->getToJunction()->getNonReferencedId());
        else
        /*{*/
          if(OptionsCont::getOptions().getInt("buildVerbosity") > 2)
          /*{*/
            std::cout << "[121] MSEdgeControl::patchEdgeFromJunctions(MSJunctionControl *junctions)" << std::endl;
          /*}
        }*/
      }
      else
      {
        if(OptionsCont::getOptions().getInt("buildVerbosity") > 2)
        {
          std::cout << "[125] MSEdgeControl::patchEdgeFromJunctions(MSJunctionControl *junctions)" << std::endl;
        }
      }
      if(OptionsCont::getOptions().getInt("buildVerbosity") > 2)
        std::cout << "[130] MSEdgeControl::patchEdgeFromJunctions(MSJunctionControl *junctions)" << std::endl;
      for(int j = 0; j < getEdges().size(); j++)
      {
        /*if(OptionsCont::getOptions().getInt("buildVerbosity") > 2)
          std::cout << "[140] MSEdgeControl::patchEdgeFromJunctions(MSJunctionControl *junctions)" << std::endl;*/
        edgeJ = getEdges().at(j);/*
        if(
        dynamic_cast < MSRightOfWayJunction * > (edgeIToJunction)->ge*/      
      }
    }
  }
  else
  {
    if(OptionsCont::getOptions().getInt("buildVerbosity") > 2)
      std::cout << "[900] MSEdgeControl::patchEdgeFromJunctions(MSJunctionControl *junctions)" << std::endl;
    /* No junctions */
  }
}

#define ANY_LANE getLanes().at(0) /* rightLane(NULL) */

void patchForwardSlopes(MSEdge *e)
{
  OptionsCont &oc = OptionsCont::getOptions();
  if(oc.getBuildVerbosity()>1)
    std::cout<<"----> void patchForwardSlopes(MSEdge *e)"<<std::endl;
  int i;
  MSEdge *f;
  SUMOReal heightDiff, opp_cat, adj_cat, slope;
  bool downing;
  for(i=0;i<e->getNoFollowing();i++)
  {
    if(e->getID().at(0)!=':')
    {
      if(oc.getBuildVerbosity()>2)
      {
        std::cout<<"[100] void patchForwardSlopes(MSEdge *e)"<<std::endl;
        std::cout<<" e->getID()="<<e->getID()<<std::endl;
      }
      f = const_cast<MSEdge *>(e->getFollower(i));
      if(oc.getBuildVerbosity()>2)
      {
        std::cout<<"[200] void patchForwardSlopes(MSEdge *e)"<<std::endl;
        std::cout<<" f->getID()="<<f->getID()<<std::endl;
      }
      if(e->getToJunction())
      {
      if(f->getToJunction())
      {
        if(oc.getBuildVerbosity()>2)
        {
          std::cout<<"e->getToJunction()->getDynamicClassName(){"
           <<e->getToJunction()->getDynamicClassName()<<"}"<<std::endl;
          std::cout<<"f->getToJunction()->getDynamicClassName(){"
           <<f->getToJunction()->getDynamicClassName()<<"}"<<std::endl;
        }
        if(e->getToJunction()->getHeight())
        {
        if(f->getToJunction()->getHeight())
        {
        heightDiff=e->getToJunction()->getHeight();
        heightDiff-=f->getToJunction()->getHeight();
        }
        }
      }
      }
      else
        heightDiff = 0;
      /* Rmmbr this is setting slope for f, if f.start - f.end > 0 is downing,
       * if f.start - f.end < 0 is upping
       */
      downing = heightDiff > 0;
      heightDiff = abs(heightDiff);
      /* heightDiff is measured in centimeters */
      opp_cat = heightDiff / 100;
      /* opp_cat is now turned to meters, since we assume adj_cat (edge length)
       * is expressed in meters too
       */
      if(oc.getBuildVerbosity()>2)
      {
        std::cout<<"[750] void patchForwardSlopes(MSEdge *e)"<<std::endl;
        std::cout<<" opp_cat="<<opp_cat<<std::endl;
      }
      adj_cat = f->ANY_LANE->getLength();
      if(oc.getBuildVerbosity()>2)
      {
        std::cout<<"[750] void patchForwardSlopes(MSEdge *e)"<<std::endl;
        std::cout<<" adj_cat="<<adj_cat<<std::endl;
      }
      /*
      * / * Arc tangent of X.  * /
      * __MATHCALL (atan,, (_Mdouble_ __x));
      * / * Arc tangent of Y/X.  * /
      * __MATHCALL (atan2,, (_Mdouble_ __y, _Mdouble_ __x));
      */
      slope = atan2(opp_cat,adj_cat);
      slope = downing ? - slope : slope;
      if(oc.getBuildVerbosity()>2)
      {
        std::cout<<"[750] void patchForwardSlopes(MSEdge *e)"<<std::endl;
        std::cout<<" slope \""<<slope<<"\""
                 <<" set on the edge \""<<f->getID()<<"\""<<std::endl;
      }
      f->setSlope(slope);
    }
    else
    {
      /* internal slopes don't processed? */
      ;
    }
  }
  if(oc.getBuildVerbosity()>1)
    std::cout<<"<---- void patchForwardSlopes(MSEdge *e)"<<std::endl;
}

void MSEdgeControl::patchSlopes(MSJunctionControl *junctions)
{
  SUMOReal heightDiff, opp_cat, adj_cat;
  MSEdge *edge;
  int a, i;
  OptionsCont &oc = OptionsCont::getOptions();
  //return; // ALERT
  if(oc.getBuildVerbosity() > 1)
    std::cout << "----> MSEdgeControl::patchSlopes(MSJunctionControl *junctions)" << std::endl;
  //patchEdgeFromJunctions(junctions);
  for(i = 0; i < getEdges().size(); i++)
  {
    if(oc.getBuildVerbosity()>2)
    {
      std::cout<<"'''''''''''''''"<<std::endl;
      std::cout<<"i="<<i<<std::endl;
      std::cout<<"getEdges().at("<<i<<")->getID()="<<getEdges().at(i)->getID()<<std::endl;
    }
    if(oc.getBuildVerbosity() > 2)
      std::cout << "[100] MSEdgeControl::patchSlopes(MSJunctionControl *junctions)" << std::endl;
    edge = getEdges().at(i);
    patchForwardSlopes(edge);
    //return; // ALERT
#ifdef HAVE_INTERNAL_LANES
    /* not process internal or nonexistant lanes */
    MSLane *logicalNoninternalPredecessorLane =
      edge->ANY_LANE->getLogicalPredecessorLane();
    if(oc.getBuildVerbosity() > 2)
      std::cout<<"a="<<a<<std::endl;
    a=0;
    if(logicalNoninternalPredecessorLane)
    {
      if(oc.getBuildVerbosity() > 2)
        std::cout<<"logicalNoninternalPredecessorLane->getID()="<<logicalNoninternalPredecessorLane->getID()<<std::endl;
    }
    while(logicalNoninternalPredecessorLane &&
          logicalNoninternalPredecessorLane->getID().at(0)==':')
    {
      if(oc.getBuildVerbosity() > 2)
        std::cout<<"a="<<++a<<std::endl;      
      logicalNoninternalPredecessorLane =
        logicalNoninternalPredecessorLane->getLogicalPredecessorLane();
      if(logicalNoninternalPredecessorLane)
      {
        if(oc.getBuildVerbosity() > 2)
          std::cout<<"logicalNoninternalPredecessorLane->getID()="<<logicalNoninternalPredecessorLane->getID()<<std::endl;
      }
    }
    if(oc.getBuildVerbosity() > 2)
      std::cout<<"out of while"<<std::endl;
    if(edge->getPurpose()!=MSEdge::EDGEFUNCTION_INTERNAL &&
       logicalNoninternalPredecessorLane)
    //if(true)
    {
    if(oc.getBuildVerbosity() > 2)
      std::cout<<"logicalNoninternalPredecessorLane->getID()="<<logicalNoninternalPredecessorLane->getID();
#endif   
    if(oc.getBuildVerbosity() > 2)
    {
      std::cout<<"slopePatching supposedly noninternal edge"<<std::endl;
      std::cout<<"edge="<<edge<<std::endl;
      std::cout<<"edge->ANY_LANE="<<edge->ANY_LANE<<std::endl;
      std::cout<<"edge->ANY_LANE->getID()="<<edge->ANY_LANE->getID()<<std::endl;
      std::cout<<"edge->ANY_LANE->getLogicalPredecessorLane()->getID()="<<edge->ANY_LANE->getLogicalPredecessorLane()->getID()<<std::endl;
      //std::cout<<"edge->ANY_LANE->getLogicalPredecessorLane()->getEdge()="<<edge->ANY_LANE->getLogicalPredecessorLane()->getEdge()<<std::endl;
      std::cout<<"logicalNoninternalPredecessorLane->getEdge().getToJunction()="
        <<logicalNoninternalPredecessorLane->getEdge().getToJunction()<<std::endl;
      std::cout<<"logicalNoninternalPredecessorLane->getEdge().getToJunction()->getID()="
        <<logicalNoninternalPredecessorLane->getEdge().getToJunction()->getID()<<std::endl;
      std::cout<<"edge->getToJunction()="<<edge->getToJunction()<<std::endl;
      std::cout<<"edge->getToJunction()->getID()="<<edge->getToJunction()->getID()<<std::endl;
    }
    MSEdge *e1,*e2;
    MSJunction *j1,*j2;
    bool broken=false;
    e1=&logicalNoninternalPredecessorLane->getEdge();
    e2=edge;
    if(e1)
    {
      if(e1->getToJunction())
      {
        if(e1->getToJunction()->getID()!="")
          j1=junctions->get(e1->getToJunction()->getID());
        else
        {
          std::cout<<"CRITICAL IN "<<__FILE__<<":"<<__LINE__<<"?"<<std::endl;
          std::cout<<"uninitialized 'e1->getToJunction()->getID()'?"<<std::endl;
          std::cout<<"e1->getToJunction(){"<<e1->getToJunction()<<"}"
           <<std::endl;
          broken=true;
        }
      }
      else
      {
        std::cout<<"CRITICAL IN "<<__FILE__<<":"<<__LINE__<<"."<<std::endl;
        std::cout<<"uninitialized 'e1->getToJunction()'"<<std::endl;
        std::cout<<"e1->getID(){"<<e1->getID()<<"}"<<std::endl;
        broken=true;
      }
    }
    else
    {
      std::cout<<"CRITICAL IN "<<__FILE__<<":"<<__LINE__<<"."<<std::endl;
      std::cout<<"uninitialized 'e1'"<<std::endl;
      broken=true;
    }
    if(e2)
    {
      if(e2->getToJunction())
        j2=junctions->get(e2->getToJunction()->getID());
      else
      {
        std::cout<<"CRITICAL IN "<<__FILE__<<":"<<__LINE__<<"."<<std::endl;
        std::cout<<"uninitialized 'e2->getToJunction()'"<<std::endl;
        broken=true;
      }
    }
    else
    {
      std::cout<<"CRITICAL IN "<<__FILE__<<":"<<__LINE__<<"."<<std::endl;
      std::cout<<"uninitialized 'e2'"<<std::endl;
      broken=true;
    }
    if(broken)
      heightDiff=0;
    else
      heightDiff=abs(j1->getHeight()-j2->getHeight());
    /*
    heightDiff = abs(
      junctions->get(logicalNoninternalPredecessorLane
                     ->getEdge()
                       .getToJunction()->getID())->getHeight()
    -
      junctions->get(edge
                       ->getToJunction()->getID())->getHeight()
    );
    */
    opp_cat = heightDiff;
    adj_cat = edge->ANY_LANE->getLength();
    /*
     * / * Arc tangent of X.  * /
     * __MATHCALL (atan,, (_Mdouble_ __x));
     * / * Arc tangent of Y/X.  * /
     * __MATHCALL (atan2,, (_Mdouble_ __y, _Mdouble_ __x));
     */
    edge->setSlope(atan2(opp_cat,adj_cat));
    if(oc.getBuildVerbosity() > 2)
    {
      std::cout<<"[800] void MSEdgeControl::patchSlopes(MSJunctionControl *junctions)"<<std::endl;
      std::cout<<"slope "<<edge->getSlope()<<" for edge "<<edge->getID();
      std::cout<<"[850] void MSEdgeControl::patchSlopes(MSJunctionControl *junctions)"<<std::endl;
    }
#ifdef HAVE_INTERNAL_LANES
    }
    else
    {
      if(oc.getBuildVerbosity() > 1)
        std::cout<<"internal or nonappliable edge not slopePatched"<<std::endl;
    }
#endif
  }
}

void MSEdgeControl::resolveToJunctionsFromNames(MSJunctionControl *junctions)
{
  OptionsCont &oc = OptionsCont::getOptions();
  int j; /* junction index */
  int il; /* incoming lane index */
  int nil; /* number of incoming lanes */
  std::vector < MSJunction * > junctionsVector;
  std::string type;
  /*if(OptionsCont::getOptions()..getInt("buildVerbosity")>2)*/
  if(oc.getBuildVerbosity()>1)
    std::cout<<"----> void MSEdgeControl::resolveToJunctionsFromNames(...)"<<std::endl;
  junctionsVector = junctions->getTempVector();
  for(j=0;j<junctionsVector.size();j++)
  {
    if(!junctionsVector.at(j)->isInternal())
    {
      type=junctionsVector.at(j)->getDynamicClassName();
      if(oc.getBuildVerbosity()>2)
        std::cout<<"junctionsVector.at("<<j<<") is of dynamic type "<<type<<std::endl;
      if(type=="MSRightOfWayJunction")
      {
        nil=dynamic_cast<MSRightOfWayJunction*>(junctionsVector.at(j))->getIncLanes().size();
        for(il=0;il<nil;il++)
        {
          /*
          dynamic_cast < MSRightOfWayJunction * >
          (junctionsVector.at(j))
          ->getIncLanes().at(il)
          ->setToJunction(
            junctionsVector.at(j)
          );
          */
          dynamic_cast<MSRightOfWayJunction*>(junctionsVector.at(j))->getIncLanes().at(il)->setToJunction(junctionsVector.at(j));
        }
      }
      else if(type=="MSLogicJunction")
      {
        nil=dynamic_cast<MSLogicJunction*>(junctionsVector.at(j))->getIncLanes().size();
        for(il=0;il<nil;il++)
        {
          dynamic_cast<MSLogicJunction*>(junctionsVector.at(j))->getIncLanes().at(il)->setToJunction(junctionsVector.at(j));
        }
      }
      else if(type=="MSNoLogicJunction")
      {
        if(oc.getBuildVerbosity()>2)
          std::cout<<"junctionsVector.at("<<j<<") is a MSNoLogicJunction with id "<<junctionsVector.at(j)->getID()<<std::endl;
        nil=dynamic_cast<MSNoLogicJunction*>(junctionsVector.at(j))->getIncLanes().size();
        for(il=0;il<nil;il++)
        {
          dynamic_cast<MSNoLogicJunction*>(junctionsVector.at(j))->getIncLanes().at(il)->setToJunction(junctionsVector.at(j));
        }
      }
      else
      {
        std::cout<<"CRITICAL in MSEdgeControl::resolveToJunctionsFromNames(MSJunctionControl *junctions)"<<std::endl;
      }
    }
  }
}

void MSEdgeControl::resolveToJunctionsFromNames_(MSJunctionControl *junctions)
{
  bool found;
  int i, j;
  OptionsCont &oc = OptionsCont::getOptions();
  if(oc.getBuildVerbosity()>1)
    std::cout<<"----> void MSEdgeControl::resolveToJunctionsFromNames_(...)"<<std::endl;
  for(i=0;i<getEdges().size();i++)
  {
    found = false;
    if(oc.getBuildVerbosity()>2)
      std::cout<<"[100] void MSEdgeControl::resolveToJunctionsFromNames_(...)"
      <<" i="<<i<<std::endl;
    /* Find junction by name... */
    j=0;
    while(!found&&j<junctions->size())
      /*junctions->getTempVector().size() seems not to work as well*/
    {
      if(oc.getBuildVerbosity()>2)
      {
        std::cout<<"[100] void MSEdgeControl::resolveToJunctionsFromNames_(...)"
        <<" j="<<j<<std::endl;
        /*if(getEdges())
          std::cout<<"getEdges()"<<std::endl;*/
        if(getEdges().at(i))
        {
          std::cout<<"getEdges().at("<<i<<")"<<std::endl;
          /*if(getEdges().at(i)->getNameOfMyToJunction())
            std::cout<<"getEdges().at(i)->getNameOfMyToJunction()"<<std::endl;*/
          std::cout<<"getEdges().at("<<i<<")->getNameOfMyToJunction()"<<std::endl;
        }
        if(junctions)
          std::cout<<"junctions"<<std::endl;
        /*if(junctions->getTempVector())
          std::cout<<"junctions->getTempVector()"<<std::endl;*/
        if(junctions->getTempVector().at(j))
        {
          std::cout<<"junctions->getTempVector().at("<<j<<")"<<std::endl;
          /*if(junctions->getTempVector().at(j)->getID())
            std::cout<<"junctions->getTempVector().at(j)->getID()"<<std::endl;*/
          std::cout<<"junctions->getTempVector().at("<<j<<")->getID()="<<junctions->getTempVector().at(j)->getID()<<std::endl;
        }
      }
      if(junctions->getTempVector().at(j)->getID() == getEdges().at(i)->getNameOfMyToJunction())
      {
        getEdges().at(i)->setToJunction(junctions->getTempVector().at(j));
        found=true;
        std::cout<<"found the ToJunction junction of the edge "<<i<<" (junction "<<j<<")"<<std::endl;
      }
      else
        j++;
    }
    if(!found)
      std::cout<<"CRITICAL in MSEdgeControl::resolveToJunctionsFromNames_(MSJunctionControl *junctions)"<<std::endl;
  }
}
