/****************************************************************************/
/// @file    MSRouteLoaderControl.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Wed, 06 Nov 2002
/// @version $Id: MSRouteLoaderControl.cpp 11671 2012-01-07 20:14:30Z behrisch $
///
// Class responsible for loading of routes from some files
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

#include <vector>
#include "utils/options/OptionsCont.h"
#include "MSRouteLoader.h"
#include "MSRouteLoaderControl.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS

// ===========================================================================
// method definitions
// ===========================================================================
MSRouteLoaderControl::MSRouteLoaderControl(MSNet&,
        int inAdvanceStepNo,
        LoaderVector loader)
    : myLastLoadTime(-inAdvanceStepNo),
      myInAdvanceStepNo(inAdvanceStepNo),
      myRouteLoaders(loader),
      myAllLoaded(false) {
    myLoadAll = myInAdvanceStepNo <= 0;
    myAllLoaded = false;
    myLastLoadTime = -1 * (int) myInAdvanceStepNo;
    // initialize all used loaders
    for (LoaderVector::iterator i = myRouteLoaders.begin();
            i != myRouteLoaders.end(); ++i) {
        (*i)->init();
    }
}

MSRouteLoaderControl::~MSRouteLoaderControl() {
    for (LoaderVector::iterator i = myRouteLoaders.begin();
            i != myRouteLoaders.end(); ++i) {
        delete(*i);
    }
}

void MSRouteLoaderControl::loadNext(SUMOTime step)
{
  OptionsCont &oc = OptionsCont::getOptions();
  if(oc.getLoadVerbosity()>1)
    std::cout<<"----> MSRouteLoaderControl::loadNext(SUMOTime step{"<<step<<"})"<<std::endl;
  // check whether new vehicles shall be loaded
  //  return if not
  if ((myLoadAll && myAllLoaded) || (myLastLoadTime >= 0 && myLastLoadTime/*+myInAdvanceStepNo*/ >= step)) {
      return;
  }
  // load all routes for the specified time period
  SUMOTime run = step;
  bool furtherAvailable = true;
  for (;
          furtherAvailable &&
          (myLoadAll || run <= step + (SUMOTime) myInAdvanceStepNo);
          run++) {
      furtherAvailable = false;
      for (LoaderVector::iterator i = myRouteLoaders.begin();
              i != myRouteLoaders.end(); ++i) {
          if ((*i)->moreAvailable()) {
              (*i)->loadUntil(run);
          }
          furtherAvailable |= (*i)->moreAvailable();
      }
  }
  // no further loading when all was loaded
  if (myLoadAll || !furtherAvailable) {
      myAllLoaded = true;
  }
  // set the step information
  myLastLoadTime = run - 1;
}
