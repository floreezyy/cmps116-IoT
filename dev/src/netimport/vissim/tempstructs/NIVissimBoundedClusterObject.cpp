/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    NIVissimBoundedClusterObject.cpp
/// @author  Daniel Krajzewicz
/// @date    Sept 2002
/// @version $Id$
///
// -------------------
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
#include <utils/geom/Boundary.h>
#include "NIVissimBoundedClusterObject.h"

NIVissimBoundedClusterObject::ContType NIVissimBoundedClusterObject::myDict;

NIVissimBoundedClusterObject::NIVissimBoundedClusterObject()
    : myBoundary(0), myClusterID(-1) {
    myDict.insert(this);
}


NIVissimBoundedClusterObject::~NIVissimBoundedClusterObject() {
    delete myBoundary;
}


bool
NIVissimBoundedClusterObject::crosses(const AbstractPoly& poly,
                                      double offset) const {
    assert(myBoundary != 0 && myBoundary->xmax() >= myBoundary->xmin());
    return myBoundary->overlapsWith(poly, offset);
}


void
NIVissimBoundedClusterObject::inCluster(int id) {
    myClusterID = id;
}


bool
NIVissimBoundedClusterObject::clustered() const {
    return myClusterID > 0;
}


void
NIVissimBoundedClusterObject::closeLoading() {
    for (ContType::iterator i = myDict.begin(); i != myDict.end(); i++) {
        (*i)->computeBounding();
    }
}


const Boundary&
NIVissimBoundedClusterObject::getBoundary() const {
    return *myBoundary;
}



/****************************************************************************/

