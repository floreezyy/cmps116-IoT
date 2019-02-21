/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2010-2017 German Aerospace Center (DLR) and others.
// activitygen module
// Copyright 2010 TUM (Technische Universitaet Muenchen, http://www.tum.de/)
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    AGStreet.cpp
/// @author  Piotr Woznica
/// @author  Walter Bamberger
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    July 2010
/// @version $Id$
///
// Represents a SUMO edge and contains people and work densities
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "AGStreet.h"
#include "router/ROEdge.h"
#include <iostream>


// ===========================================================================
// method definitions
// ===========================================================================
AGStreet::AGStreet(const std::string& id, RONode* from, RONode* to, int index, const int priority) :
    ROEdge(id, from, to, index, priority), myPopulation(0.), myNumWorkplaces(0.) {
}


void
AGStreet::print() const {
    std::cout << "- AGStreet: Name=" << getID() << " Length=" << getLength() << " pop=" << myPopulation << " work=" << myNumWorkplaces << std::endl;
}


double
AGStreet::getPopulation() const {
    return myPopulation;
}


void
AGStreet::setPopulation(const double population) {
    myPopulation = population;
}


double
AGStreet::getWorkplaceNumber() const {
    return myNumWorkplaces;
}


void
AGStreet::setWorkplaceNumber(const double workPositions) {
    myNumWorkplaces = workPositions;
}


bool
AGStreet::allows(const SUMOVehicleClass vclass) const {
    return (getPermissions() & vclass) == vclass;
}


/****************************************************************************/
