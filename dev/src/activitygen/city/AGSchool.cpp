/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2017 German Aerospace Center (DLR) and others.
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
/// @file    AGSchool.cpp
/// @author  Piotr Woznica
/// @author  Daniel Krajzewicz
/// @author  Walter Bamberger
/// @date    July 2010
/// @version $Id$
///
// Correspond to given ages and referenced by children. Has a precise location.
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <iostream>
#include <string>
#include "AGSchool.h"
#include "AGPosition.h"


// ===========================================================================
// method definitions
// ===========================================================================
void
AGSchool::print() const {
    std::cout << "- school: " << " placeNbr=" << capacity << " hours=[" << opening << ";" << closing << "] ages=[" << beginAge << ";" << endAge << "]" << std::endl;
}

int
AGSchool::getPlaces() {
    return capacity;
}

bool
AGSchool::addNewChild() {
    if (capacity > 0) {
        --capacity;
        return true;
    }
    return false;
}

bool
AGSchool::removeChild() {
    if (capacity < initCapacity) {
        ++capacity;
        return true;
    }
    return false;
}

bool
AGSchool::acceptThisAge(int age) {
    if (age <= endAge && age >= beginAge) {
        return true;
    }
    return false;
}

int
AGSchool::getBeginAge() {
    return beginAge;
}

int
AGSchool::getEndAge() {
    return endAge;
}

AGPosition
AGSchool::getPosition() {
    return location;
}

int
AGSchool::getClosingHour() {
    return closing;
}

int
AGSchool::getOpeningHour() {
    return opening;
}

/****************************************************************************/
