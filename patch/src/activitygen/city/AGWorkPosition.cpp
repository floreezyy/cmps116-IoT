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
/// @file    AGWorkPosition.cpp
/// @author  Piotr Woznica
/// @author  Walter Bamberger
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    July 2010
/// @version $Id$
///
// Location and schedules of a work position: linked with one adult
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "AGWorkPosition.h"
#include "AGStreet.h"
#include "AGPosition.h"
#include "AGDataAndStatistics.h"
#include "AGAdult.h"
#include <utils/common/RandHelper.h>
#include <iostream>


// ===========================================================================
// method definitions
// ===========================================================================
AGWorkPosition::AGWorkPosition(AGDataAndStatistics* ds, const AGStreet& inStreet) :
    myStatData(ds),
    myLocation(inStreet),
    myAdult(0),
    myOpeningTime(generateOpeningTime(*ds)),
    myClosingTime(generateClosingTime(*ds)) {
    ds->workPositions++;
}


AGWorkPosition::AGWorkPosition(AGDataAndStatistics* ds, const AGStreet& inStreet, double pos) :
    myStatData(ds),
    myLocation(inStreet, pos),
    myAdult(0),
    myOpeningTime(generateOpeningTime(*ds)),
    myClosingTime(generateClosingTime(*ds)) {
    ds->workPositions++;
}

AGWorkPosition::~AGWorkPosition() {
//    let();
}


void
AGWorkPosition::print() const {
    std::cout << "- AGWorkPosition: open=" << myOpeningTime << " closingTime=" << myClosingTime << " taken=" << isTaken() << std::endl;
    std::cout << "\t";
    myLocation.print();
}


int
AGWorkPosition::generateOpeningTime(const AGDataAndStatistics& ds) {
    double choice = RandHelper::rand();
    double cumul = 0;

    for (std::map<int, double>::const_iterator it = ds.beginWorkHours.begin();
            it != ds.beginWorkHours.end(); ++it) {
        cumul += it->second;
        if (cumul >= choice) {
            return it->first;
        }
    }
    std::cout << "-- WARNING: work time distribution not complete (Sum(proportions) != 1): AUTODEFINED at 9.00am --" << std::endl;
    return 900;
}


int
AGWorkPosition::generateClosingTime(const AGDataAndStatistics& ds) {
    double choice = RandHelper::rand();
    double cumul = 0;
    for (std::map<int, double>::const_iterator it = ds.endWorkHours.begin();
            it != ds.endWorkHours.end(); ++it) {
        cumul += it->second;
        if (cumul >= choice) {
            return it->first;
        }
    }
    std::cout << "-- WARNING: work time distribution not complete (Sum(proportions) != 1): AUTODEFINED at 5.00pm --" << std::endl;
    return 1700;
}


bool
AGWorkPosition::isTaken() const {
    return (myAdult != 0);
}


void
AGWorkPosition::let() {
    if (myAdult != 0) {
        myStatData->workPositions++;
        myAdult->lostWorkPosition();
        myAdult = 0;
    }
}


void
AGWorkPosition::take(AGAdult* worker) {
    if (myAdult == 0) {
        myStatData->workPositions--;
        myAdult = worker;
    } else {
        throw (std::runtime_error("Work position already occupied. Cannot give it to another adult."));
    }
}


AGPosition
AGWorkPosition::getPosition() const {
    return myLocation;
}


int
AGWorkPosition::getClosing() const {
    return myClosingTime;
}


int
AGWorkPosition::getOpening() const {
    return myOpeningTime;
}

/****************************************************************************/
