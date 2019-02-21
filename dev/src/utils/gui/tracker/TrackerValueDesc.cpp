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
/// @file    TrackerValueDesc.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @author  Laura Bieker
/// @date    Sept 2002
/// @version $Id$
///
// Storage for a tracked value
/****************************************************************************/
// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <vector>
#include <utils/common/RGBColor.h>
#include <utils/gui/globjects/GUIGlObject.h>
#include "TrackerValueDesc.h"


// ===========================================================================
// method definitions
// ===========================================================================
TrackerValueDesc::TrackerValueDesc(const std::string& name,
                                   const RGBColor& col,
                                   SUMOTime recordBegin,
                                   double aggregationSeconds)
    : myName(name), myActiveCol(col), myInactiveCol(col),
      myMin(0), myMax(0),
      myAggregationInterval(MAX2(1, (int)(TIME2STEPS(aggregationSeconds) / DELTA_T))),
      myInvalidValue(INVALID_DOUBLE),
      myValidNo(0),
      myRecordingBegin(recordBegin), myTmpLastAggValue(0) {}


TrackerValueDesc::~TrackerValueDesc() {
    // just to quit cleanly on a failure
    if (myLock.locked()) {
        myLock.unlock();
    }
}


void
TrackerValueDesc::addValue(double value) {
    if (myValues.size() == 0) {
        myMin = value;
        myMax = value;
    } else {
        myMin = value < myMin ? value : myMin;
        myMax = value > myMax ? value : myMax;
    }
    AbstractMutex::ScopedLocker locker(myLock);
    myValues.push_back(value);
    if (value != myInvalidValue) {
        myTmpLastAggValue += value;
        myValidNo++;
    }
    const double avg = myValidNo == 0 ? static_cast<double>(0) : myTmpLastAggValue / static_cast<double>(myValidNo);
    if (myAggregationInterval == 1 || myValues.size() % myAggregationInterval == 1) {
        myAggregatedValues.push_back(avg);
    } else {
        myAggregatedValues.back() = avg;
    }
    if (myValues.size() % myAggregationInterval == 0) {
        myTmpLastAggValue = 0;
        myValidNo = 0;
    }
}


double
TrackerValueDesc::getRange() const {
    return myMax - myMin;
}


double
TrackerValueDesc::getMin() const {
    return myMin;
}


double
TrackerValueDesc::getMax() const {
    return myMax;
}


double
TrackerValueDesc::getYCenter() const {
    return (myMin + myMax) / 2.0f;
}


const RGBColor&
TrackerValueDesc::getColor() const {
    return myActiveCol;
}


const std::vector<double>&
TrackerValueDesc::getValues() {
    myLock.lock();
    return myValues;
}


const std::vector<double>&
TrackerValueDesc::getAggregatedValues() {
    myLock.lock();
    return myAggregatedValues;
}


const std::string&
TrackerValueDesc::getName() const {
    return myName;
}

void
TrackerValueDesc::unlockValues() {
    myLock.unlock();
}


void
TrackerValueDesc::setAggregationSpan(SUMOTime as) {
    AbstractMutex::ScopedLocker locker(myLock);
    if (myAggregationInterval != as / DELTA_T) {
        myAggregationInterval = (int)(as / DELTA_T);
        // ok, the aggregation has changed,
        //  let's recompute the list of aggregated values
        myAggregatedValues.clear();
        std::vector<double>::const_iterator i = myValues.begin();
        while (i != myValues.end()) {
            myTmpLastAggValue = 0;
            myValidNo = 0;
            for (int j = 0; j < myAggregationInterval && i != myValues.end(); j++, ++i) {
                if ((*i) != myInvalidValue) {
                    myTmpLastAggValue += (*i);
                    myValidNo++;
                }
            }
            if (myValidNo == 0) {
                myAggregatedValues.push_back(0);
            } else {
                myAggregatedValues.push_back(myTmpLastAggValue / static_cast<double>(myValidNo));
            }
        }
    }
}


SUMOTime
TrackerValueDesc::getAggregationSpan() const {
    return (SUMOTime)(myAggregationInterval) * DELTA_T;
}


SUMOTime
TrackerValueDesc::getRecordingBegin() const {
    return myRecordingBegin;
}



/****************************************************************************/

