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
/// @file    MSDetectorControl.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Clemens Honomichl
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @author  Laura Bieker
/// @date    2005-09-15
/// @version $Id$
///
// Detectors container; responsible for string and output generation
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
#include "MSDetectorControl.h"
#include "MSMeanData_Net.h"
#include <utils/options/OptionsCont.h>
#include <utils/options/Option.h>
#include <utils/common/MsgHandler.h>


// ===========================================================================
// member method definitions
// ===========================================================================
MSDetectorControl::MSDetectorControl() {
}


MSDetectorControl::~MSDetectorControl() {
    for (std::map<SumoXMLTag, NamedObjectCont<MSDetectorFileOutput*> >::iterator i = myDetectors.begin(); i != myDetectors.end(); ++i) {
        (*i).second.clear();
    }
    for (std::vector<MSMeanData*>::const_iterator i = myMeanData.begin(); i != myMeanData.end(); ++i) {
        delete *i;
    }
}


void
MSDetectorControl::close(SUMOTime step) {
    // flush the last values
    writeOutput(step, true);
    // [...] files are closed on another place [...]
    myIntervals.clear();
}


void
MSDetectorControl::add(SumoXMLTag type, MSDetectorFileOutput* d, const std::string& device, SUMOTime splInterval, SUMOTime begin) {
    if (!myDetectors[type].add(d->getID(), d)) {
        throw ProcessError(toString(type) + " detector '" + d->getID() + "' could not be build (declared twice?).");
    }
    addDetectorAndInterval(d, &OutputDevice::getDevice(device), splInterval, begin);
}



void
MSDetectorControl::add(SumoXMLTag type, MSDetectorFileOutput* d) {
    if (!myDetectors[type].add(d->getID(), d)) {
        throw ProcessError(toString(type) + " detector '" + d->getID() + "' could not be build (declared twice?).");
    }
}



void
MSDetectorControl::add(MSMeanData* mn, const std::string& device,
                       SUMOTime frequency, SUMOTime begin) {
    myMeanData.push_back(mn);
    addDetectorAndInterval(mn, &OutputDevice::getDevice(device), frequency, begin);
    if (begin == string2time(OptionsCont::getOptions().getString("begin"))) {
        mn->init();
    }
}


const std::vector<SumoXMLTag>
MSDetectorControl::getAvailableTypes() const {
    std::vector<SumoXMLTag> result;
    for (std::map<SumoXMLTag, NamedObjectCont<MSDetectorFileOutput*> >::const_iterator i = myDetectors.begin(); i != myDetectors.end(); ++i) {
        result.push_back(i->first);
    }
    return result;
}


const NamedObjectCont<MSDetectorFileOutput*>&
MSDetectorControl::getTypedDetectors(SumoXMLTag type) const {
    if (myDetectors.find(type) == myDetectors.end()) {
        return myEmptyContainer;
    }
    return myDetectors.find(type)->second;
}


void
MSDetectorControl::updateDetectors(const SUMOTime step) {
    for (const auto& i : myDetectors) {
        for (const auto& j : getTypedDetectors(i.first)) {
            j.second->detectorUpdate(step);
        }
    }
    for (MSMeanData* const i : myMeanData) {
        i->detectorUpdate(step);
    }
}


void
MSDetectorControl::writeOutput(SUMOTime step, bool closing) {
    for (Intervals::iterator i = myIntervals.begin(); i != myIntervals.end(); ++i) {
        IntervalsKey interval = (*i).first;
        if (myLastCalls[interval] + interval.first <= step || (closing && myLastCalls[interval] < step)) {
            DetectorFileVec dfVec = (*i).second;
            SUMOTime startTime = myLastCalls[interval];
            // check whether at the end the output was already generated
            for (DetectorFileVec::iterator it = dfVec.begin(); it != dfVec.end(); ++it) {
                MSDetectorFileOutput* det = it->first;
                det->writeXMLOutput(*(it->second), startTime, step);
            }
            myLastCalls[interval] = step;
        }
    }
}


void
MSDetectorControl::addDetectorAndInterval(MSDetectorFileOutput* det,
        OutputDevice* device,
        SUMOTime interval,
        SUMOTime begin) {
    if (begin == -1) {
        begin = string2time(OptionsCont::getOptions().getString("begin"));
    }
    IntervalsKey key = std::make_pair(interval, begin);
    Intervals::iterator it = myIntervals.find(key);
    // Add command for given key only once to MSEventControl...
    if (it == myIntervals.end()) {
        DetectorFileVec detAndFileVec;
        detAndFileVec.push_back(std::make_pair(det, device));
        myIntervals.insert(std::make_pair(key, detAndFileVec));
        myLastCalls[key] = begin;
    } else {
        DetectorFileVec& detAndFileVec = it->second;
        if (find_if(detAndFileVec.begin(), detAndFileVec.end(), bind2nd(detectorEquals(), det)) == detAndFileVec.end()) {
            detAndFileVec.push_back(std::make_pair(det, device));
        } else {
            // detector already in container. Don't add several times
            WRITE_WARNING("MSDetectorControl::addDetectorAndInterval: detector already in container. Ignoring.");
            return;
        }
    }
    det->writeXMLDetectorProlog(*device);
}



/****************************************************************************/

