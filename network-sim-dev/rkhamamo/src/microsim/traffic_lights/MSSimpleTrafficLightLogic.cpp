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
/// @file    MSSimpleTrafficLightLogic.cpp
/// @author  Daniel Krajzewicz
/// @author  Julia Ringel
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @author  Friedemann Wesner
/// @date    Sept 2002
/// @version $Id$
///
// A fixed traffic light logic
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
#include <utility>
#include <vector>
#include <bitset>
#include <sstream>
#include <microsim/MSEventControl.h>
#include <microsim/MSNet.h>
#include "MSTLLogicControl.h"
#include "MSTrafficLightLogic.h"
#include "MSSimpleTrafficLightLogic.h"


// ===========================================================================
// member method definitions
// ===========================================================================
MSSimpleTrafficLightLogic::MSSimpleTrafficLightLogic(MSTLLogicControl& tlcontrol,
        const std::string& id, const std::string& subid, const Phases& phases,
        int step, SUMOTime delay,
        const std::map<std::string, std::string>& parameters) :
    MSTrafficLightLogic(tlcontrol, id, subid, delay, parameters),
    myPhases(phases),
    myStep(step) {
    for (int i = 0; i < (int)myPhases.size(); i++) {
        myDefaultCycleTime += myPhases[i]->duration;
    }
}


MSSimpleTrafficLightLogic::~MSSimpleTrafficLightLogic() {
    deletePhases();
}


// ------------ Switching and setting current rows
SUMOTime
MSSimpleTrafficLightLogic::trySwitch() {
    // check whether the current duration shall be increased
    if (myCurrentDurationIncrement > 0) {
        SUMOTime delay = myCurrentDurationIncrement;
        myCurrentDurationIncrement = 0;
        return delay;
    }

    // increment the index
    myStep++;
    // if the last phase was reached ...
    if (myStep >= (int)myPhases.size()) {
        // ... set the index to the first phase
        myStep = 0;
    }
    assert((int)myPhases.size() > myStep);
    //stores the time the phase started
    myPhases[myStep]->myLastSwitch = MSNet::getInstance()->getCurrentTimeStep();
    // check whether the next duration was overridden
    if (myOverridingTimes.size() > 0) {
        SUMOTime nextDuration = myOverridingTimes[0];
        myOverridingTimes.erase(myOverridingTimes.begin());
        return nextDuration;
    }
    // return offset to the next switch
    return myPhases[myStep]->duration;
}


// ------------ Static Information Retrieval
int
MSSimpleTrafficLightLogic::getPhaseNumber() const {
    return (int) myPhases.size();
}


const MSSimpleTrafficLightLogic::Phases&
MSSimpleTrafficLightLogic::getPhases() const {
    return myPhases;
}


MSSimpleTrafficLightLogic::Phases&
MSSimpleTrafficLightLogic::getPhases() {
    return myPhases;
}


const MSPhaseDefinition&
MSSimpleTrafficLightLogic::getPhase(int givenStep) const {
    assert((int)myPhases.size() > givenStep);
    return *myPhases[givenStep];
}


// ------------ Dynamic Information Retrieval
int
MSSimpleTrafficLightLogic::getCurrentPhaseIndex() const {
    return myStep;
}


const MSPhaseDefinition&
MSSimpleTrafficLightLogic::getCurrentPhaseDef() const {
    return *myPhases[myStep];
}


// ------------ Conversion between time and phase
SUMOTime
MSSimpleTrafficLightLogic::getPhaseIndexAtTime(SUMOTime simStep) const {
    SUMOTime position = 0;
    if (myStep > 0) {
        for (int i = 0; i < myStep; i++) {
            position = position + getPhase(i).duration;
        }
    }
    position = position + simStep - getPhase(myStep).myLastSwitch;
    position = position % myDefaultCycleTime;
    assert(position <= myDefaultCycleTime);
    return position;
}


SUMOTime
MSSimpleTrafficLightLogic::getOffsetFromIndex(int index) const {
    assert(index < (int)myPhases.size());
    if (index == 0) {
        return 0;
    }
    SUMOTime pos = 0;
    for (int i = 0; i < index; i++) {
        pos += getPhase(i).duration;
    }
    return pos;
}


int
MSSimpleTrafficLightLogic::getIndexFromOffset(SUMOTime offset) const {
    offset = offset % myDefaultCycleTime;
    if (offset == myDefaultCycleTime) {
        return 0;
    }
    SUMOTime testPos = 0;
    for (int i = 0; i < (int)myPhases.size(); i++) {
        testPos = testPos + getPhase(i).duration;
        if (testPos > offset) {
            return i;
        }
        if (testPos == offset) {
            assert((int)myPhases.size() > (i + 1));
            return (i + 1);
        }
    }
    return 0;
}


// ------------ Changing phases and phase durations
void
MSSimpleTrafficLightLogic::changeStepAndDuration(MSTLLogicControl& tlcontrol,
        SUMOTime simStep, int step, SUMOTime stepDuration) {
    mySwitchCommand->deschedule(this);
    mySwitchCommand = new SwitchCommand(tlcontrol, this, stepDuration + simStep);
    if (step != myStep) {
        myStep = step;
        setTrafficLightSignals(simStep);
        tlcontrol.get(getID()).executeOnSwitchActions();
    }
    MSNet::getInstance()->getBeginOfTimestepEvents()->addEvent(
        mySwitchCommand, stepDuration + simStep);
}


void
MSSimpleTrafficLightLogic::setPhases(const Phases& phases, int step) {
    assert(step < (int)phases.size());
    deletePhases();
    myPhases = phases;
    myStep = step;
}


void
MSSimpleTrafficLightLogic::deletePhases() {
    for (int i = 0; i < (int)myPhases.size(); i++) {
        delete myPhases[i];
    }
}


/****************************************************************************/

