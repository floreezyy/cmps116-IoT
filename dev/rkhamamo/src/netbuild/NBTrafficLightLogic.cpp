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
/// @file    NBTrafficLightLogic.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id$
///
// A SUMO-compliant built logic for a traffic light
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
#include <bitset>
#include <utility>
#include <string>
#include <sstream>
#include <cassert>
#include "NBEdge.h"
#include "NBEdgeCont.h"
#include "NBTrafficLightLogic.h"
#include "NBTrafficLightDefinition.h"
#include <utils/options/OptionsCont.h>
#include <utils/options/Option.h>
#include <utils/common/ToString.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/StringTokenizer.h>
#include <utils/iodevices/OutputDevice.h>


// ===========================================================================
// static members
// ===========================================================================
const char NBTrafficLightLogic::allowedStatesInitializer[] = {LINKSTATE_TL_GREEN_MAJOR,
                                                              LINKSTATE_TL_GREEN_MINOR,
                                                              LINKSTATE_STOP, // used for NODETYPE_TRAFFIC_LIGHT_RIGHT_ON_RED
                                                              LINKSTATE_TL_RED,
                                                              LINKSTATE_TL_REDYELLOW,
                                                              LINKSTATE_TL_YELLOW_MAJOR,
                                                              LINKSTATE_TL_YELLOW_MINOR,
                                                              LINKSTATE_TL_OFF_BLINKING,
                                                              LINKSTATE_TL_OFF_NOSIGNAL
                                                             };

const std::string NBTrafficLightLogic::ALLOWED_STATES(NBTrafficLightLogic::allowedStatesInitializer, 9);

// ===========================================================================
// member method definitions
// ===========================================================================
NBTrafficLightLogic::NBTrafficLightLogic(const std::string& id,
        const std::string& subid, int noLinks,
        SUMOTime offset, TrafficLightType type) :
    Named(id), myNumLinks(noLinks), mySubID(subid),
    myOffset(offset),
    myType(type) {}

NBTrafficLightLogic::NBTrafficLightLogic(const NBTrafficLightLogic* logic) :
    Named(logic->getID()),
    myNumLinks(logic->myNumLinks),
    mySubID(logic->getProgramID()),
    myOffset(logic->getOffset()),
    myPhases(logic->myPhases.begin(), logic->myPhases.end()),
    myType(logic->getType()) {}


NBTrafficLightLogic::~NBTrafficLightLogic() {}

void
NBTrafficLightLogic::addStep(SUMOTime duration, const std::string& state, int index) {
    addStep(duration, state,
            NBTrafficLightDefinition::UNSPECIFIED_DURATION,
            NBTrafficLightDefinition::UNSPECIFIED_DURATION,
            index);
}

void
NBTrafficLightLogic::addStep(SUMOTime duration, const std::string& state, SUMOTime minDur, SUMOTime maxDur, int index) {
    // check state size
    if (myNumLinks == 0) {
        // initialize
        myNumLinks = (int)state.size();
    } else if ((int)state.size() != myNumLinks) {
        throw ProcessError("When adding phase to tlLogic '" + getID() + "': state length of " + toString(state.size()) +
                           " does not match declared number of links " + toString(myNumLinks));
    }
    // check state contents
    const std::string::size_type illegal = state.find_first_not_of(ALLOWED_STATES);
    if (std::string::npos != illegal) {
        throw ProcessError("When adding phase: illegal character '" + toString(state[illegal]) + "' in state");
    }
    // interpret index
    if (index < 0 || index >= (int)myPhases.size()) {
        // insert at the end
        index = (int)myPhases.size();
    }
    myPhases.insert(myPhases.begin() + index, PhaseDefinition(duration, state, minDur, maxDur));
}


void
NBTrafficLightLogic::deletePhase(int index) {
    if (index >= (int)myPhases.size()) {
        throw InvalidArgument("Index " + toString(index) + " out of range for logic with "
                              + toString(myPhases.size()) + " phases.");
    }
    myPhases.erase(myPhases.begin() + index);
}


void
NBTrafficLightLogic::setStateLength(int numLinks, LinkState fill) {
    if (myNumLinks > numLinks) {
        for (PhaseDefinition& p : myPhases) {
            p.state = p.state.substr(0, numLinks);
        }
    } else {
        std::string add(numLinks - myNumLinks, (char)fill);
        for (PhaseDefinition& p : myPhases) {
            p.state = p.state + add;
        }
    }
    myNumLinks = numLinks;
}


void
NBTrafficLightLogic::resetPhases() {
    myNumLinks = 0;
    myPhases.clear();
}


SUMOTime
NBTrafficLightLogic::getDuration() const {
    SUMOTime duration = 0;
    for (PhaseDefinitionVector::const_iterator i = myPhases.begin(); i != myPhases.end(); ++i) {
        duration += (*i).duration;
    }
    return duration;
}


void
NBTrafficLightLogic::closeBuilding(bool checkVarDurations) {
    for (int i = 0; i < (int)myPhases.size() - 1;) {
        if (myPhases[i].state != myPhases[i + 1].state) {
            ++i;
            continue;
        }
        myPhases[i].duration += myPhases[i + 1].duration;
        if (myPhases[i + 1].minDur != NBTrafficLightDefinition::UNSPECIFIED_DURATION) {
            if (myPhases[i].minDur != NBTrafficLightDefinition::UNSPECIFIED_DURATION) {
                myPhases[i].minDur += myPhases[i + 1].minDur;
            } else {
                myPhases[i].minDur = myPhases[i + 1].minDur;
            }
        }
        if (myPhases[i + 1].maxDur != NBTrafficLightDefinition::UNSPECIFIED_DURATION) {
            if (myPhases[i].maxDur != NBTrafficLightDefinition::UNSPECIFIED_DURATION) {
                myPhases[i].maxDur += myPhases[i + 1].maxDur;
            } else {
                myPhases[i].maxDur = myPhases[i + 1].maxDur;
            }
        }
        myPhases.erase(myPhases.begin() + i + 1);
    }
    // check if actuated lights are defined correctly
    if (checkVarDurations) {
        if (myType != TLTYPE_STATIC) {
            bool found = false;
            for (auto p : myPhases) {
                if (p.minDur != NBTrafficLightDefinition::UNSPECIFIED_DURATION
                        || p.maxDur != NBTrafficLightDefinition::UNSPECIFIED_DURATION) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                WRITE_WARNING("Non-static traffic light '" + getID() + "' does not define variable phase length.");
            }
        }
    }
}


void
NBTrafficLightLogic::setPhaseState(int phaseIndex, int tlIndex, LinkState linkState) {
    assert(phaseIndex < (int)myPhases.size());
    std::string& phaseState = myPhases[phaseIndex].state;
    assert(tlIndex < (int)phaseState.size());
    phaseState[tlIndex] = (char)linkState;
}


void
NBTrafficLightLogic::setPhaseDuration(int phaseIndex, SUMOTime duration) {
    assert(phaseIndex < (int)myPhases.size());
    myPhases[phaseIndex].duration = duration;
}

void
NBTrafficLightLogic::setPhaseMinDuration(int phaseIndex, SUMOTime duration) {
    assert(phaseIndex < (int)myPhases.size());
    myPhases[phaseIndex].minDur = duration;
}

void
NBTrafficLightLogic::setPhaseMaxDuration(int phaseIndex, SUMOTime duration) {
    assert(phaseIndex < (int)myPhases.size());
    myPhases[phaseIndex].maxDur = duration;
}


/****************************************************************************/

