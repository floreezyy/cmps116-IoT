/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2010-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    MSSOTLWaveTrafficLightLogic.cpp
/// @author  Riccardo Belletti
/// @author  Anna Chiara Bellini
/// @date    Sep 2013
/// @version $Id$
///
// The class for SOTL Platoon logics
/****************************************************************************/

#include "MSSOTLWaveTrafficLightLogic.h"

MSSOTLWaveTrafficLightLogic::MSSOTLWaveTrafficLightLogic(
    MSTLLogicControl& tlcontrol, const std::string& id,
    const std::string& subid, const Phases& phases, int step,
    SUMOTime delay,
    const std::map<std::string, std::string>& parameters) throw() :
    MSSOTLTrafficLightLogic(tlcontrol, id, subid, phases, step, delay,
                            parameters) {
    MsgHandler::getMessageInstance()->inform(
        "*** Intersection " + id
        + " will run using MSSOTLWaveTrafficLightLogic ***");
    //sets the lastDuration of every phase to the same value as the default duration of that phase
    for (int i = 0; i < getPhaseNumber(); i++) {
        (*myPhases[i]).lastDuration = (*myPhases[i]).duration;
    }
}

MSSOTLWaveTrafficLightLogic::MSSOTLWaveTrafficLightLogic(
    MSTLLogicControl& tlcontrol, const std::string& id,
    const std::string& subid, const Phases& phases, int step,
    SUMOTime delay, const std::map<std::string, std::string>& parameters,
    MSSOTLSensors* sensors) throw() :
    MSSOTLTrafficLightLogic(tlcontrol, id, subid, phases, step, delay,
                            parameters, sensors) {
    //sets the lastDuration of every phase to the same value as the default duration of that phase
    for (int i = 0; i < getPhaseNumber(); i++) {
        (*myPhases[i]).lastDuration = (*myPhases[i]).duration;
    }
}

bool MSSOTLWaveTrafficLightLogic::canRelease() throw() {

    //10% of lastDuration
    SUMOTime delta = 10 * getCurrentPhaseDef().lastDuration / 100;

    //this allows a minimum variation of +-1s
    if (delta < 1000) {
        delta = 1000;
    }
    if (getCurrentPhaseElapsed() >= getCurrentPhaseDef().minDuration) {
        if (getCurrentPhaseElapsed()
                >= getCurrentPhaseDef().lastDuration - delta) {
            if ((countVehicles() == 0) //no other vehicles approaching green lights
                    || (getCurrentPhaseElapsed()
                        >= getCurrentPhaseDef().lastDuration + delta) //maximum value of the window surrounding lastDuration
                    || (getCurrentPhaseElapsed()
                        >= getCurrentPhaseDef().maxDuration) //declared maximum duration has been reached
               ) {

                (*myPhases[getCurrentPhaseIndex()]).lastDuration =
                    getCurrentPhaseElapsed();
                return true;
            }
        }
    }
    return false;
}

int MSSOTLWaveTrafficLightLogic::countVehicles() {
    std::string state = getCurrentPhaseDef().getState();
    int vehicles = 0;
    for (int i = 0; i < (int)getLaneVectors().size(); i++) {
        if (i > 0
                && ((getLaneVectors()[i][0]->getID()).compare(
                        getLaneVectors()[i - 1][0]->getID()) == 0)) {
            continue;
        }
        if (state[i] != 'r') {
            vehicles += getSensors()->countVehicles(getLaneVectors()[i][0]);
        }

    }
    return vehicles;
}
