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
/// @file    GUIMEVehicleControl.cpp
/// @author  Jakob Erdmann
/// @date    Okt 2012
/// @version $Id$
///
// The class responsible for building and deletion of meso vehicles (gui-version)
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <utils/foxtools/MFXMutex.h>
#include <utils/vehicle/SUMOVehicle.h>
#include <gui/GUIGlobals.h>
#include <microsim/MSRouteHandler.h>
#include "GUIMEVehicleControl.h"
#include "GUIMEVehicle.h"


// ===========================================================================
// member method definitions
// ===========================================================================
GUIMEVehicleControl::GUIMEVehicleControl()
    : MEVehicleControl() {}


GUIMEVehicleControl::~GUIMEVehicleControl() {
    // just to quit cleanly on a failure
    if (myLock.locked()) {
        myLock.unlock();
    }
}


SUMOVehicle*
GUIMEVehicleControl::buildVehicle(SUMOVehicleParameter* defs,
                                  const MSRoute* route, MSVehicleType* type,
                                  const bool ignoreStopErrors, const bool fromRouteFile) {
    myLoadedVehNo++;
    if (fromRouteFile) {
        defs->depart += computeRandomDepartOffset();
    }
    MSBaseVehicle* built = new GUIMEVehicle(defs, route, type, type->computeChosenSpeedDeviation(fromRouteFile ? MSRouteHandler::getParsingRNG() : 0));
    built->addStops(ignoreStopErrors);
    MSNet::getInstance()->informVehicleStateListener(built, MSNet::VEHICLE_STATE_BUILT);
    return built;
}



bool
GUIMEVehicleControl::addVehicle(const std::string& id, SUMOVehicle* v) {
    AbstractMutex::ScopedLocker locker(myLock);
    return MEVehicleControl::addVehicle(id, v);
}


void
GUIMEVehicleControl::deleteVehicle(SUMOVehicle* veh, bool discard) {
    AbstractMutex::ScopedLocker locker(myLock);
    MEVehicleControl::deleteVehicle(veh, discard);
}


void
GUIMEVehicleControl::insertVehicleIDs(std::vector<GUIGlID>& into) {
    AbstractMutex::ScopedLocker locker(myLock);
    into.reserve(myVehicleDict.size());
    for (VehicleDictType::iterator i = myVehicleDict.begin(); i != myVehicleDict.end(); ++i) {
        SUMOVehicle* veh = (*i).second;
        if (veh->isOnRoad()) {
            into.push_back(static_cast<GUIMEVehicle*>((*i).second)->getGlID());
        }
    }
}



void
GUIMEVehicleControl::secureVehicles() {
    myLock.lock();
}


void
GUIMEVehicleControl::releaseVehicles() {
    myLock.unlock();
}



/****************************************************************************/

