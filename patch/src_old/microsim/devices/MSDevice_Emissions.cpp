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
/// @file    MSDevice_Emissions.cpp
/// @author  Daniel Krajzewicz
/// @author  Laura Bieker
/// @author  Michael Behrisch
/// @date    Fri, 30.01.2009
/// @version $Id$
///
// A device which collects vehicular emissions
/****************************************************************************/

// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "MSDevice_Emissions.h"
#include <microsim/MSNet.h>
#include <microsim/MSLane.h>
#include <microsim/MSVehicleControl.h>
#include <utils/options/OptionsCont.h>
#include <utils/emissions/PollutantsInterface.h>
#include <utils/iodevices/OutputDevice.h>


// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// static initialisation methods
// ---------------------------------------------------------------------------
void
MSDevice_Emissions::insertOptions() {
    insertDefaultAssignmentOptions("emissions", "Emissions", OptionsCont::getOptions());
}


void
MSDevice_Emissions::buildVehicleDevices(SUMOVehicle& v, std::vector<MSDevice*>& into) {
    if (equippedByDefaultAssignmentOptions(OptionsCont::getOptions(), "emissions", v)) {
        // build the device
        MSDevice_Emissions* device = new MSDevice_Emissions(v, "emissions_" + v.getID());
        into.push_back(device);
    }
}


// ---------------------------------------------------------------------------
// MSDevice_Emissions-methods
// ---------------------------------------------------------------------------
MSDevice_Emissions::MSDevice_Emissions(SUMOVehicle& holder, const std::string& id)
    : MSDevice(holder, id), myEmissions() {
}


MSDevice_Emissions::~MSDevice_Emissions() {
}


bool
MSDevice_Emissions::notifyMove(SUMOVehicle& veh, double /*oldPos*/, double /*newPos*/, double newSpeed) {
    const SUMOEmissionClass c = veh.getVehicleType().getEmissionClass();
    const double a = veh.getAcceleration();
    const double slope = veh.getSlope();
    myEmissions.addScaled(PollutantsInterface::computeAll(c, newSpeed, a, slope), TS);
    return true;
}


void
MSDevice_Emissions::generateOutput() const {
    if (OptionsCont::getOptions().isSet("tripinfo-output")) {
        OutputDevice& os = OutputDevice::getDeviceByOption("tripinfo-output");
        (os.openTag("emissions") <<
         " CO_abs=\"" << OutputDevice::realString(myEmissions.CO, 6) <<
         "\" CO2_abs=\"" << OutputDevice::realString(myEmissions.CO2, 6) <<
         "\" HC_abs=\"" << OutputDevice::realString(myEmissions.HC, 6) <<
         "\" PMx_abs=\"" << OutputDevice::realString(myEmissions.PMx, 6) <<
         "\" NOx_abs=\"" << OutputDevice::realString(myEmissions.NOx, 6) <<
         "\" fuel_abs=\"" << OutputDevice::realString(myEmissions.fuel, 6) <<
         "\" electricity_abs=\"" << OutputDevice::realString(myEmissions.electricity, 6) <<
         "\"").closeTag();
    }
}



/****************************************************************************/

