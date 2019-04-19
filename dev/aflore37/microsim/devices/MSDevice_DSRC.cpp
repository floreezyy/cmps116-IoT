/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2013-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    MSDevice_DSRC.cpp
/// @author  Alexis Flores, based on Daniel Krajzewicz's MSDevice_Example framework
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    11.06.2013
/// @version $Id$
///
// A device which stands as a barebones implementation example and which outputs movereminder calls
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
#include <fstream> //file creation to output BSM data
#include <chrono> //for calculating time of computations
#include <ctime> // used to get time of message creation completion

#include <utils/common/TplConvert.h>
#include <utils/options/OptionsCont.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/vehicle/SUMOVehicle.h>
#include <microsim/MSNet.h>
#include <microsim/MSLane.h>
#include <microsim/MSEdge.h>
#include <microsim/MSVehicle.h>
#include <utils/common/SUMOTime.h>
#include "MSDevice_Tripinfo.h"
#include "MSDevice_DSRC.h"


// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// static initialisation methods
// ---------------------------------------------------------------------------
void
MSDevice_DSRC::insertOptions(OptionsCont& oc) {
    oc.addOptionSubTopic("DSRC Device");
    insertDefaultAssignmentOptions("dsrc", "DSRC Device", oc);

    oc.doRegister("device.dsrc.parameter", new Option_Float(0.0));
    oc.addDescription("device.dsrc.parameter", "DSRC Device", "An exemplary parameter which can be used by all instances of the dsrc device");
}


void
MSDevice_DSRC::buildVehicleDevices(SUMOVehicle& v, std::vector<MSDevice*>& into) {
    OptionsCont& oc = OptionsCont::getOptions();
    if (equippedByDefaultAssignmentOptions(oc, "dsrc", v)) {
        // build the device
        // get custom vehicle parameter
        double customParameter2 = -1;
        if (v.getParameter().knowsParameter("dsrc")) {
            try {
                customParameter2 = TplConvert::_2double(v.getParameter().getParameter("dsrc", "-1").c_str());
            } catch (...) {
                WRITE_WARNING("Invalid value '" + v.getParameter().getParameter("dsrc", "-1") + "'for vehicle parameter 'dsrc'");
            }

        } else {
            //std::cout << "VehicleID: '" << v.getID() << "' does not supply vehicle parameter 'dsrc'. Using default of " << customParameter2 << "\n";
        }
        // get custom vType parameter
        double customParameter3 = -1;
        if (v.getVehicleType().getParameter().knowsParameter("dsrc")) {
            try {
                customParameter3 = TplConvert::_2double(v.getVehicleType().getParameter().getParameter("dsrc", "-1").c_str());
            } catch (...) {
                WRITE_WARNING("Invalid value '" + v.getVehicleType().getParameter().getParameter("dsrc", "-1") + "'for vType parameter 'dsrc'");
            }

        } else {
            //std::cout << "VehicleID '" << v.getID() << "' does not supply vType parameter 'dsrc'. Using default of " << customParameter3 << "\n";
        }

        MSDevice_DSRC* device = new MSDevice_DSRC(v, "dsrc_" + v.getID(),
                oc.getFloat("device.dsrc.parameter"),
                customParameter2,
                customParameter3);
        into.push_back(device);
    }
}


// ---------------------------------------------------------------------------
// MSDevice_DSRC-methods
// ---------------------------------------------------------------------------
MSDevice_DSRC::MSDevice_DSRC(SUMOVehicle& holder, const std::string& id,
                                   double customValue1, double customValue2, double customValue3) :
    MSDevice(holder, id),
    myCustomValue1(customValue1),
    myCustomValue2(customValue2),
    myCustomValue3(customValue3) {
    //std::cout << "initialized device '" << id << "' with myCustomValue1=" << myCustomValue1 << ", myCustomValue2=" << myCustomValue2 << ", myCustomValue3=" << myCustomValue3 << "\n";
}


MSDevice_DSRC::~MSDevice_DSRC() {
}

int msg_num = 0;

bool
MSDevice_DSRC::notifyMove(SUMOVehicle& veh, double /* oldPos */,
                             double /* newPos */, double newSpeed) {
    msg_num++; //increment the message id
    std::string file_name = "dsrc_out_" + veh.getID() + ".txt";                    
    std::ofstream dsrcfile (file_name, std::ios_base::app);

    //std::cout << "vehicleID: " << veh.getID() << " MoveUpdate: Speed=" << newSpeed << "\n";
    // check whether another device is present on the vehicle:
    MSDevice_Tripinfo* otherDevice = static_cast<MSDevice_Tripinfo*>(veh.getDevice(typeid(MSDevice_Tripinfo)));
    if (otherDevice != 0) {
        std::cout << "  veh '" << veh.getID() << " has device '" << otherDevice->getID() << "'\n";
    }
    //auto start = std::chrono::system_clock::now();
    dsrcfile << "DSRC MessageID: " << veh.getID() << "_" << msg_num << "\n" <<std::endl;
    dsrcfile << "vehicleID: " << veh.getID() << std::endl;
    dsrcfile << "Vehicle Type: " << veh.getVehicleType().getID() << std::endl;
    dsrcfile << "Vehicle Length: " << veh.getVehicleType().getLength() << std::endl;
    dsrcfile << "Vehicle Width: " << veh.getVehicleType().getWidth() << std::endl;
    dsrcfile << "Vehicle Height: " << veh.getVehicleType().getHeight() << std::endl;
    dsrcfile << "Vehicle Angle Trajectory: " << veh.getAngle() << std::endl;
    dsrcfile << "Vehicle Coordinates: " << veh.getPosition() << std::endl;
    dsrcfile << "Vehicle Speed: " << newSpeed << std::endl;
    dsrcfile << "Vehicle Acceleration: " << veh.getAcceleration() << std::endl;
    dsrcfile << "Vehicle Slope Angle: " << veh.getSlope() << std::endl;
    dsrcfile << "time step is: " << SIMSTEP << std::endl;

    auto sent = std::chrono::system_clock::now();
    
    //std::chrono::duration<double> elapsed_seconds = end-start;
    std::time_t end_time = std::chrono::system_clock::to_time_t(sent);

    dsrcfile << "Time sent: " << std::ctime(&end_time) << std::endl;
    dsrcfile << "\n\n**************************************************\n\n" << std::endl;
    return true; // keep the device
}


// bool
// MSDevice_DSRC::notifyEnter(SUMOVehicle& veh, MSMoveReminder::Notification reason, const MSLane* /* enteredLane */) {
//     std::cout << "device '" << getID() << "' notifyEnter: reason=" << reason << " currentEdge=" << veh.getEdge()->getID() << "\n";
//     return true; // keep the device
// }


bool
MSDevice_DSRC::notifyLeave(SUMOVehicle& veh, double /*lastPos*/, MSMoveReminder::Notification reason, const MSLane* /* enteredLane */) {
    std::cout << "device '" << getID() << "' notifyLeave: reason=" << reason << " currentEdge=" << veh.getEdge()->getID() << "\n";
    return true; // keep the device
}


void
MSDevice_DSRC::generateOutput() const {
    if (OptionsCont::getOptions().isSet("tripinfo-output")) {
        OutputDevice& os = OutputDevice::getDeviceByOption("tripinfo-output");
        os.openTag("dsrc_device");
        os.writeAttr("customValue1", toString(myCustomValue1));
        os.writeAttr("customValue2", toString(myCustomValue2));
        os.closeTag();
    }
}

std::string
MSDevice_DSRC::getParameter(const std::string& key) const {
    if (key == "customValue1") {
        return toString(myCustomValue1);
    } else if (key == "customValue2") {
        return toString(myCustomValue2);
    } else if (key == "meaningOfLife") {
        return "42";
    }
    throw InvalidArgument("Parameter '" + key + "' is not supported for device of type '" + deviceName() + "'");
}


void
MSDevice_DSRC::setParameter(const std::string& key, const std::string& value) {
    double doubleValue;
    try {
        doubleValue = TplConvert::_2double(value.c_str());
    } catch (NumberFormatException) {
        throw InvalidArgument("Setting parameter '" + key + "' requires a number for device of type '" + deviceName() + "'");
    }
    if (key == "customValue1") {
        myCustomValue1 = doubleValue;
    } else {
        throw InvalidArgument("Setting parameter '" + key + "' is not supported for device of type '" + deviceName() + "'");
    }
}


/****************************************************************************/

