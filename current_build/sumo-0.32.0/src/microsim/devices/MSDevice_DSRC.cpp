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
// A basic getHelper function that calculates the current brakesystem status status
// of the vehicle based on the speed of the vehicle at time t and t-1. If the
// current vehicle speed is less than the previous speed at time t-1, we can
// assume that the car is slowing down 
int
MSDevice_DSRC::getBrakeSystemStatus(double prevSpeed, double currSpeed){
    
    int brakeStatus; // returns if brakes are on or off
    if(prevSpeed > currSpeed){ // if the speed is slowwinf down, then brakes are on
        brakeStatus = BRAKES_ON;
    }
    else{
        brakeStatus = BRAKES_OFF;
    }
    return brakeStatus;
}


// A basic getHelper function that calculates the current TransmissionStateSystem status status
// of the vehicle based on the speed of the vehicle at time t. If the
// current vehicle speed is 0, then the car is parked 
int
MSDevice_DSRC::getTransmissionStatus(double currSpeed){
    
    int transStatus; // returns if brakes are on or off
    if(currSpeed != 0.0){ // if the speed is slowwinf down, then brakes are on
        transStatus = DRIVE;
    }
    else{
        transStatus = PARKED;
    }
    return transStatus;

    // Need to find other ways to find REVERSE and NEUTRAL
}
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
    
    std::string file_name = "dsrc_out_" + holder.getID() + ".csv";   
    std::ofstream dsrcfile (file_name, std::ios_base::app);

    //Instantiate the json format by declaring the variables to use
    dsrcfile << "MsgCount,TemporaryID,VehicleID,Vehicle Type,Vehicle Length,Vehicle Width,Vehicle Height,Vehicle Wheel Angle,Longitude,Latitude,Vehicle Speed,Vehicle Acceleration,Vehicle Slope Angle,BrakeSystemStatus,TransmissionState,Dsecond" << "\n"; 
    //std::cout << "this should only pront once" << "\n";
}


MSDevice_DSRC::~MSDevice_DSRC() {
}

int msg_num = 0;
int brakeStatus = 0;
int transStatus = 0;
bool
MSDevice_DSRC::notifyMove(SUMOVehicle& veh, double /* oldPos */,
                             double /* newPos */, double newSpeed) {
    msg_num++; //increment the message id
    std::string file_name = "dsrc_out_" + veh.getID() + ".csv";                    
    std::ofstream dsrcfile (file_name, std::ios_base::app);

    //std::cout << "vehicleID: " << veh.getID() << " MoveUpdate: Speed=" << newSpeed << "\n";
    // check whether another device is present on the vehicle:
    MSDevice_Tripinfo* otherDevice = static_cast<MSDevice_Tripinfo*>(veh.getDevice(typeid(MSDevice_Tripinfo)));
    if (otherDevice != 0) {
        std::cout << "  veh '" << veh.getID() << " has device '" << otherDevice->getID() << "'\n";
    }

    
    MSVehicle& sus = dynamic_cast<MSVehicle&>(veh);
    int veh_curr_speed = sus.getSpeed();
    int veh_prev_speed = sus.getPreviousSpeed();
    
    
    //std::cout << "last speed: " <<  << " curr speed: " << sped << std::endl;
    //std::cout << "if this works the speed is: " << sped << "\n";
    std::cout << "just to check, signals: " << sus.getSignals() << "\n";
    //auto start = std::chrono::system_clock::now();
    // dsrcfile << "MsgCount: " << msg_num << std::endl;
    // dsrcfile << "TemporaryID: " << veh.getID() << "_" << msg_num << "\n" <<std::endl;
    // dsrcfile << "VehicleID: " << veh.getID() << std::endl;
    // dsrcfile << "Vehicle Type: " << veh.getVehicleType().getID() << std::endl;
    // dsrcfile << "Vehicle Length: " << veh.getVehicleType().getLength() << std::endl;
    // dsrcfile << "Vehicle Width: " << veh.getVehicleType().getWidth() << std::endl;
    // dsrcfile << "Vehicle Height: " << veh.getVehicleType().getHeight() << std::endl;
    // dsrcfile << "Vehicle Wheel Angle: " << veh.getAngle() << std::endl;
    // dsrcfile << "Longitude: " << veh.getPosition().x() << std::endl;
    // dsrcfile << "Latitude: " << veh.getPosition().y() << std::endl;
    // dsrcfile << "Vehicle Speed: " << sped << std::endl;
    // dsrcfile << "Vehicle Acceleration: " << veh.getAcceleration() << std::endl;
    // dsrcfile << "Vehicle Slope Angle: " << veh.getSlope() << std::endl;

    dsrcfile << msg_num << ","; //MsgCount
    dsrcfile << veh.getID() << "_" << msg_num << ","; //TemporaryID
    dsrcfile << veh.getID() << ","; // VehicleID
    dsrcfile << veh.getVehicleType().getID() << ","; // Vehicle Type
    dsrcfile << veh.getVehicleType().getLength() << ","; // Vehicle Length
    dsrcfile << veh.getVehicleType().getWidth() << ","; // Vehicle Width
    dsrcfile << veh.getVehicleType().getHeight() << ","; // Vehicle Height
    dsrcfile << veh.getAngle() << ","; // Vehicle Wheel Angle
    dsrcfile << veh.getPosition().x() << ","; // Vehicle Longitude
    dsrcfile << veh.getPosition().y() << ","; // Vehicle Latitude
    dsrcfile << veh_curr_speed << ","; // Vehicle Speed
    dsrcfile << veh.getAcceleration() << ","; //vehicle acceleration
    dsrcfile << veh.getSlope() << ","; // Vehicle Slope
    //dsrcfile << "time step is: " << SIMSTEP << std::endl;
    brakeStatus = getBrakeSystemStatus(veh_prev_speed, veh_curr_speed);
    if(brakeStatus == BRAKES_ON){
        dsrcfile << "BRAKES_ON" << ",";
    }
    else{
        dsrcfile << "BRAKES_OFF" << ",";
    }
    transStatus = getTransmissionStatus(veh_curr_speed);
    if(transStatus == PARKED){
        dsrcfile << "PARK" << ",";
    }
    else{
        dsrcfile << "DRIVE" << ",";
    }
    auto sent = std::chrono::system_clock::now();
    
    // for BSM standards data should be collected at lest 10 times a second
    std::time_t end_time = std::chrono::system_clock::to_time_t(sent);

    dsrcfile << std::ctime(&end_time);
    dsrcfile << std::endl;
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