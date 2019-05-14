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



#include <string>
#include <iomanip>
#include <sstream>
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
// module-level variables
// ===========================================================================
int msg_num = 0; // message count for DSRC communication standards
int brakeStatus = 0; // variable holding the brake status information
int transStatus = 0; // variable holding the transmission state information
int rsu_domain = NONE; // holds information on the vehicles current location in terms of RSU domains
int rsu_detected = NONE;
double rsu_x_coordinate = 0;
double rsu_y_coordinate = 0;
double rsu_sig_strength = 0; // calculates the signal strength of the RSU to the vehicle
// ===========================================================================
// method definitions
// ===========================================================================

int
MSDevice_DSRC::getBrakeSystemStatus(double prevSpeed, double currSpeed){
    
    int brakeStatus; // returns if brakes are on or off
    if(prevSpeed > currSpeed){ // if the speed is slowwing down, then brakes are on
        brakeStatus = BRAKES_ON;
    }
    else{ // the vehicle is not slowing down, so brakes must be on
        brakeStatus = BRAKES_OFF;
    }
    return brakeStatus; // return the brake status of the vehicle  
}



int
MSDevice_DSRC::getTransmissionStatus(double currSpeed){
    
    int transStatus; // variable that holds the current transmission status of the vehicle
    if(currSpeed != 0.0){ // if the speed is positive and non zero, car is driving
        transStatus = DRIVE;
    }
    else if(currSpeed < 0.0){ // if vehicle speed is negative, meaning car is backtracking
        transStatus = REVERSE; // transmission is in reverse
    }
    else{
        transStatus = PARKED; // car speed is 0.0, so it is parked or vehicle has stopped
    }
    return transStatus;

    // Need to find other ways to find REVERSE and NEUTRAL to better represent transmissionStatus
}



int 
MSDevice_DSRC::RoadSideUnitDetect(double x_coordinate, double y_coordinate){

    int rsu_detected = 0; // signal if the vehicle is near a RSU vicinity
    
    // if the vehicles current coordinates are within the range of RSU 1
    if (((x_coordinate >= RSU1_X_COR_MIN_RANGE) && (x_coordinate <= RSU1_X_COR_MAX_RANGE)) &&
        ((y_coordinate >= RSU1_Y_COR_MIN_RANGE) && (y_coordinate <= RSU1_Y_COR_MAX_RANGE))) {
        
        rsu_detected = RSU_1_DOMAIN; // vehicle IS in the vicinity of RSU1

    }
    // if the vehicles current coordinates are within the range of RSU 2
    else if (((x_coordinate >= RSU2_X_COR_MIN_RANGE) && (x_coordinate <= RSU2_X_COR_MAX_RANGE)) &&
        ((y_coordinate >= RSU2_Y_COR_MIN_RANGE) && (y_coordinate <= RSU2_Y_COR_MAX_RANGE))) {
        
        rsu_detected = RSU_2_DOMAIN; // vehicle IS in the vicinity of RSU2

    }
    // if the vehicles current coordinates are within the range of RSU 3
    else if (((x_coordinate >= RSU3_X_COR_MIN_RANGE) && (x_coordinate <= RSU3_X_COR_MAX_RANGE)) &&
        ((y_coordinate >= RSU3_Y_COR_MIN_RANGE) && (y_coordinate <= RSU3_Y_COR_MAX_RANGE))) {
        
        rsu_detected = RSU_3_DOMAIN; // vehicle IS in the vicinity of RSU3

    }
    else{ // the vehicle is currently not in the range of any RSU mentioned within light_run sim
        rsu_detected = NONE; 
    }
    return rsu_detected;
}

int 
MSDevice_DSRC::RSUSignalStrength(double x_coordinate, double y_coordinate, double x_rsu_coordinate, double y_rsu_coordinate){

    double signal_strength = 0;

	double x_distance = x_coordinate - x_rsu_coordinate;
	double y_distance = y_coordinate - y_rsu_coordinate;
	
	double abs_distance = sqrt((x_distance*x_distance) + (y_distance*y_distance));
    signal_strength = 100.0 - (abs_distance * 2);
    
    return abs(signal_strength);
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
    
    int i = 0; // counter for number of RSU files to generate
    //std::ostringstream file_name;
    std::string file_name;
    for (i = 1; i <= NUM_RSU_ACTIVE; i++) // for every RSU active on simulation, create own RSU file
    {
        std::stringstream file_name;
        file_name << "RSU_" << std::to_string(i) << "_dsrc_pkt_received" << ".csv";
        //std::string query(file_name.str());
        std::ofstream file(file_name.str()); // make string into file name

        // data is formatted as a csv file for processing, data collecting purposes
        file << "MsgCount,TemporaryID,VehicleID,Vehicle Type,Vehicle Length,Vehicle Width," << 
        "Vehicle Height,Vehicle Wheel Angle,Longitude,Latitude,Vehicle Speed,Vehicle Acceleration," << 
        "Vehicle Slope Angle,BrakeSystemStatus,TransmissionState,Dsecond,RSU_ID,RSU_SIG_STRENGTH" << "\n"; 
    }
}


MSDevice_DSRC::~MSDevice_DSRC() {
}


bool
MSDevice_DSRC::notifyMove(SUMOVehicle& veh, double /* oldPos */,
                             double /* newPos */, double newSpeed) {
    
    std::string rsu_file;                       
    rsu_domain = RoadSideUnitDetect(veh.getPosition().x(), veh.getPosition().y());
    switch(rsu_domain){
        case RSU_1_DOMAIN:
            rsu_file = "RSU_1_dsrc_pkt_received.csv";
            rsu_x_coordinate = RSU1_X_COORDINATE;
            rsu_y_coordinate = RSU1_Y_COORDINATE;
            break;
        case RSU_2_DOMAIN:
            rsu_file = "RSU_2_dsrc_pkt_received.csv";
            rsu_x_coordinate = RSU2_X_COORDINATE;
            rsu_y_coordinate = RSU2_Y_COORDINATE;
            break;
        case RSU_3_DOMAIN:
            rsu_file = "RSU_3_dsrc_pkt_received.csv";
            rsu_x_coordinate = RSU3_X_COORDINATE;
            rsu_y_coordinate = RSU3_Y_COORDINATE;
            break;
    }
    rsu_detected = NONE;
    if(rsu_domain != NONE){
        rsu_detected = rsu_domain;
    }
    //std::string rsu_id = "RSU_" << rsu_domain;
    msg_num++; //increment the message id
    //std::string file_name = "rsu_incoming_msg.csv";                    
    std::ofstream dsrcfile (rsu_file, std::ios_base::app);
    
    //std::cout << "vehicleID: " << veh.getID() << " MoveUpdate: Speed=" << newSpeed << "\n";
    // check whether another device is present on the vehicle:
    MSDevice_Tripinfo* otherDevice = static_cast<MSDevice_Tripinfo*>(veh.getDevice(typeid(MSDevice_Tripinfo)));
    // if (otherDevice != 0) {
    //     std::cout << "  veh '" << veh.getID() << " has device '" << otherDevice->getID() << "'\n";
    // }

    
    MSVehicle& sus = dynamic_cast<MSVehicle&>(veh);
    int veh_curr_speed = sus.getSpeed();
    int veh_prev_speed = sus.getPreviousSpeed();
    
    //std::cout << "last speed: " <<  << " curr speed: " << sped << std::endl;
    //std::cout << "if this works the speed is: " << sped << "\n";
    //std::cout << "just to check, signals: " << sus.getSignals() << "\n";
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

    if(rsu_detected != NONE){
        //std::cout << "VEHICLE IS IN THE VICINITY OF RSU 1 \n";
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

        dsrcfile << std::ctime(&end_time) << ",";
        dsrcfile << "rsu_" << rsu_domain << ",";
        rsu_sig_strength = RSUSignalStrength(veh.getPosition().x(), veh.getPosition().y(), rsu_x_coordinate, rsu_y_coordinate);
        dsrcfile << rsu_sig_strength;
        dsrcfile << std::endl;
    }
    
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
