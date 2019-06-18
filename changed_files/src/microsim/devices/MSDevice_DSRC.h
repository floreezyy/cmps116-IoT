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
/// @file    MSDevice_DSRC.h
/// @author  Alexis Flores

/// based on the work from the example device provided by:
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
///
///
/// @date    05.27.2019
/// @version $Id$
///
// A device which generates Basic Safety Messages of a vehicle and outputs them 
// to console or Road Side Unit files specified by the User
/****************************************************************************/
#ifndef MSDevice_DSRC_h
#define MSDevice_DSRC_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "MSDevice.h"
#include <utils/common/SUMOTime.h>

// 3 currently implemented states for TransmissionStateStatus based on vehicle speed
#define PARKED 0
#define DRIVE 1
#define REVERSE 2

// 2 currently implemented states of BrakeSystemStatus parameter based on vehicle speed
#define BRAKES_OFF 0 // brakes are on
#define BRAKES_ON 1 // brakes are off

// hardcoded range coordinates for each of the RSU polygons found inside the additional xml
// file found inside the light_run simulation folder. MIN and MAX coordinates are used to calculate
// the signal strength of the Road Side Units. The farther the vehicle is from the MIN/MAX coordinate
// of the RSU device the weaker the signal will be. The closer it is the stronger the signal will be.
// These coordinates can be modified depending on where (long x, lat y) the RSU polygons are located.
// 
// In this example, the additional file in light_run folder has three RSU polygons located at:
//
// RSU 1: (356.57, 594.94)
// RSU 2: (495.03, 291.76)
// RSU 3: (254.99, 306.39)

#define NUM_RSU_ACTIVE 3 // number of RSU devices inside light_run simulation, will create X number of RSU message files

#define RSU1_X_COORDINATE 356.57 // absolute x coordinate of the RSU 1 polygon in the simulation
#define RSU1_Y_COORDINATE 594.94 // absolute y coordinate of the RSU 1 polygon in the simulation
#define RSU1_X_COR_MIN_RANGE 300 // these values are the ranges that determine RSU signal strength 
#define RSU1_Y_COR_MIN_RANGE 550
#define RSU1_X_COR_MAX_RANGE 400
#define RSU1_Y_COR_MAX_RANGE 650

#define RSU2_X_COORDINATE 495.03 // absolute x coordinate of the RSU 2 polygon in the simulation
#define RSU2_Y_COORDINATE 291.76 // absolute y coordinate of the RSU 2 polygon in the simulation
#define RSU2_X_COR_MIN_RANGE 450 // these values are the ranges that determine RSU signal strength 
#define RSU2_Y_COR_MIN_RANGE 250
#define RSU2_X_COR_MAX_RANGE 550
#define RSU2_Y_COR_MAX_RANGE 350

#define RSU3_X_COORDINATE 254.99 // absolute x coordinate of the RSU 3 polygon in the simulation
#define RSU3_Y_COORDINATE 306.39 // absolute y coordinate of the RSU 3 polygon in the simulation
#define RSU3_X_COR_MIN_RANGE 200 // these values are the ranges that determine RSU signal strength 
#define RSU3_Y_COR_MIN_RANGE 250
#define RSU3_X_COR_MAX_RANGE 300
#define RSU3_Y_COR_MAX_RANGE 350

// vehicles will pass through 3 different RSU ranges and will output to the corresponding file based on the location domain
#define NONE 0
#define RSU_1_DOMAIN 1
#define RSU_2_DOMAIN 2
#define RSU_3_DOMAIN 3

//These defines are used for the signal strength of the vehicle with the DSRC device
// installed to the Road Side Unit within the light_run simulation file

#define SIGNAL_WEAK 50 // vehicle is at a far distance from the Road Side Unit, messages might be lost
#define SIGNAL_MEDIUM 75 // vehicle is within acceptable range for receiving messages without issues
#define SIGNAL_STRONG 90 // vehicle is close to RSU and signal strength is best

// ===========================================================================
// class declarations
// ===========================================================================
class SUMOVehicle;
class MSVehicle; // need for breake lights

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSDevice_DSRC
 * @brief A device which collects info on Basic Safety Messages for DSRC applications
 *
 * Each device collects safety information from vehicle attached 
 *
 * @see MSDevice
 */
class MSDevice_DSRC : public MSDevice {
public:
    /** @brief Inserts MSDevice_DSRC-options
     * @param[filled] oc The options container to add the options to
     */
    static void insertOptions(OptionsCont& oc);


    /** @brief Build devices for the given vehicle, if needed
     *
     * The options are read and evaluated whether a DSRC-device shall be built
     *  for the given vehicle.
     *
     * The built device is stored in the given vector.
     *
     * @param[in] v The vehicle for which a device may be built
     * @param[filled] into The vector to store the built device in
     */
    static void buildVehicleDevices(SUMOVehicle& v, std::vector<MSDevice*>& into);
    // may be able to appy N DSRC devices using the device vector?


public:
    /// @brief Destructor.
    ~MSDevice_DSRC();

    
    /** @brief
     * A basic getHelper function that calculates the current brakesystem status status
     * of the vehicle based on the speed of the vehicle at time t and t-1. If the
     * current vehicle speed is less than the previous speed at time t-1, we can
     * assume that the car is slowing down 
     * 
     * NOTE: brake system is not currently supported in SUMO so braking status returned
     *       is completely based on the changing speed of the vehicle
     * 
     * @param[in] prevSpeed is the speed of the vehicle at the previous timestep t-1
     * @param[in] currSpeed is the speed of the vehicle at the current timestep t
     *
     * @return the BrakeSystem status of the vehicle based on previous and current speed
     */
    int getBrakeSystemStatus(double prevSpeed, double currSpeed);

    /** @brief
     * A basic getHelper function that calculates the current TransmissionStateSystem status
     * of the vehicle based on the speed of the vehicle at time t. If the
     * current vehicle speed is 0, then the car is PARKED. Less than 0 we assume the vehicle is
     * in REVERSE, or if speed is positive, then the vehicle is in the DRIVE state.
     *
     * NOTE: transmission status is not currently supported in SUMO so the transmission status
     *       returned is completely based on the changing speed of the vehicle
     * 
     * @param[in] currSpeed is the speed of the vehicle at the current timestep t
     *
     * @return the transmission status of the vehicle based current speed
     */
    int getTransmissionStatus(double currSpeed);
    

    /** @brief Checks if vehicle is within the range of an Road Side Unit
     * currently there is no interobject communication between a vehicle and Road Side Unit,
     * so as a temporary "hacky" solution is to hardcode the location of a RSU 
     * (found inside the additional files) in the simulatio and output data to a file 
     * when it comes across the vicinity of the RSU
     * 
     * NOTE: x_coordinate and y_coordinate are the x and y coordinates of the vehicle.
     *       this will check the hardcoded #define'd values on lines 65-90 and return
     *       whether the vehicle is inside the range of hardcoded values
     * 
     * @param[in] x_coordinate is the current longitudal coordiate of the vehicle
     * @param[in] y_coordinate is the current latitudal coordiate of the vehicle
     *
     * @return if the vehicle is within the vicinity of an RSU or near none
     */
    int RoadSideUnitDetect(double X_coordinate, double y_coordinate);

    /** @brief 
     * Calculates the signal stregth of the Vehicle to the RSU using the
     *  distance formula. Signam strength is determined by how far a vehicle
     * is from the x, y coordinates of the RSU
     * 
     * @param[in] x_coordinate is the current longitudal coordiate of the vehicle
     * @param[in] y_coordinate is the current latitudal coordiate of the vehicle
     * @param[in] x_rsu_coordinate is the current longitudal coordiate of the RSU
     * @param[in] y_rsu_coordinate is the current latitudal coordiate of the RSU
     * @return signal stregth of the connection between the vehicle and RSU
     */
    int RSUSignalStrength(double x_coordinate, double y_coordinate, double x_rsu_coordinate, double y_rsu_coordinate);
    /// @name Methods called on vehicle movement / state change, overwriting MSDevice
    /// @{

    /** @brief Checks for waiting steps when the vehicle moves
     *
     * @param[in] veh Vehicle that asks this reminder.
     * @param[in] oldPos Position before move.
     * @param[in] newPos Position after move with newSpeed.
     * @param[in] newSpeed Moving speed.
     *
     * @return True (always).
     */
    bool notifyMove(SUMOVehicle& veh, double oldPos,
                    double newPos, double newSpeed);


    /// @brief return the name for this type of device
    const std::string deviceName() const {
        return "dsrc";
    }

    /// @brief try to retrieve the given parameter from this device. Throw exception for unsupported key
    std::string getParameter(const std::string& key) const;

    /// @brief try to set the given parameter for this device. Throw exception for unsupported key
    void setParameter(const std::string& key, const std::string& value);

    /** @brief Called on writing tripinfo output
     *
     * @param[in] os The stream to write the information into
     * @exception IOError not yet implemented
     * @see MSDevice::generateOutput
     */
    void generateOutput() const;

    

private:
    /** @brief Constructor
     *
     * @param[in] holder The vehicle that holds this device
     * @param[in] id The ID of the device
     */
    MSDevice_DSRC(SUMOVehicle& holder, const std::string& id, double customValue1,
                     double customValue2, double customValue3);



private:
    // private state members of the DSRC device, not really used for anything. Based on example device.

    /// @brief a value which is initialised based on a commandline/configuration option
    double myCustomValue1;

    /// @brief a value which is initialised based on a vehicle parameter
    double myCustomValue2;

    /// @brief a value which is initialised based on a vType parameter
    double myCustomValue3;



private:
    /// @brief Invalidated copy constructor.
    MSDevice_DSRC(const MSDevice_DSRC&);

    /// @brief Invalidated assignment operator.
    MSDevice_DSRC& operator=(const MSDevice_DSRC&);


};


#endif

/****************************************************************************/
