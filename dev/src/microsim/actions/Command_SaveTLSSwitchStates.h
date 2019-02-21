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
/// @file    Command_SaveTLSSwitchStates.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    08.05.2007
/// @version $Id$
///
// Writes the switch times of a tls into a file when the tls switches
/****************************************************************************/
#ifndef Command_SaveTLSSwitchStates_h
#define Command_SaveTLSSwitchStates_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <map>
#include <utils/common/Command.h>
#include <microsim/traffic_lights/MSTLLogicControl.h>


// ===========================================================================
// class declarations
// ===========================================================================
class MSTrafficLightLogic;
class OutputDevice;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class Command_SaveTLSSwitchStates
 * @brief Writes the switch times of a tls into a file when the tls switches
 *
 * @todo Revalidate this - as tls are not seting the link information directly ater being switched, the computed information may be delayed
 */
class Command_SaveTLSSwitchStates : public Command {
public:
    /** @brief Constructor
     *
     * @param[in] tlls The logic to write state of
     * @param[in] od The output device to write the state into
     */
    Command_SaveTLSSwitchStates(const MSTLLogicControl::TLSLogicVariants& logics,
                                OutputDevice& od);


    /// @brief Destructor
    ~Command_SaveTLSSwitchStates();


    /// @name Derived from Command
    /// @{

    /** @brief Writes the state of the tls if a change occured
     *
     * If the state or the active program has changed, the state is written
     *  to the output device.
     *
     * @param[in] currentTime The current simulation time
     * @return Always DELTA_T (will be executed in next time step)
     * @see Command
     * @todo Here, a discrete even (on switch / program change) would be appropriate
     */
    SUMOTime execute(SUMOTime currentTime);
    /// @}


private:
    /// @brief The device to write to
    OutputDevice& myOutputDevice;

    /// @brief The traffic light logic to use
    const MSTLLogicControl::TLSLogicVariants& myLogics;

    /// @brief Storage for prior state
    std::string myPreviousState;

    /// @brief Storage for prior sub-id
    std::string myPreviousProgramID;


private:
    /// @brief Invalidated copy constructor.
    Command_SaveTLSSwitchStates(const Command_SaveTLSSwitchStates&);

    /// @brief Invalidated assignment operator.
    Command_SaveTLSSwitchStates& operator=(const Command_SaveTLSSwitchStates&);

};


#endif

/****************************************************************************/

