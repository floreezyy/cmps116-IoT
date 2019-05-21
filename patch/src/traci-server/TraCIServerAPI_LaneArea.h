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
/// @file    TraCIServerAPI_LaneArea.h
/// @author  Mario Krumnow
/// @author  Michael Behrisch
/// @date    15.09.2013
/// @version $Id$
///
// APIs for getting/setting areal detector values via TraCI
/****************************************************************************/
#ifndef TraCIServerAPI_LaneArea_h
#define TraCIServerAPI_LaneArea_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#ifndef NO_TRACI

#include <foreign/tcpip/storage.h>


// ===========================================================================
// class declarations
// ===========================================================================
class TraCIServer;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class TraCIServerAPI_AreaDetector
 * @brief APIs for getting/setting multi-entry/multi-exit detector values via TraCI
 */
class TraCIServerAPI_LaneArea {
public:
    /** @brief Processes a get value command (Command 0xa1: Get AreaDetector Variable)
     *
     * @param[in] server The TraCI-server-instance which schedules this request
     * @param[in] inputStorage The storage to read the command from
     * @param[out] outputStorage The storage to write the result to
     */
    static bool processGet(TraCIServer& server, tcpip::Storage& inputStorage,
                           tcpip::Storage& outputStorage);


private:
    /// @brief invalidated copy constructor
    TraCIServerAPI_LaneArea(const TraCIServerAPI_LaneArea& s);

    /// @brief invalidated assignment operator
    TraCIServerAPI_LaneArea& operator=(const TraCIServerAPI_LaneArea& s);


};


#endif

#endif

/****************************************************************************/

