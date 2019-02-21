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
/// @file    TraCIServerAPI_InductionLoop.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    07.05.2009
/// @version $Id$
///
// APIs for getting/setting induction loop values via TraCI
/****************************************************************************/
#ifndef TraCIServerAPI_InductionLoop_h
#define TraCIServerAPI_InductionLoop_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#ifndef NO_TRACI

#include "TraCIServer.h"
#include <foreign/tcpip/storage.h>


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class TraCIServerAPI_InductionLoop
 * @brief APIs for getting/setting induction loop values via TraCI
 */
class TraCIServerAPI_InductionLoop {
public:
    /** @brief Processes a get value command (Command 0xa0: Get Induction Loop Variable)
     *
     * @param[in] server The TraCI-server-instance which schedules this request
     * @param[in] inputStorage The storage to read the command from
     * @param[out] outputStorage The storage to write the result to
     */
    static bool processGet(TraCIServer& server, tcpip::Storage& inputStorage,
                           tcpip::Storage& outputStorage);


    /** @brief Returns the named inductive loop's position
     * @param[in] id The id of the searched inductive loop
     * @param[out] p The position, if the inductive loop is known
     * @return Whether the inductive loop is known (and on road)
     */
    static bool getPosition(const std::string& id, Position& p);


private:
    /// @brief invalidated copy constructor
    TraCIServerAPI_InductionLoop(const TraCIServerAPI_InductionLoop& s);

    /// @brief invalidated assignment operator
    TraCIServerAPI_InductionLoop& operator=(const TraCIServerAPI_InductionLoop& s);


};


#endif

#endif

/****************************************************************************/

