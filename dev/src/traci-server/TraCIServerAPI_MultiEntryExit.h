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
/// @file    TraCIServerAPI_MultiEntryExit.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    07.05.2009
/// @version $Id$
///
// APIs for getting/setting multi-entry/multi-exit detector values via TraCI
/****************************************************************************/
#ifndef TraCIServerAPI_MultiEntryExit_h
#define TraCIServerAPI_MultiEntryExit_h


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
 * @class TraCIServerAPI_MultiEntryExit
 * @brief APIs for getting/setting multi-entry/multi-exit detector values via TraCI
 */
class TraCIServerAPI_MultiEntryExit {
public:
    /** @brief Processes a get value command (Command 0xa1: Get MeMeDetector Variable)
     *
     * @param[in] server The TraCI-server-instance which schedules this request
     * @param[in] inputStorage The storage to read the command from
     * @param[out] outputStorage The storage to write the result to
     */
    static bool processGet(TraCIServer& server, tcpip::Storage& inputStorage,
                           tcpip::Storage& outputStorage);


private:
    /// @brief invalidated copy constructor
    TraCIServerAPI_MultiEntryExit(const TraCIServerAPI_MultiEntryExit& s);

    /// @brief invalidated assignment operator
    TraCIServerAPI_MultiEntryExit& operator=(const TraCIServerAPI_MultiEntryExit& s);


};


#endif

#endif

/****************************************************************************/

