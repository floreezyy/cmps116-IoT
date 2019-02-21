/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2017-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    MultiEntryExit.h
/// @author  Michael Behrisch
/// @date    15.03.2017
/// @version $Id$
///
// C++ TraCI client API implementation
/****************************************************************************/
#ifndef MultiEntryExit_h
#define MultiEntryExit_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <vector>


// ===========================================================================
// class declarations
// ===========================================================================
class MSE2Collector;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MultiEntryExit
 * @brief C++ TraCI client API implementation
 */
namespace libsumo {
class MultiEntryExit {
public:
    static std::vector<std::string> getIDList();
    static int getIDCount();
    static int getLastStepVehicleNumber(const std::string& detID);
    static double getLastStepMeanSpeed(const std::string& detID);
    static std::vector<std::string> getLastStepVehicleIDs(const std::string& detID);
    static int getLastStepHaltingNumber(const std::string& detID);

private:
    static MSE3Collector* getDetector(const std::string& detID);

    /// @brief invalidated standard constructor
    MultiEntryExit();

    /// @brief invalidated copy constructor
    MultiEntryExit(const MultiEntryExit& src);

    /// @brief invalidated assignment operator
    MultiEntryExit& operator=(const MultiEntryExit& src);

};
}


#endif

/****************************************************************************/
