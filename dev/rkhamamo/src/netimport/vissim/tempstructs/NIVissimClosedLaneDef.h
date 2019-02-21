/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2002-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    NIVissimClosedLaneDef.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id$
///
// -------------------
/****************************************************************************/
#ifndef NIVissimClosedLaneDef_h
#define NIVissimClosedLaneDef_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif



// ===========================================================================
// class definitions
// ===========================================================================
/**
 *
 */
class NIVissimClosedLaneDef {
public:
    NIVissimClosedLaneDef(const std::vector<int>& assignedVehicles);
    ~NIVissimClosedLaneDef();
private:
    std::vector<int> myAssignedVehicles;
};


#endif

/****************************************************************************/

