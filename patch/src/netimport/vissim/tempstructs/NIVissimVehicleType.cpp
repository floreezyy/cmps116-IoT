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
/// @file    NIVissimVehicleType.cpp
/// @author  Daniel Krajzewicz
/// @date    Sept 2002
/// @version $Id$
///
// -------------------
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif


#include "NIVissimVehicleType.h"

NIVissimVehicleType::DictType NIVissimVehicleType::myDict;

NIVissimVehicleType::NIVissimVehicleType(const std::string& name,
        const std::string& category, const RGBColor& color)
    : myName(name), myCategory(category),
      myColor(color) {}


NIVissimVehicleType::~NIVissimVehicleType() {}



bool
NIVissimVehicleType::dictionary(int id, const std::string& name, const std::string& category,
                                const RGBColor& color) {
    NIVissimVehicleType* o = new NIVissimVehicleType(name, category, color);
    if (!dictionary(id, o)) {
        delete o;
        return false;
    }
    return true;
}


bool
NIVissimVehicleType::dictionary(int id, NIVissimVehicleType* o) {
    DictType::iterator i = myDict.find(id);
    if (i == myDict.end()) {
        myDict[id] = o;
        return true;
    }
    return false;
}


NIVissimVehicleType*
NIVissimVehicleType::dictionary(int id) {
    DictType::iterator i = myDict.find(id);
    if (i == myDict.end()) {
        return 0;
    }
    return (*i).second;
}

void
NIVissimVehicleType::clearDict() {
    for (DictType::iterator i = myDict.begin(); i != myDict.end(); i++) {
        delete(*i).second;
    }
    myDict.clear();
}



/****************************************************************************/

