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
/// @file    NIVissimSingleTypeParser_Routenentscheidungsdefinition.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Wed, 18 Dec 2002
/// @version $Id$
///
//
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
#include <utils/common/TplConvert.h>
#include "../NIImporter_Vissim.h"
#include "NIVissimSingleTypeParser_Routenentscheidungsdefinition.h"


// ===========================================================================
// method definitions
// ===========================================================================
NIVissimSingleTypeParser_Routenentscheidungsdefinition::NIVissimSingleTypeParser_Routenentscheidungsdefinition(NIImporter_Vissim& parent)
    : NIImporter_Vissim::VissimSingleTypeParser(parent) {}


NIVissimSingleTypeParser_Routenentscheidungsdefinition::~NIVissimSingleTypeParser_Routenentscheidungsdefinition() {}


bool
NIVissimSingleTypeParser_Routenentscheidungsdefinition::parse(std::istream& from) {
    std::string tag;
    while (tag != "fahrzeugklassen") {
        tag = myRead(from);
    }
    do {
        while (tag != "DATAEND" || tag == "route") {
            if (tag == "route") {
                while (tag != "strecke") {
                    tag = myRead(from);
                }
                tag = readEndSecure(from);
            } else {
                tag = readEndSecure(from);
            }
        }
        if (tag != "DATAEND") {
            tag = readEndSecure(from);
        }
    } while (tag != "DATAEND");
    return true;
}



/****************************************************************************/

