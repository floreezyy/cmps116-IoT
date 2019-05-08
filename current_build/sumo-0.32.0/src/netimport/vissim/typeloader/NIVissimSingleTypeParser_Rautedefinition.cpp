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
/// @file    NIVissimSingleTypeParser_Rautedefinition.cpp
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
#include "../tempstructs/NIVissimSource.h"
#include "NIVissimSingleTypeParser_Rautedefinition.h"


// ===========================================================================
// method definitions
// ===========================================================================
NIVissimSingleTypeParser_Rautedefinition::NIVissimSingleTypeParser_Rautedefinition(NIImporter_Vissim& parent)
    : NIImporter_Vissim::VissimSingleTypeParser(parent) {}


NIVissimSingleTypeParser_Rautedefinition::~NIVissimSingleTypeParser_Rautedefinition() {}


bool
NIVissimSingleTypeParser_Rautedefinition::parse(std::istream& from) {
    std::string t;
    getline(from, t);
    return true;
}



/****************************************************************************/

