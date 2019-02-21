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
/// @file    NIVissimSingleTypeParser_Fahrzeugklassendefinition.cpp
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
#include <utils/common/ToString.h>
#include <utils/common/VectorHelper.h>
#include "../NIImporter_Vissim.h"
#include "../tempstructs/NIVissimVehTypeClass.h"
#include "NIVissimSingleTypeParser_Fahrzeugklassendefinition.h"


// ===========================================================================
// method definitions
// ===========================================================================
NIVissimSingleTypeParser_Fahrzeugklassendefinition::NIVissimSingleTypeParser_Fahrzeugklassendefinition(
    NIImporter_Vissim& parent, NIImporter_Vissim::ColorMap& colorMap)
    : NIImporter_Vissim::VissimSingleTypeParser(parent),
      myColorMap(colorMap) {}


NIVissimSingleTypeParser_Fahrzeugklassendefinition::~NIVissimSingleTypeParser_Fahrzeugklassendefinition() {}


bool
NIVissimSingleTypeParser_Fahrzeugklassendefinition::parse(std::istream& from) {
    // id
    int id;
    from >> id; // type-checking is missing!
    // name
    std::string tag;
    from >> tag;
    std::string name = readName(from);
    // color
    from >> tag;
    std::string colorName = myRead(from);
    RGBColor color;
    NIImporter_Vissim::ColorMap::iterator i = myColorMap.find(colorName);
    if (i != myColorMap.end()) {
        color = (*i).second;
    } else {
        int r, g, b;
        r = TplConvert::_2int(colorName.c_str());
        if (!(from >> g)) {
            throw NumberFormatException();
        }
        if (!(from >> b)) {
            throw NumberFormatException();
        }
        if (r < 0 || r > 255 || g < 0 || g > 255 || b < 0 || b > 255) {
            throw NumberFormatException();
        }
        color = RGBColor((unsigned char)r, (unsigned char)g, (unsigned char)b, 255);
    }
    // types
    from >> tag;
    if (tag == "ANM_ID") {
        readName(from);
        from >> tag;
    }
    std::vector<int> types;
    from >> tag;
    do {
        types.push_back(TplConvert::_2int(tag.c_str()));
        tag = readEndSecure(from);
    } while (tag != "DATAEND");
    return NIVissimVehTypeClass::dictionary(id, name, color, types);
}



/****************************************************************************/

