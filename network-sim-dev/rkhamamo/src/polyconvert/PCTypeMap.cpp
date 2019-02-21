/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2005-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    PCTypeMap.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Mon, 05 Dec 2005
/// @version $Id$
///
// A storage for type mappings
/****************************************************************************/


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
#include "utils/options/OptionsCont.h"
#include "PCTypeMap.h"


// ===========================================================================
// method definitions
// ===========================================================================
PCTypeMap::PCTypeMap(const OptionsCont& oc) {
    myDefaultType.id = oc.getString("type");
    myDefaultType.color = RGBColor::parseColor(oc.getString("color"));
    myDefaultType.layer = oc.getFloat("layer");
    myDefaultType.discard = oc.getBool("discard");
    myDefaultType.allowFill = oc.getBool("fill");
    myDefaultType.prefix = oc.getString("prefix");
}


PCTypeMap::~PCTypeMap() {}


bool
PCTypeMap::add(const std::string& id, const std::string& newid,
               const std::string& color, const std::string& prefix,
               double layer, bool discard, bool allowFill) {
    if (has(id)) {
        return false;
    }
    TypeDef td;
    td.id = newid;
    td.color = RGBColor::parseColor(color);
    td.layer = layer;
    td.discard = discard;
    td.allowFill = allowFill;
    td.prefix = prefix;
    myTypes[id] = td;
    return true;
}


const PCTypeMap::TypeDef&
PCTypeMap::get(const std::string& id) {
    return myTypes.find(id)->second;
}


bool
PCTypeMap::has(const std::string& id) {
    return myTypes.find(id) != myTypes.end();
}



/****************************************************************************/

