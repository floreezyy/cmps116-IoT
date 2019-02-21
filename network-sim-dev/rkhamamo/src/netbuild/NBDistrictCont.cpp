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
/// @file    NBDistrictCont.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Tue, 20 Nov 2001
/// @version $Id$
///
// A container for districts
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
#include <iostream>
#include <utils/common/MsgHandler.h>
#include <utils/common/ToString.h>
#include <utils/iodevices/OutputDevice.h>
#include "NBDistrict.h"
#include "NBDistrictCont.h"


// ===========================================================================
// method definitions
// ===========================================================================
NBDistrictCont::NBDistrictCont() {}


NBDistrictCont::~NBDistrictCont() {
    for (DistrictCont::iterator i = myDistricts.begin(); i != myDistricts.end(); i++) {
        delete((*i).second);
    }
    myDistricts.clear();
}


bool
NBDistrictCont::insert(NBDistrict* const district) {
    DistrictCont::const_iterator i = myDistricts.find(district->getID());
    if (i != myDistricts.end()) {
        return false;
    }
    myDistricts.insert(DistrictCont::value_type(district->getID(), district));
    return true;
}


NBDistrict*
NBDistrictCont::retrieve(const std::string& id) const {
    DistrictCont::const_iterator i = myDistricts.find(id);
    if (i == myDistricts.end()) {
        return 0;
    }
    return (*i).second;
}


int
NBDistrictCont::size() const {
    return (int)myDistricts.size();
}


bool
NBDistrictCont::addSource(const std::string& dist, NBEdge* const source,
                          double weight) {
    NBDistrict* o = retrieve(dist);
    if (o == 0) {
        return false;
    }
    return o->addSource(source, weight);
}


bool
NBDistrictCont::addSink(const std::string& dist, NBEdge* const destination,
                        double weight) {
    NBDistrict* o = retrieve(dist);
    if (o == 0) {
        return false;
    }
    return o->addSink(destination, weight);
}


void
NBDistrictCont::removeFromSinksAndSources(NBEdge* const e) {
    for (DistrictCont::iterator i = myDistricts.begin(); i != myDistricts.end(); i++) {
        (*i).second->removeFromSinksAndSources(e);
    }
}



/****************************************************************************/

