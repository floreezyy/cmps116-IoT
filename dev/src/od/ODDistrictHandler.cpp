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
/// @file    ODDistrictHandler.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id$
///
// An XML-Handler for districts
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
#include <utility>
#include <iostream>
#include <utils/common/UtilExceptions.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/ToString.h>
#include <utils/xml/SUMOSAXHandler.h>
#include <utils/xml/SUMOXMLDefinitions.h>
#include "ODDistrict.h"
#include "ODDistrictCont.h"
#include "ODDistrictHandler.h"


// ===========================================================================
// method definitions
// ===========================================================================
ODDistrictHandler::ODDistrictHandler(ODDistrictCont& cont,
                                     const std::string& file)
    : SUMOSAXHandler(file), myContainer(cont), myCurrentDistrict(0) {}


ODDistrictHandler::~ODDistrictHandler() {}


void
ODDistrictHandler::myStartElement(int element,
                                  const SUMOSAXAttributes& attrs) {
    switch (element) {
        case SUMO_TAG_TAZ:
            openDistrict(attrs);
            break;
        case SUMO_TAG_TAZSOURCE:
            addSource(attrs);
            break;
        case SUMO_TAG_TAZSINK:
            addSink(attrs);
            break;
        default:
            break;
    }
}


void
ODDistrictHandler::myEndElement(int element) {
    if (element == SUMO_TAG_TAZ) {
        closeDistrict();
    }
}


void
ODDistrictHandler::openDistrict(const SUMOSAXAttributes& attrs) {
    myCurrentDistrict = 0;
    // get the id, report an error if not given or empty...
    bool ok = true;
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, 0, ok);
    if (!ok) {
        return;
    }
    myCurrentDistrict = new ODDistrict(id);
    if (attrs.hasAttribute(SUMO_ATTR_EDGES)) {
        std::vector<std::string> desc = attrs.getStringVector(SUMO_ATTR_EDGES);
        for (std::vector<std::string>::const_iterator i = desc.begin(); i != desc.end(); ++i) {
            myCurrentDistrict->addSource(*i, 1.);
            myCurrentDistrict->addSink(*i, 1.);
        }
    }
}


void
ODDistrictHandler::addSource(const SUMOSAXAttributes& attrs) {
    std::pair<std::string, double> vals = parseTAZ(attrs);
    if (vals.second >= 0) {
        myCurrentDistrict->addSource(vals.first, vals.second);
    }
}


void
ODDistrictHandler::addSink(const SUMOSAXAttributes& attrs) {
    std::pair<std::string, double> vals = parseTAZ(attrs);
    if (vals.second >= 0) {
        myCurrentDistrict->addSink(vals.first, vals.second);
    }
}



std::pair<std::string, double>
ODDistrictHandler::parseTAZ(const SUMOSAXAttributes& attrs) {
    // check the current district first
    if (myCurrentDistrict == 0) {
        return std::pair<std::string, double>("", -1);
    }
    // get the id, report an error if not given or empty...
    bool ok = true;
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, 0, ok);
    if (!ok) {
        return std::pair<std::string, double>("", -1);
    }
    // get the weight
    double weight = attrs.get<double>(SUMO_ATTR_WEIGHT, id.c_str(), ok);
    if (ok) {
        if (weight < 0) {
            WRITE_ERROR("'probability' must be positive (in definition of " + attrs.getObjectType() + " '" + id + "').");
        } else {
            return std::pair<std::string, double>(id, weight);
        }
    }
    return std::pair<std::string, double>("", -1);
}


void
ODDistrictHandler::closeDistrict() {
    if (myCurrentDistrict != 0) {
        myContainer.add(myCurrentDistrict->getID(), myCurrentDistrict);
    }
}



/****************************************************************************/

