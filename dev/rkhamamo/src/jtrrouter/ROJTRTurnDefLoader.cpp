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
/// @file    ROJTRTurnDefLoader.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Tue, 20 Jan 2004
/// @version $Id$
///
// Loader for the of turning percentages and source/sink definitions
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <set>
#include <string>
#include <utils/common/FileHelpers.h>
#include <utils/xml/XMLSubSys.h>
#include <utils/common/UtilExceptions.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/TplConvert.h>
#include <utils/common/ToString.h>
#include <utils/xml/SUMOXMLDefinitions.h>
#include <router/RONet.h>
#include "ROJTREdge.h"
#include "ROJTRTurnDefLoader.h"


// ===========================================================================
// method definitions
// ===========================================================================
ROJTRTurnDefLoader::ROJTRTurnDefLoader(RONet& net)
    : SUMOSAXHandler("turn-ratio-file"), myNet(net),
      myIntervalBegin(0), myIntervalEnd(STEPS2TIME(SUMOTime_MAX)), myEdge(0) {}


ROJTRTurnDefLoader::~ROJTRTurnDefLoader() {}


void
ROJTRTurnDefLoader::myStartElement(int element,
                                   const SUMOSAXAttributes& attrs) {
    bool ok = true;
    switch (element) {
        case SUMO_TAG_INTERVAL:
            myIntervalBegin = attrs.get<double>(SUMO_ATTR_BEGIN, 0, ok);
            myIntervalEnd = attrs.get<double>(SUMO_ATTR_END, 0, ok);
            break;
        case SUMO_TAG_FROMEDGE:
            beginFromEdge(attrs);
            break;
        case SUMO_TAG_TOEDGE:
            addToEdge(attrs);
            break;
        case SUMO_TAG_SINK:
            if (attrs.hasAttribute(SUMO_ATTR_EDGES)) {
                std::string edges = attrs.get<std::string>(SUMO_ATTR_EDGES, 0, ok);
                StringTokenizer st(edges, StringTokenizer::WHITECHARS);
                while (st.hasNext()) {
                    std::string id = st.next();
                    ROEdge* edge = myNet.getEdge(id);
                    if (edge == 0) {
                        throw ProcessError("The edge '" + id + "' declared as a sink is not known.");
                    }
                    edge->setSink();
                }
            }
            break;
        case SUMO_TAG_SOURCE:
            if (attrs.hasAttribute(SUMO_ATTR_EDGES)) {
                std::string edges = attrs.get<std::string>(SUMO_ATTR_EDGES, 0, ok);
                StringTokenizer st(edges, StringTokenizer::WHITECHARS);
                while (st.hasNext()) {
                    std::string id = st.next();
                    ROEdge* edge = myNet.getEdge(id);
                    if (edge == 0) {
                        throw ProcessError("The edge '" + id + "' declared as a source is not known.");
                    }
                    edge->setSource();
                }
            }
            break;
        default:
            break;
    }
}


void
ROJTRTurnDefLoader::beginFromEdge(const SUMOSAXAttributes& attrs) {
    myEdge = 0;
    bool ok = true;
    // get the id, report an error if not given or empty...
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, 0, ok);
    if (!ok) {
        return;
    }
    //
    myEdge = static_cast<ROJTREdge*>(myNet.getEdge(id));
    if (myEdge == 0) {
        WRITE_ERROR("The edge '" + id + "' is not known within the network (within a 'from-edge' tag).");
        return;
    }
}


void
ROJTRTurnDefLoader::addToEdge(const SUMOSAXAttributes& attrs) {
    if (myEdge == 0) {
        return;
    }
    bool ok = true;
    // get the id, report an error if not given or empty...
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, 0, ok);
    if (!ok) {
        return;
    }
    //
    ROJTREdge* edge = static_cast<ROJTREdge*>(myNet.getEdge(id));
    if (edge == 0) {
        WRITE_ERROR("The edge '" + id + "' is not known within the network (within a 'to-edge' tag).");
        return;
    }
    const double probability = attrs.get<double>(SUMO_ATTR_PROB, id.c_str(), ok);
    if (ok) {
        if (probability < 0) {
            WRITE_ERROR("'probability' must be positive (in definition of to-edge '" + id + "').");
        } else {
            myEdge->addFollowerProbability(edge, myIntervalBegin, myIntervalEnd, probability);
        }
    }
}



/****************************************************************************/

