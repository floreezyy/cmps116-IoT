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
/// @file    GUITriggerBuilder.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Mon, 26.04.2004
/// @version $Id$
///
// Builds trigger objects for guisim
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
#include <fstream>
#include <guisim/GUILaneSpeedTrigger.h>
#include <guisim/GUINet.h>
#include <guisim/GUITriggeredRerouter.h>
#include <guisim/GUIBusStop.h>
#include <guisim/GUIContainerStop.h>
#include <guisim/GUIParkingArea.h>
#include <guisim/GUICalibrator.h>
#include <guisim/GUIChargingStation.h>
#include "GUITriggerBuilder.h"



// ===========================================================================
// method definitions
// ===========================================================================
GUITriggerBuilder::GUITriggerBuilder() {}


GUITriggerBuilder::~GUITriggerBuilder() {}


MSLaneSpeedTrigger*
GUITriggerBuilder::buildLaneSpeedTrigger(MSNet& net,
        const std::string& id, const std::vector<MSLane*>& destLanes,
        const std::string& file) {
    GUILaneSpeedTrigger* lst = new GUILaneSpeedTrigger(id, destLanes, file);
    static_cast<GUINet&>(net).getVisualisationSpeedUp().addAdditionalGLObject(lst);
    return lst;
}


MSTriggeredRerouter*
GUITriggerBuilder::buildRerouter(MSNet& net, const std::string& id,
                                 MSEdgeVector& edges,
                                 double prob, const std::string& file, bool off,
                                 SUMOTime timeThreshold) {
    GUITriggeredRerouter* rr = new GUITriggeredRerouter(id, edges, prob, file, off, timeThreshold,
            dynamic_cast<GUINet&>(net).getVisualisationSpeedUp());
    return rr;
}


void
GUITriggerBuilder::buildStoppingPlace(MSNet& net, std::string id, std::vector<std::string> lines, MSLane* lane,
                                      double frompos, double topos, const SumoXMLTag element, std::string name) {
    if (element == SUMO_TAG_CONTAINER_STOP) {
        //TODO: shall we also allow names for container stops? might make sense [GL March '17]
        myCurrentStop = new GUIContainerStop(id, lines, *lane, frompos, topos);
    } else {
        myCurrentStop = new GUIBusStop(id, lines, *lane, frompos, topos, name);
    }
    if (!net.addStoppingPlace(element, myCurrentStop)) {
        delete myCurrentStop;
        myCurrentStop = 0;
        throw InvalidArgument("Could not build " + toString(element) + " '" + id + "'; probably declared twice.");
    }
    static_cast<GUINet&>(net).getVisualisationSpeedUp().addAdditionalGLObject(dynamic_cast<GUIGlObject*>(myCurrentStop));
}


void
GUITriggerBuilder::beginParkingArea(MSNet& net, const std::string& id,
                                    const std::vector<std::string>& lines,
                                    MSLane* lane,
                                    double frompos, double topos,
                                    unsigned int capacity,
                                    double width, double length, double angle) {
    assert(myParkingArea == 0);
    GUIParkingArea* stop = new GUIParkingArea(id, lines, *lane, frompos, topos, capacity, width, length, angle);
    if (!net.addStoppingPlace(SUMO_TAG_PARKING_AREA, stop)) {
        delete stop;
        throw InvalidArgument("Could not build parking area '" + id + "'; probably declared twice.");
    } else {
        myParkingArea = stop;
    }
}

void
GUITriggerBuilder::buildChargingStation(MSNet& net, const std::string& id, MSLane* lane, double frompos, double topos,
                                        double chargingPower, double efficiency, bool chargeInTransit, int chargeDelay) {
    GUIChargingStation* chargingStation = new GUIChargingStation(id, *lane, frompos, topos, chargingPower, efficiency, chargeInTransit, chargeDelay);
    if (!net.addStoppingPlace(SUMO_TAG_CHARGING_STATION, chargingStation)) {
        delete chargingStation;
        throw InvalidArgument("Could not build charging station '" + id + "'; probably declared twice.");
    }
    static_cast<GUINet&>(net).getVisualisationSpeedUp().addAdditionalGLObject(chargingStation);
}

MSCalibrator*
GUITriggerBuilder::buildCalibrator(MSNet& net, const std::string& id,
                                   MSEdge* edge, MSLane* lane, double pos,
                                   const std::string& file,
                                   const std::string& outfile,
                                   const SUMOTime freq,
                                   const MSRouteProbe* probe) {
    GUICalibrator* cali = new GUICalibrator(id, edge, lane, pos, file, outfile, freq, probe);
    static_cast<GUINet&>(net).getVisualisationSpeedUp().addAdditionalGLObject(cali);
    return cali;
}


void
GUITriggerBuilder::endParkingArea() {
    if (myParkingArea != 0) {
        static_cast<GUINet*>(MSNet::getInstance())->getVisualisationSpeedUp().addAdditionalGLObject(static_cast<GUIParkingArea*>(myParkingArea));
        myParkingArea = 0;
    } else {
        throw InvalidArgument("Could not end a parking area that is not opened.");
    }
}



/****************************************************************************/

