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
/// @file    GUIDetectorBuilder.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Tue, 22 Jul 2003
/// @version $Id$
///
// Builds detectors for guisim
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
#include <guisim/GUIInductLoop.h>
#include <guisim/GUIE2Collector.h>
#include <guisim/GUIE3Collector.h>
#include <guisim/GUIInstantInductLoop.h>
#include <microsim/MSGlobals.h>
#include <microsim/MSNet.h>
#include <microsim/output/MSInductLoop.h>
#include <utils/common/UtilExceptions.h>
#include <utils/common/FileHelpers.h>
#include "GUIDetectorBuilder.h"

#include <mesogui/GUIMEInductLoop.h>
#include <mesosim/MELoop.h>


// ===========================================================================
// method definitions
// ===========================================================================
GUIDetectorBuilder::GUIDetectorBuilder(MSNet& net)
    : NLDetectorBuilder(net) {}


GUIDetectorBuilder::~GUIDetectorBuilder() {}


MSDetectorFileOutput*
GUIDetectorBuilder::createInductLoop(const std::string& id,
                                     MSLane* lane, double pos, const std::string& vTypes, bool show) {
    if (show) {
        if (MSGlobals::gUseMesoSim) {
            return new GUIMEInductLoop(id, MSGlobals::gMesoNet->getSegmentForEdge(lane->getEdge(), pos), pos, vTypes);
        }
        return new GUIInductLoop(id, lane, pos, vTypes);
    } else {
        return NLDetectorBuilder::createInductLoop(id, lane, pos, vTypes);
    }
}


MSDetectorFileOutput*
GUIDetectorBuilder::createInstantInductLoop(const std::string& id,
        MSLane* lane, double pos, const std::string& od, const std::string& vTypes) {
    return new GUIInstantInductLoop(id, OutputDevice::getDevice(od), lane, pos, vTypes);
}


MSE2Collector*
GUIDetectorBuilder::createE2Detector(const std::string& id,
                                     DetectorUsage usage, MSLane* lane, double pos, double endPos, double length,
                                     SUMOTime haltingTimeThreshold, double haltingSpeedThreshold, double jamDistThreshold,
                                     const std::string& vTypes, bool showDetector) {
    return new GUIE2Collector(id, usage, lane, pos, endPos, length, haltingTimeThreshold, haltingSpeedThreshold, jamDistThreshold, vTypes, showDetector);
}

MSE2Collector*
GUIDetectorBuilder::createE2Detector(const std::string& id,
                                     DetectorUsage usage, std::vector<MSLane*> lanes, double pos, double endPos,
                                     SUMOTime haltingTimeThreshold, double haltingSpeedThreshold, double jamDistThreshold,
                                     const std::string& vTypes, bool showDetector) {
    return new GUIE2Collector(id, usage, lanes, pos, endPos, haltingTimeThreshold, haltingSpeedThreshold, jamDistThreshold, vTypes, showDetector);
}

MSDetectorFileOutput*
GUIDetectorBuilder::createE3Detector(const std::string& id,
                                     const CrossSectionVector& entries,
                                     const CrossSectionVector& exits,
                                     double haltingSpeedThreshold,
                                     SUMOTime haltingTimeThreshold, const std::string& vTypes) {
    return new GUIE3Collector(id, entries, exits, haltingSpeedThreshold, haltingTimeThreshold, vTypes);
}



/****************************************************************************/

