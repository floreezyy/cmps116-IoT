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
/// @file    MEInductLoop.cpp
/// @author  Daniel Krajzewicz
/// @date    Tue, May 2005
/// @version $Id$
///
// An induction loop for mesoscopic simulation
/****************************************************************************/


/* =========================================================================
 * included modules
 * ======================================================================= */
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "MEInductLoop.h"
#include <cassert>
#include <numeric>
#include <utility>
#include <limits>
#include <utils/common/WrappingCommand.h>
#include <utils/common/ToString.h>
#include <microsim/MSEdge.h>
#include <microsim/MSEventControl.h>
#include <mesosim/MESegment.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/UtilExceptions.h>
#include <utils/common/StringUtils.h>


// ===========================================================================
// method definitions
// ===========================================================================
MEInductLoop::MEInductLoop(const std::string& id,
                           MESegment* s,
                           double positionInMeters,
                           const std::string& vTypes) :
    MSDetectorFileOutput(id, vTypes), mySegment(s),
    myPosition(positionInMeters),
    myMeanData(0, mySegment->getLength(), false, 0) {
    myMeanData.setDescription("inductionLoop_" + id);
    s->addDetector(&myMeanData);
}


MEInductLoop::~MEInductLoop() {}


void
MEInductLoop::writeXMLOutput(OutputDevice& dev,
                             SUMOTime startTime, SUMOTime stopTime) {
    mySegment->prepareDetectorForWriting(myMeanData);
    dev.openTag(SUMO_TAG_INTERVAL).writeAttr(SUMO_ATTR_BEGIN, time2string(startTime)).writeAttr(SUMO_ATTR_END, time2string(stopTime));
    dev.writeAttr(SUMO_ATTR_ID, StringUtils::escapeXML(myID)).writeAttr("sampledSeconds", myMeanData.getSamples());
    myMeanData.write(dev, stopTime - startTime, (double)mySegment->getEdge().getLanes().size(), -1.0);
    myMeanData.reset();
}


/****************************************************************************/

