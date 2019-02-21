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
/// @file    RODFDetFlowLoader.cpp
/// @author  Daniel Krajzewicz
/// @author  Eric Nicolay
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Thu, 16.03.2006
/// @version $Id$
///
// A loader for detector flows
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
#include <sstream>
#include <utils/importio/LineReader.h>
#include <utils/options/OptionsCont.h>
#include <utils/common/StringTokenizer.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/FileHelpers.h>
#include <utils/common/TplConvert.h>
#include <utils/common/UtilExceptions.h>
#include "RODFDetFlowLoader.h"


// ===========================================================================
// method definitions
// ===========================================================================
RODFDetFlowLoader::RODFDetFlowLoader(const RODFDetectorCon& dets,
                                     RODFDetectorFlows& into,
                                     SUMOTime startTime, SUMOTime endTime,
                                     SUMOTime timeOffset, SUMOTime timeScale)
    : myStorage(into), myTimeOffset(timeOffset), myTimeScale(timeScale),
      myStartTime(startTime), myEndTime(endTime), myDetectorContainer(dets),
      myHaveWarnedAboutOverridingBoundaries(false), myHaveWarnedAboutPartialDefs(false) {}



RODFDetFlowLoader::~RODFDetFlowLoader() {}


void
RODFDetFlowLoader::read(const std::string& file) {
    LineReader lr(file);
    // parse first line
    myLineHandler.reinit(lr.readLine(), ";", ";", true, true);
    // parse values
    while (lr.hasMore()) {
        std::string line = lr.readLine();
        if (line.find(';') == std::string::npos) {
            continue;
        }
        myLineHandler.parseLine(line);
        try {
            std::string detName = myLineHandler.get("detector");
            if (!myDetectorContainer.knows(detName)) {
                continue;
            }
            const double parsedTime = TplConvert::_2double((myLineHandler.get("time").c_str())) * myTimeScale - myTimeOffset;
            // parsing as float to handle values which would cause int overflow
            if (parsedTime < myStartTime || parsedTime >= myEndTime) {
                if (!myHaveWarnedAboutOverridingBoundaries) {
                    myHaveWarnedAboutOverridingBoundaries = true;
                    WRITE_WARNING("At least one value lies beyond given time boundaries.");
                }
                continue;
            }
            const SUMOTime time = (SUMOTime)(parsedTime + .5);
            FlowDef fd;
            fd.isLKW = 0;
            fd.qPKW = TplConvert::_2double(myLineHandler.get("qpkw").c_str());
            fd.vPKW = 0;
            if (myLineHandler.know("vPKW")) {
                fd.vPKW = TplConvert::_2double(myLineHandler.get("vpkw").c_str());
            }
            fd.qLKW = 0;
            if (myLineHandler.know("qLKW")) {
                fd.qLKW = TplConvert::_2double(myLineHandler.get("qlkw").c_str());
            }
            fd.vLKW = 0;
            if (myLineHandler.know("vLKW")) {
                fd.vLKW = TplConvert::_2double(myLineHandler.get("vlkw").c_str());
            }
            if (fd.qLKW < 0) {
                fd.qLKW = 0;
            }
            if (fd.qPKW < 0) {
                fd.qPKW = 0;
            }
            myStorage.addFlow(detName, time, fd);
            if (!myHaveWarnedAboutPartialDefs && !myLineHandler.hasFullDefinition()) {
                myHaveWarnedAboutPartialDefs = true;
                WRITE_WARNING("At least one line does not contain the correct number of columns.");
            }
            continue;
        } catch (UnknownElement&) {} catch (OutOfBoundsException&) {} catch (NumberFormatException&) {}
        throw ProcessError("The detector-flow-file '" + lr.getFileName() + "' is corrupt;\n"
                           + " The following values must be supplied : 'Detector', 'Time', 'qPKW'\n"
                           + " The according column names must be given in the first line of the file.");
    }
}


/****************************************************************************/

