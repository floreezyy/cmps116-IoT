/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2002-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    GUIEvent_SimulationLoaded.h
/// @author  Daniel Krajzewicz
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    Sept 2002
/// @version $Id$
///
// Event send when the simulation has been loaded by GUILadThread
/****************************************************************************/
#ifndef GUIEvent_SimulationLoaded_h
#define GUIEvent_SimulationLoaded_h


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
#include <utils/gui/events/GUIEvent.h>
#include <utils/common/SUMOTime.h>


// ===========================================================================
// class declarations
// ===========================================================================
class GUINet;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class  GUIEvent_SimulationLoaded
 *
 * Throw to GUIApplicationWindow from GUILoadThread after a simulation has
 * been loaded or the loading process failed
 */
class GUIEvent_SimulationLoaded : public GUIEvent {
public:
    /// constructor
    GUIEvent_SimulationLoaded(GUINet* net,
                              SUMOTime startTime, SUMOTime endTime,
                              const std::string& file,
                              const std::vector<std::string>& settingsFiles,
                              const bool osgView)
        : GUIEvent(EVENT_SIMULATION_LOADED),
          myNet(net), myBegin(startTime), myEnd(endTime),
          myFile(file), mySettingsFiles(settingsFiles),
          myOsgView(osgView) {
    }

    /// destructor
    ~GUIEvent_SimulationLoaded() { }

public:
    /// the loaded net
    GUINet*  myNet;

    /// the time the simulation shall start with
    const SUMOTime myBegin;

    /// the time the simulation shall end with
    const SUMOTime myEnd;

    /// the name of the loaded file
    const std::string myFile;

    /// the name of the settings file to load
    const std::vector<std::string> mySettingsFiles;

    /// whether to load the OpenSceneGraph view
    const bool myOsgView;

private:
    /// @brief Invalidated assignment operator
    GUIEvent_SimulationLoaded& operator=(const GUIEvent_SimulationLoaded& s);
};


#endif

/****************************************************************************/

