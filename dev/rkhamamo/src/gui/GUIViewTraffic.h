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
/// @file    GUIViewTraffic.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @author  Andreas Gaubatz
/// @date    Sept 2002
/// @version $Id$
///
// A view on the simulation; this view is a microscopic one
/****************************************************************************/
#ifndef GUIViewTraffic_h
#define GUIViewTraffic_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <utils/geom/Boundary.h>
#include <utils/geom/Position.h>
#include <utils/common/RGBColor.h>
#include <utils/geom/PositionVector.h>
#include "GUISUMOViewParent.h"
#include <utils/gui/windows/GUISUMOAbstractView.h>


// ===========================================================================
// class declarations
// ===========================================================================
class GUINet;
class GUISUMOViewParent;
class GUIVehicle;
class GUIVideoEncoder;
class MSRoute;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GUIViewTraffic
 * Microsocopic view at the simulation
 */
class GUIViewTraffic : public GUISUMOAbstractView {
    FXDECLARE(GUIViewTraffic)
public:
    /// constructor
    GUIViewTraffic(FXComposite* p, GUIMainWindow& app,
                   GUISUMOViewParent* parent, GUINet& net, FXGLVisual* glVis,
                   FXGLCanvas* share);
    /// destructor
    virtual ~GUIViewTraffic();

    /// builds the view toolbars
    virtual void buildViewToolBars(GUIGlChildWindow&);


    /** @brief Starts vehicle tracking
     * @param[in] id The glID of the vehicle to track
     */
    void startTrack(int id);


    /** @brief Stops vehicle tracking
     */
    void stopTrack();


    /** @brief Returns the id of the tracked vehicle (-1 if none)
     * @return The glID of the vehicle to track
     */
    GUIGlID getTrackedID() const;

    bool setColorScheme(const std::string& name);

    ///@brief recalibrate color scheme according to the current value range
    void buildColorRainbow(GUIColorScheme& scheme, int active, GUIGlObjectType objectType);

    /// @brief handle mouse click in gaming mode
    void onGamingClick(Position pos);

    /// @brief get the current simulation time
    SUMOTime getCurrentTimeStep() const;

    /// @brief interaction with the simulation
    long onCmdCloseLane(FXObject*, FXSelector, void*);
    long onCmdCloseEdge(FXObject*, FXSelector, void*);
    long onCmdAddRerouter(FXObject*, FXSelector, void*);

    /** @brief Adds a frame to a video snapshot which will be initialized if neccessary
     */
    void saveFrame(const std::string& destFile, FXColor* buf);

    /** @brief Ends a video snapshot
     */
    void endSnapshot();

    /** @brief Checks whether it is time for a snapshot
     */
    void checkSnapshots();

protected:
    int doPaintGL(int mode, const Boundary& bound);

    GUILane* getLaneUnderCursor();

private:
    GUIGlID myTrackedID;

#ifdef HAVE_FFMPEG
    GUIVideoEncoder* myCurrentVideo;
#endif

protected:
    GUIViewTraffic() { }

};


#endif

/****************************************************************************/

