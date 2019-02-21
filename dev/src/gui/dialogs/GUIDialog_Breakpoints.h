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
/// @file    GUIDialog_Breakpoints.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @date    Thu, 17 Jun 2004
/// @version $Id$
///
// Editor for simulation breakpoints
/****************************************************************************/
#ifndef GUIDialog_Breakpoints_h
#define GUIDialog_Breakpoints_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <vector>
#include <fx.h>


// ===========================================================================
// class definition
// ===========================================================================
/**
 * @class GUIDialog_Breakpoints
 * @brief Editor for simulation breakpoints
 *
 * This dialog shows and lets the user edit the list of breakpoints - simulation
 *  time steps where the simulation halts.
 * @todo Use a LineReader instead of >> while reading
 */
class GUIDialog_Breakpoints : public FXMainWindow {
    // FOX-declarations
    FXDECLARE(GUIDialog_Breakpoints)

public:
    /** @brief Constructor
     * @param[in] parent The parent window
     */
    GUIDialog_Breakpoints(GUIMainWindow* parent, std::vector<SUMOTime>& breakpoints, FXMutex& breakpointLock);


    /// @brief Destructor
    ~GUIDialog_Breakpoints();

    /// @brief sets the focus after the window is created
    void show();



    /// @name FOX-callbacks
    /// @{

    /// @brief Called when the user presses the Load-button
    long onCmdLoad(FXObject*, FXSelector, void*);

    /// @brief Called when the user presses the Save-button
    long onCmdSave(FXObject*, FXSelector, void*);

    /// @brief Called when the user presses the Clear-button
    long onCmdClear(FXObject*, FXSelector, void*);

    /// @brief Called when the user presses the Close-button
    long onCmdClose(FXObject*, FXSelector, void*);

    /// @brief Called when the table was changed
    long onCmdEditTable(FXObject*, FXSelector, void*);
    /// @}


private:
    /// @brief Rebuilds the entire list
    void rebuildList();

    /** @brief Builds a text representation of the items in the list
     * @return Breakpoints encoded as a string
     */
    std::string encode2TXT();


private:
    /// @brief The list that holds the ids
    FXTable* myTable;

    /// @brief The parent window
    GUIMainWindow* myParent;

    /// @brief List of breakpoints
    std::vector<SUMOTime>* myBreakpoints;

    /// @brief Lock for modifying the list of breakpoints
    FXMutex* myBreakpointLock;


protected:
    /// FOX needs this
    GUIDialog_Breakpoints() { }

};


#endif

/****************************************************************************/

