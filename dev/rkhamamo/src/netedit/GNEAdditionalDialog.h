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
/// @file    GNEAdditionalDialog.h
/// @author  Pablo Alvarez Lopez
/// @date    April 2016
/// @version $Id$
///
// A abstract class for editing additional elements
/****************************************************************************/
#ifndef GNEAdditionalDialog_h
#define GNEAdditionalDialog_h

// ===========================================================================
// included modules
// ===========================================================================

#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <fx.h>
#include <vector>
#include <utils/xml/SUMOXMLDefinitions.h>

// ===========================================================================
// class declarations
// ===========================================================================

class GNEAdditional;
class GNEUndoList;

// ===========================================================================
// class definitions
// ===========================================================================

/**
 * @class GNEAdditionalDialog
 * @brief Dialog to edit sequences, parameters, etc.. of Additionals
 */
class GNEAdditionalDialog : protected FXTopWindow {
    /// @brief FOX-declaration abstract
    FXDECLARE_ABSTRACT(GNEAdditionalDialog)

public:
    /// @brief constructor
    GNEAdditionalDialog(GNEAdditional* parent, int width, int height);

    /// @brief destructor
    ~GNEAdditionalDialog();

    /// @name FOX-callbacks
    /// @{
    /// @brief event after press accept button
    virtual long onCmdAccept(FXObject* sender, FXSelector sel, void* ptr) = 0;

    /// @brief event after press cancel button
    virtual long onCmdCancel(FXObject* sender, FXSelector sel, void* ptr) = 0;

    /// @brief event after press cancel button
    virtual long onCmdReset(FXObject*, FXSelector, void*) = 0;

    /// @brief event after press a key
    long onKeyPress(FXObject* sender, FXSelector sel, void* ptr);

    /// @brief event after release a key
    long onKeyRelease(FXObject* sender, FXSelector sel, void* ptr);

    /// @}

protected:
    /// @brief FOX needs this
    GNEAdditionalDialog() {}

    /// @brief frame for contents
    FXVerticalFrame* myContentFrame;

    /// @brief execute dialog as modal
    FXint openAsModalDialog(FXuint placement = PLACEMENT_CURSOR);

    /// @brief change additional dialog header
    void changeAdditionalDialogHeader(const std::string& newHeader);

    /// @brief init a new group of changes that will be do it in dialog
    void initChanges();

    /// @brief Accept changes did in this dialog.
    void acceptChanges();

    /// @brief Cancel changes did in this dialog.
    void cancelChanges();

    /// @brief reset changes did in this dialog.
    void resetChanges();

private:
    /// @brief accept button
    FXButton* myAcceptButton;

    /// @brief cancel button
    FXButton* myCancelButton;

    /// @brief cancel button
    FXButton* myResetButton;

    /// @brief description of changes did in this additional dialog
    std::string myChangesDescription;

    /// @brief number of GNEChanges_... in dialog
    int myNumberOfChanges;

    /// @brief pointer to UndoList
    GNEUndoList* myUndoList;

    /// @brief Invalidated copy constructor
    GNEAdditionalDialog(const GNEAdditionalDialog&) = delete;

    /// @brief Invalidated assignment operator
    GNEAdditionalDialog& operator=(const GNEAdditionalDialog&) = delete;
};

#endif
