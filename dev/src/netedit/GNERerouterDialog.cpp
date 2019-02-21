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
/// @file    GNERerouterDialog.cpp
/// @author  Pablo Alvarez Lopez
/// @date    April 2016
/// @version $Id$
///
// Dialog for edit rerouters
/****************************************************************************/

// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <iostream>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/images/GUIIconSubSys.h>
#include <utils/gui/div/GUIDesigns.h>

#include "GNERerouterDialog.h"
#include "GNERerouter.h"
#include "GNERerouterInterval.h"
#include "GNERerouterIntervalDialog.h"
#include "GNENet.h"
#include "GNEViewNet.h"
#include "GNEUndoList.h"
#include "GNEChange_RerouterItem.h"


// ===========================================================================
// FOX callback mapping
// ===========================================================================

FXDEFMAP(GNERerouterDialog) GNERerouterDialogMap[] = {
    FXMAPFUNC(SEL_COMMAND,          MID_GNE_REROUTEDIALOG_ADD_INTERVAL,     GNERerouterDialog::onCmdAddInterval),
    FXMAPFUNC(SEL_CLICKED,          MID_GNE_REROUTEDIALOG_TABLE_INTERVAL,   GNERerouterDialog::onCmdClickedInterval),
    FXMAPFUNC(SEL_DOUBLECLICKED,    MID_GNE_REROUTEDIALOG_TABLE_INTERVAL,   GNERerouterDialog::onCmdClickedInterval),
    FXMAPFUNC(SEL_TRIPLECLICKED,    MID_GNE_REROUTEDIALOG_TABLE_INTERVAL,   GNERerouterDialog::onCmdClickedInterval),
};

// Object implementation
FXIMPLEMENT(GNERerouterDialog, GNEAdditionalDialog, GNERerouterDialogMap, ARRAYNUMBER(GNERerouterDialogMap))

// ===========================================================================
// member method definitions
// ===========================================================================

GNERerouterDialog::GNERerouterDialog(GNERerouter* rerouterParent) :
    GNEAdditionalDialog(rerouterParent, 320, 240),
    myEditedRerouter(rerouterParent) {

    // create add buton and label
    FXHorizontalFrame* buttonAndLabelInterval = new FXHorizontalFrame(myContentFrame, GUIDesignAuxiliarHorizontalFrame);
    myAddInterval = new FXButton(buttonAndLabelInterval, "", GUIIconSubSys::getIcon(ICON_ADD), this, MID_GNE_REROUTEDIALOG_ADD_INTERVAL, GUIDesignButtonIcon);
    new FXLabel(buttonAndLabelInterval, ("Add new " + toString(SUMO_TAG_INTERVAL)).c_str(), 0, GUIDesignLabelThick);

    // Create table
    myIntervalTable = new FXTable(myContentFrame, this, MID_GNE_REROUTEDIALOG_TABLE_INTERVAL, GUIDesignTableAdditionals);
    myIntervalTable->setSelBackColor(FXRGBA(255, 255, 255, 255));
    myIntervalTable->setSelTextColor(FXRGBA(0, 0, 0, 255));
    myIntervalTable->setEditable(false);

    // update intervals
    updateIntervalTable();

    // start a undo list for editing local to this additional
    initChanges();

    // Open dialog as modal
    openAsModalDialog();
}


GNERerouterDialog::~GNERerouterDialog() {}


GNERerouter*
GNERerouterDialog::getEditedRerouter() const {
    return myEditedRerouter;
}


long
GNERerouterDialog::onCmdAccept(FXObject*, FXSelector, void*) {
    // get number of Rerouter intervals overlapped
    int numberOfOverlappings = myEditedRerouter->getNumberOfOverlappedIntervals();
    // overlapped intervals aren't allowed
    if (numberOfOverlappings > 0) {
        // write warning if netedit is running in testing mode
        if (OptionsCont::getOptions().getBool("gui-testing-debug")) {
            WRITE_WARNING("Opening FXMessageBox of type 'warning'");
        }
        // open warning Box
        std::string errorMessage = numberOfOverlappings > 1 ? ("There are " + toString(numberOfOverlappings) + " intervals overlapped.") : ("There is " + toString(numberOfOverlappings) + " interval overlapped.");
        FXMessageBox::warning(getApp(), MBOX_OK, "Overlapping detected", "%s", ("Values of '" + myEditedRerouter->getID() + "' cannot be saved. " + errorMessage).c_str());
        // write warning if netedit is running in testing mode
        if (OptionsCont::getOptions().getBool("gui-testing-debug")) {
            WRITE_WARNING("Closed FXMessageBox of type 'warning' with 'OK'");
        }
        return 0;
    } else {
        // accept changes before closing dialog
        acceptChanges();
        // Stop Modal
        getApp()->stopModal(this, TRUE);
        return 1;
    }
}


long
GNERerouterDialog::onCmdCancel(FXObject*, FXSelector, void*) {
    // cancel changes
    cancelChanges();
    // Stop Modal
    getApp()->stopModal(this, FALSE);
    return 1;
}


long
GNERerouterDialog::onCmdReset(FXObject*, FXSelector, void*) {
    // reset changes
    resetChanges();
    // update interval table
    updateIntervalTable();
    return 1;
}


long
GNERerouterDialog::onCmdAddInterval(FXObject*, FXSelector, void*) {
    // create empty rerouter interval and configure it with GNERerouterIntervalDialog
    GNERerouterIntervalDialog(new GNERerouterInterval(this), false);
    // update interval table
    updateIntervalTable();
    return 1;
}


long
GNERerouterDialog::onCmdClickedInterval(FXObject*, FXSelector, void*) {
    // check if some delete button was pressed
    for (int i = 0; i < (int)myEditedRerouter->getRerouterIntervals().size(); i++) {
        if (myIntervalTable->getItem(i, 2)->hasFocus()) {
            // get rerouter interval to remove
            GNERerouterInterval* rerouterInterval = myEditedRerouter->getRerouterIntervals().at(i);
            // drop all closing reroutes of interval
            while (rerouterInterval->getClosingReroutes().size() > 0) {
                myEditedRerouter->getViewNet()->getUndoList()->add(new GNEChange_RerouterItem(rerouterInterval->getClosingReroutes().front(), false), true);
            }
            // drop all closing lane reroutes of interval
            while (rerouterInterval->getClosingLaneReroutes().size() > 0) {
                myEditedRerouter->getViewNet()->getUndoList()->add(new GNEChange_RerouterItem(rerouterInterval->getClosingLaneReroutes().front(), false), true);
            }
            // drop all route probability reroutes of interval
            while (rerouterInterval->getRouteProbReroutes().size() > 0) {
                myEditedRerouter->getViewNet()->getUndoList()->add(new GNEChange_RerouterItem(rerouterInterval->getRouteProbReroutes().front(), false), true);
            }
            // drop all destiny probability reroutes of interval
            while (rerouterInterval->getDestProbReroutes().size() > 0) {
                myEditedRerouter->getViewNet()->getUndoList()->add(new GNEChange_RerouterItem(rerouterInterval->getDestProbReroutes().front(), false), true);
            }
            // remove interval
            myEditedRerouter->getViewNet()->getUndoList()->add(new GNEChange_RerouterItem(rerouterInterval, false), true);
            // update interval table after removing
            updateIntervalTable();
            return 1;
        }
    }
    // check if some begin or o end  button was pressed
    for (int i = 0; i < (int)myEditedRerouter->getRerouterIntervals().size(); i++) {
        if (myIntervalTable->getItem(i, 0)->hasFocus() || myIntervalTable->getItem(i, 1)->hasFocus()) {
            // edit interval
            GNERerouterIntervalDialog(myEditedRerouter->getRerouterIntervals().at(i), true);
            // update interval table after editing
            updateIntervalTable();
            return 1;
        }
    }
    // nothing to do
    return 0;
}


void
GNERerouterDialog::updateIntervalTable() {
    // clear table
    myIntervalTable->clearItems();
    // set number of rows
    myIntervalTable->setTableSize(int(myEditedRerouter->getRerouterIntervals().size()), 3);
    // Configure list
    myIntervalTable->setVisibleColumns(4);
    myIntervalTable->setColumnWidth(0, 137);
    myIntervalTable->setColumnWidth(1, 136);
    myIntervalTable->setColumnWidth(2, GUIDesignTableIconCellWidth);
    myIntervalTable->setColumnText(0, toString(SUMO_ATTR_BEGIN).c_str());
    myIntervalTable->setColumnText(1, toString(SUMO_ATTR_END).c_str());
    myIntervalTable->setColumnText(2, "");
    myIntervalTable->getRowHeader()->setWidth(0);
    // Declare index for rows and pointer to FXTableItem
    int indexRow = 0;
    FXTableItem* item = 0;
    // iterate over values
    for (auto i : myEditedRerouter->getRerouterIntervals()) {
        // Set time
        item = new FXTableItem(i->getAttribute(SUMO_ATTR_BEGIN).c_str());
        myIntervalTable->setItem(indexRow, 0, item);
        // Set speed
        item = new FXTableItem(i->getAttribute(SUMO_ATTR_END).c_str());
        myIntervalTable->setItem(indexRow, 1, item);
        // set remove
        item = new FXTableItem("", GUIIconSubSys::getIcon(ICON_REMOVE));
        item->setJustify(FXTableItem::CENTER_X | FXTableItem::CENTER_Y);
        item->setEnabled(false);
        myIntervalTable->setItem(indexRow, 2, item);
        // Update index
        indexRow++;
    }
}

/****************************************************************************/
