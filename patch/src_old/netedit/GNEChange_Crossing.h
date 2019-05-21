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
/// @file    GNEChange_Crossing.h
/// @author  Pablo Alvarez Lopez
/// @date    Oct 2016
/// @version $Id$
///
// A network change in which a single crossing is created or deleted
/****************************************************************************/
#ifndef GNEChange_Crossing_h
#define GNEChange_Crossing_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <fx.h>
#include <utils/foxtools/fxexdefs.h>
#include <utils/geom/PositionVector.h>
#include <vector>
#include <map>
#include "GNEChange.h"

// ===========================================================================
// class declarations
// ===========================================================================
class GNENet;
class GNEJunction;
class NBEdge;

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GNEChange_Crossing
 * A network change in which a single crossing is created or deleted
 */
class GNEChange_Crossing : public GNEChange {
    // @brief FOX Declaration
    FXDECLARE_ABSTRACT(GNEChange_Crossing)

public:
    /**@brief Constructor for creating/deleting an crossing
     * @param[in] JunctionParent GNEJunction in which the crossing will be created/deleted
     * @param[in] edges vector of edges of crossing
     * @param[in] width value with the width of crossing
     * @param[in] priority boolean with the priority of crossing
     * @param[in] check if in the moment of change connection was selected
     * @param[in] forward Whether to create/delete (true/false)
     */
    GNEChange_Crossing(GNEJunction* junctionParent, const std::vector<NBEdge*>& edges, double width,
                       bool priority, int customTLIndex, const PositionVector& customShape, bool selected, bool forward);

    /// @brief Destructor
    ~GNEChange_Crossing();

    /// @name inherited from GNEChange
    /// @{
    /// @brief get undo Name
    FXString undoName() const;

    /// @brief get Redo name
    FXString redoName() const;

    /// @brief undo action
    void undo();

    /// @brief redo action
    void redo();
    /// @}

private:
    /// @brief full information regarding the Junction in which GNECRossing is created
    GNEJunction* myJunctionParent;

    /// @brief vector to save all edges of GNECrossing
    std::vector<NBEdge*> myEdges;

    /// @brief width of GNECrossing
    double myWidth;

    /// @brief priority of GNECrossing
    bool myPriority;

    /// @brief custom index of GNECrossing
    int myCustomTLIndex;

    /// @brief priority of GNECrossing
    PositionVector myCustomShape;

    /// @brief flag to indicates if crossing was previously selected
    bool mySelected;
};

#endif
/****************************************************************************/
