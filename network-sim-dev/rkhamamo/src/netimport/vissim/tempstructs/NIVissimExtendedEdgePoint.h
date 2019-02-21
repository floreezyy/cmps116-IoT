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
/// @file    NIVissimExtendedEdgePoint.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id$
///
// -------------------
/****************************************************************************/
#ifndef NIVissimExtendedEdgePoint_h
#define NIVissimExtendedEdgePoint_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <vector>


// ===========================================================================
// class declarations
// ===========================================================================
class NBEdge;
class Position;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 *
 */
class NIVissimExtendedEdgePoint {
public:
    /** @brief Constructor
     * @param[in] edgeid The id of the Vissim-edge
     * @param[in] lanes Lanes on which this point lies
     * @param[in] position The position of this point at the edge
     * @param[in] assignedVehicles Vehicle (type) indices which should be regarded by this point
     */
    NIVissimExtendedEdgePoint(int edgeid, const std::vector<int>& lanes,
                              double position, const std::vector<int>& assignedVehicles);
    ~NIVissimExtendedEdgePoint();
    int getEdgeID() const;
    double getPosition() const;
    Position getGeomPosition() const;
    const std::vector<int>& getLanes() const;


    /** @brief Resets lane numbers if all lanes shall be used
     *
     * If myLanes contains a -1, the content of myLanes is replaced
     *  by indices of all lanes of the given edge.
     *
     * @param[in] The built edge
     */
    void recheckLanes(const NBEdge* const edge);

private:
    int myEdgeID;
    std::vector<int> myLanes;
    double myPosition;
    std::vector<int> myAssignedVehicles;
};


#endif

/****************************************************************************/

