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
/// @file    NWWriter_OpenDrive.h
/// @author  Daniel Krajzewicz
/// @date    Tue, 04.05.2011
/// @version $Id$
///
// Exporter writing networks using the openDRIVE format
/****************************************************************************/
#ifndef NWWriter_OpenDrive_h
#define NWWriter_OpenDrive_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <utils/common/StringBijection.h>
#include <utils/common/SUMOVehicleClass.h>
#include <netbuild/NBEdge.h>


// ===========================================================================
// class declarations
// ===========================================================================
class NBNetBuilder;
class NBEdge;
class OptionsCont;
class PositionVector;
class OutputDevice;
class OutputDevice_String;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class NWWriter_OpenDrive
 * @brief Exporter writing networks using the openDRIVE format
 *
 */
class NWWriter_OpenDrive {
public:
    /** @brief Writes the network into a openDRIVE-file
     *
     * @param[in] oc The options to use
     * @param[in] nb The network builder to fill
     */
    static void writeNetwork(const OptionsCont& oc, NBNetBuilder& nb);

protected:
    /// @brief write normal edge to device
    static void writeNormalEdge(OutputDevice& device, const NBEdge* e,
                                int edgeID, int fromNodeID, int toNodeID,
                                const bool origNames,
                                const double straightThresh);

    /// @brief write internal edge to device, return next connectionID
    static int writeInternalEdge(OutputDevice& device, OutputDevice& junctionDevice,
                                 const NBEdge* inEdge, int nodeID,
                                 int edgeID, int inEdgeID, int outEdgeID,
                                 int connectionID,
                                 const std::vector<NBEdge::Connection>& parallel,
                                 const bool isOuterEdge,
                                 const double straightThresh,
                                 const std::string& centerMark);

    static void addPedestrianConnection(const NBEdge* inEdge, const NBEdge* outEdge, std::vector<NBEdge::Connection>& parallel);

    /// @brief write geometry as sequence of lines (sumo style)
    static double writeGeomLines(const PositionVector& shape, OutputDevice& device, OutputDevice& elevationDevice, double offset = 0);

    /* @brief write geometry as sequence of lines and bezier curves
     *
     * @param[in] straightThresh angular changes below threshold are considered to be straight and no curve will be fitted between the segments
     * @param[out] length Return the total length of the reference line
     */
    static bool writeGeomSmooth(const PositionVector& shape, double speed, OutputDevice& device, OutputDevice& elevationDevice, double straightThresh, double& length);

    /// @brief write geometry as a single bezier curve (paramPoly3)
    static double writeGeomPP3(OutputDevice& device,
                               OutputDevice& elevationDevice,
                               PositionVector init,
                               double length,
                               double offset = 0);

    static void writeElevationProfile(const PositionVector& shape, OutputDevice& device, const OutputDevice_String& elevationDevice);

    static void writeEmptyCenterLane(OutputDevice& device, const std::string& mark, double markWidth);
    static int getID(const std::string& origID, StringBijection<int>& map, int& lastID);

    static std::string getLaneType(SVCPermissions permissions);

    /// @brief get the left border of the given lane (the leftmost one by default)
    static PositionVector getLeftLaneBorder(const NBEdge* edge, int laneIndex = -1, double widthOffset = 0);
    static PositionVector getRightLaneBorder(const NBEdge* edge, int laneIndex = -1);

    /// @brief check if the lane geometries are compatible with OpenDRIVE assumptions (colinear stop line)
    static void checkLaneGeometries(const NBEdge* e);
};


#endif

/****************************************************************************/

