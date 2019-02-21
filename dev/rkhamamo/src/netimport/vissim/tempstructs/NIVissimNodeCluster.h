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
/// @file    NIVissimNodeCluster.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id$
///
// -------------------
/****************************************************************************/
#ifndef NIVissimNodeCluster_h
#define NIVissimNodeCluster_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <map>
#include <utils/geom/Position.h>


// ===========================================================================
// class declarations
// ===========================================================================
class NBNode;
class NBNodeCont;
class NBEdgeCont;
class NBDistrictCont;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 *
 */
class NIVissimNodeCluster {
public:
    NIVissimNodeCluster(int id, int nodeid, int tlid,
                        const std::vector<int>& connectors,
                        const std::vector<int>& disturbances,
                        bool amEdgeSplitOnly);
    ~NIVissimNodeCluster();
    int getID() const {
        return myID;
    }
    void buildNBNode(NBNodeCont& nc);
    bool recheckEdgeChanges();
    NBNode* getNBNode() const;
    Position getPos() const;
    std::string getNodeName() const;


public:
    static bool dictionary(int id, NIVissimNodeCluster* o);
    static int dictionary(int nodeid, int tlid, const std::vector<int>& connectors,
                          const std::vector<int>& disturbances, bool amEdgeSplitOnly);
    static NIVissimNodeCluster* dictionary(int id);
    static int contSize();
    static void assignToEdges();
    static void buildNBNodes(NBNodeCont& nc);
    static void dict_recheckEdgeChanges();
    static int getFromNode(int edgeid);
    static int getToNode(int edgeid);
    static void _debugOut(std::ostream& into);
    static void dict_addDisturbances(NBDistrictCont& dc,
                                     NBNodeCont& nc, NBEdgeCont& ec);
    static void clearDict();
    static void setCurrentVirtID(int id);


private:

    int myID;
    int myNodeID;
    int myTLID;
    std::vector<int> myConnectors;
    std::vector<int> myDisturbances;
    Position myPosition;
    typedef std::map<int, NIVissimNodeCluster*> DictType;
    static DictType myDict;
    static int myCurrentID;
    NBNode* myNBNode;
    bool myAmEdgeSplit;

};


#endif

/****************************************************************************/

