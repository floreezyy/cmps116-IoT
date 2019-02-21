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
/// @file    NIVissimNodeDef_Edges.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id$
///
// -------------------
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
#include <map>
#include <algorithm>
#include <cassert>
#include <utils/geom/Boundary.h>
#include "NIVissimNodeParticipatingEdgeVector.h"
#include "NIVissimNodeDef.h"
#include "NIVissimEdge.h"
#include "NIVissimNodeDef_Edges.h"
#include "NIVissimDisturbance.h"
#include "NIVissimConnection.h"


// ===========================================================================
// method definitions
// ===========================================================================
NIVissimNodeDef_Edges::NIVissimNodeDef_Edges(int id,
        const std::string& name, const NIVissimNodeParticipatingEdgeVector& edges)
    : NIVissimNodeDef(id, name), myEdges(edges) {}


NIVissimNodeDef_Edges::~NIVissimNodeDef_Edges() {
    for (NIVissimNodeParticipatingEdgeVector::iterator i = myEdges.begin(); i != myEdges.end(); i++) {
        delete(*i);
    }
    myEdges.clear();
}


bool
NIVissimNodeDef_Edges::dictionary(int id, const std::string& name,
                                  const NIVissimNodeParticipatingEdgeVector& edges) {
    NIVissimNodeDef_Edges* o = new NIVissimNodeDef_Edges(id, name, edges);
    if (!NIVissimNodeDef::dictionary(id, o)) {
        delete o;
        return false;
    }
    return true;
}


/*
void
NIVissimNodeDef_Edges::searchAndSetConnections() {
    std::vector<int> connections;
    std::vector<int> edges;
    Boundary boundary;
    for (NIVissimNodeParticipatingEdgeVector::const_iterator i = myEdges.begin(); i != myEdges.end(); i++) {
        NIVissimNodeParticipatingEdge* edge = *i;
        NIVissimConnection* c =
            NIVissimConnection::dictionary(edge->getID());
        NIVissimEdge* e =
            NIVissimEdge::dictionary(edge->getID());
        if (c != 0) {
            connections.push_back(edge->getID());
            boundary.add(c->getFromGeomPosition());
            boundary.add(c->getToGeomPosition());
            c->setNodeCluster(myID);
        }
        if (e != 0) {
            edges.push_back(edge->getID());
            boundary.add(e->getGeomPosition(edge->getFromPos()));
            boundary.add(e->getGeomPosition(edge->getToPos()));
        }
    }
    NIVissimConnectionCluster* c =
        new NIVissimConnectionCluster(connections, boundary, myID, edges);
    for (std::vector<int>::iterator j = edges.begin(); j != edges.end(); j++) {
        NIVissimEdge* edge = NIVissimEdge::dictionary(*j);
        edge->myConnectionClusters.push_back(c);
    }
}
*/


double
NIVissimNodeDef_Edges::getEdgePosition(int edgeid) const {
    for (NIVissimNodeParticipatingEdgeVector::const_iterator i = myEdges.begin(); i != myEdges.end(); i++) {
        NIVissimNodeParticipatingEdge* edge = *i;
        if (edge->getID() == edgeid) {
            return (edge->getFromPos() + edge->getToPos()) / (double) 2.0;
        }
    }
    return -1;
}



/****************************************************************************/

