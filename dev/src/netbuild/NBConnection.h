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
/// @file    NBConnection.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Sascha Krieg
/// @date    Sept 2002
/// @version $Id$
///
// The class holds a description of a connection between two edges
/****************************************************************************/
#ifndef NBConnection_h
#define NBConnection_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include "NBEdge.h"


// ===========================================================================
// class declarations
// ===========================================================================
class NBNode;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class NBConnection
 */
class NBConnection {
public:
    /// @brief Constructor
    NBConnection(NBEdge* from, NBEdge* to);

    /// @brief Constructor
    NBConnection(NBEdge* from, int fromLane, NBEdge* to, int toLane, int tlIndex = InvalidTlIndex);

    /// @brief Constructor
    NBConnection(const std::string& fromID, NBEdge* from,
                 const std::string& toID, NBEdge* to);

    /// @brief Constructor
    NBConnection(const NBConnection& c);

    /// @brief Destructor
    virtual ~NBConnection();

    /// @brief returns the from-edge (start of the connection)
    NBEdge* getFrom() const;

    /// @brief returns the to-edge (end of the connection)
    NBEdge* getTo() const;

    /// @brief replaces the from-edge by the one given
    bool replaceFrom(NBEdge* which, NBEdge* by);

    /// @brief replaces the from-edge by the one given
    bool replaceFrom(NBEdge* which, int whichLane, NBEdge* by, int byLane);

    /// @brief replaces the to-edge by the one given
    bool replaceTo(NBEdge* which, NBEdge* by);

    /// @brief replaces the to-edge by the one given
    bool replaceTo(NBEdge* which, int whichLane, NBEdge* by, int byLane);

    /// @brief  patches lane indices refering to the given edge
    void shiftLaneIndex(NBEdge* edge, int offset);

    /// @brief checks whether the edges are still valid
    bool check(const NBEdgeCont& ec);

    /// @brief returns the from-lane
    int getFromLane() const;

    /// @brief returns the to-lane
    int getToLane() const;

    /// @brief returns the index within the controlling tls or InvalidTLIndex if this link is unontrolled
    int getTLIndex() const {
        return myTlIndex;
    }

    // @brief reset the tlIndex
    void setTLIndex(int tlIndex) {
        myTlIndex = tlIndex;
    }

    /// @brief returns the id of the connection (!!! not really pretty)
    std::string getID() const;

    /// @brief Compares both connections in order to allow sorting
    friend bool operator<(const NBConnection& c1, const NBConnection& c2);

    /// @brief Comparison operator
    bool operator==(const NBConnection& c) const;

    /// @brief Comparison operator
    bool operator!=(const NBConnection& c) const {
        return !(*this == c);
    }

    /// @brief Output operator
    friend std::ostream& operator<<(std::ostream& os, const NBConnection& c);

    const static int InvalidTlIndex;
    const static NBConnection InvalidConnection;

private:
    /// @brief Checks whether the from-edge is still valid
    NBEdge* checkFrom(const NBEdgeCont& ec);

    /// @brief Checks whether the to-edge is still valid
    NBEdge* checkTo(const NBEdgeCont& ec);

private:
    /// @brief The from- and the to-edges
    NBEdge* myFrom, *myTo;

    /// @brief The names of both edges, needed for verification of validity
    std::string myFromID, myToID;

    /// @brief The lanes; may be -1 if no certain lane was specified
    int myFromLane, myToLane;

    // @brief the index within the controlling tls if this connection is tls-controlled
    int myTlIndex;
};


#endif

/****************************************************************************/

