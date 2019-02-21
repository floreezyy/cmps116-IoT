/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2004-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    ROAbstractEdgeBuilder.h
/// @author  Daniel Krajzewicz
/// @author  Yun-Pang Floetteroed
/// @date    Wed, 21 Jan 2004
/// @version $Id$
///
// Interface for building instances of router-edges
/****************************************************************************/
#ifndef ROAbstractEdgeBuilder_h
#define ROAbstractEdgeBuilder_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>


// ===========================================================================
// class declarations
// ===========================================================================
class ROEdge;
class RONode;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class ROAbstractEdgeBuilder
 * @brief Interface for building instances of router-edges
 *
 * As the different routing algorithms may need certain types of edges,
 *  edges are build via a factory object derived from this class.
 *
 * The only method to be implemented is "buildEdge" which builds an edge
 *  of the needed ROEdge-subtype.
 *
 * The built edges are numbered in the order they are built, the current
 *  number (index) is stored in "myCurrentIndex" and the next to use may
 *  be obtained via "getNextIndex".
 */
class ROAbstractEdgeBuilder {
public:
    /// @brief Constructor
    ROAbstractEdgeBuilder() : myCurrentIndex(0) { }


    /// @brief Destructor
    virtual ~ROAbstractEdgeBuilder() { }


    /// @name Methods to be implemented
    /// @{

    /** @brief Builds an edge with the given name
     *
     * @param[in] name The name of the edge
     * @param[in] from The node the edge begins at
     * @param[in] to The node the edge ends at
     * @param[in] priority The edge priority (road class)
     * @return A proper instance of the named edge
     */
    virtual ROEdge* buildEdge(const std::string& name, RONode* from, RONode* to, const int priority) = 0;
    /// @}


protected:
    /** @brief Returns the index of the edge to built
     * @return Next valid edge index
     */
    int getNextIndex() {
        return myCurrentIndex++;
    }


private:
    /// @brief The next edge's index
    int myCurrentIndex;


private:
    /// @brief Invalidated copy constructor
    ROAbstractEdgeBuilder(const ROAbstractEdgeBuilder& src);

    /// @brief Invalidated assignment operator
    ROAbstractEdgeBuilder& operator=(const ROAbstractEdgeBuilder& src);

};


#endif

/****************************************************************************/

