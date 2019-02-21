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
/// @file    Boundary.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id$
///
// A class that stores a 2D geometrical boundary
/****************************************************************************/
#ifndef Boundary_h
#define Boundary_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <iostream>
#include <utility>
#include "AbstractPoly.h"
#include "Position.h"


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class Boundary
 * @brief A class that stores a 2D geometrical boundary
 */
class Boundary
    : public AbstractPoly {
public:
    /// Constructor - the boundary is unset
    Boundary();

    /// Constructor - the boundary will be build using the given values
    Boundary(double x1, double y1, double x2, double y2);

    Boundary(double x1, double y1, double z1, double x2, double y2, double z2);

    /// Destructor
    ~Boundary();

    /// Resets the boundary
    void reset();

    /// Makes the boundary include the given coordinate
    void add(double x, double y, double z = 0);

    /// Makes the boundary include the given coordinate
    void add(const Position& p);

    /// Makes the boundary include the given boundary
    void add(const Boundary& p);

    /// Returns the center of the boundary
    Position getCenter() const;

    /// Returns minimum x-coordinate
    double xmin() const;

    /// Returns maximum x-coordinate
    double xmax() const;

    /// Returns minimum y-coordinate
    double ymin() const;

    /// Returns maximum y-coordinate
    double ymax() const;

    /// Returns minimum z-coordinate
    double zmin() const;

    /// Returns maximum z-coordinate
    double zmax() const;

    /// Returns the width of the boudary (x-axis)
    double getWidth() const;

    /// Returns the height of the boundary (y-axis)
    double getHeight() const;

    /// Returns the elevation range of the boundary (z-axis)
    double getZRange() const;

    /// Returns whether the boundary contains the given coordinate
    bool around(const Position& p, double offset = 0) const;

    /// Returns whether the boundary overlaps with the given polygon
    bool overlapsWith(const AbstractPoly& poly, double offset = 0) const;

    /// Returns whether the boundary is partially within the given polygon
    bool partialWithin(const AbstractPoly& poly, double offset = 0) const;

    /// Returns whether the boundary crosses the given line
    bool crosses(const Position& p1, const Position& p2) const;

    /// @brief returns the euclidean distance in the x-y-plane
    double distanceTo2D(const Position& p) const;

    /// @brief returns the euclidean distance in the x-y-plane
    double distanceTo2D(const Boundary& b) const;


    /** @brief extends the boundary by the given amount
     *
     * The method returns a reference to the instance for further use */
    Boundary& grow(double by);

    /// Increases the width of the boundary (x-axis)
    void growWidth(double by);

    /// Increases the height of the boundary (y-axis)
    void growHeight(double by);

    /// flips ymin and ymax
    void flipY();

    /// Sets the boundary to the given values
    void set(double xmin, double ymin, double xmax, double ymax);

    /// Moves the boundary by the given amount
    void moveby(double x, double y, double z = 0);

    /// Output operator
    friend std::ostream& operator<<(std::ostream& os, const Boundary& b);

private:
    /// The boundaries
    double myXmin, myXmax, myYmin, myYmax, myZmin, myZmax;

    /// Information whether the boundary was initialised
    bool myWasInitialised;

};


#endif

/****************************************************************************/

