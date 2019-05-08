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
/// @file    NIVissimBoundedClusterObject.h
/// @author  Daniel Krajzewicz
/// @date    Sept 2002
/// @version $Id$
///
// -------------------
/****************************************************************************/
#ifndef NIVissimBoundedClusterObject_h
#define NIVissimBoundedClusterObject_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <set>
#include <string>


// ===========================================================================
// class declarations
// ===========================================================================
class Boundary;

// ===========================================================================
// class definitions
// ===========================================================================
/**
 *
 */
class NIVissimBoundedClusterObject {
public:
    NIVissimBoundedClusterObject();
    virtual ~NIVissimBoundedClusterObject();
    virtual void computeBounding() = 0;
    bool crosses(const AbstractPoly& poly, double offset = 0) const;
    void inCluster(int id);
    bool clustered() const;
    const Boundary& getBoundary() const;
public:
    static void closeLoading();
protected:
    typedef std::set<NIVissimBoundedClusterObject*> ContType;
    static ContType myDict;
    Boundary* myBoundary;
    int myClusterID;
};


#endif

/****************************************************************************/

