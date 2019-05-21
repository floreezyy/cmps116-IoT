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
/// @file    Distribution_Points.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id$
///
// The description of a distribution by a curve
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <cassert>
#include <sstream>
#include <iomanip>
#include <utils/common/StdDefs.h>
#include "Distribution_Points.h"


// ===========================================================================
// method definitions
// ===========================================================================
Distribution_Points::Distribution_Points(const std::string& id)
    : Distribution(id) {}


Distribution_Points::~Distribution_Points() {}


double
Distribution_Points::getMax() const {
    assert(getVals().size() > 0);
    return getVals().back();
}


std::string
Distribution_Points::toStr(std::streamsize accuracy) const {
    std::stringstream oss;
    oss << std::setprecision(accuracy);
    const std::vector<double> vals = getVals();
    for (int i = 0; i < (int)vals.size(); i++) {
        if (i > 0) {
            oss << ",";
        }
        oss << vals[i] << ":" << getProbs()[i];
    }
    return "points(" + oss.str() + ")";
}


/****************************************************************************/
