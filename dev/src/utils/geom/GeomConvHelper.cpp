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
/// @file    GeomConvHelper.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Sept 2003
/// @version $Id$
///
// Some helping functions for geometry parsing
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
#include <sstream>
#include <utils/geom/PositionVector.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/StringTokenizer.h>
#include <utils/common/TplConvert.h>
#include "GeomConvHelper.h"


// ===========================================================================
// method definitions
// ===========================================================================
PositionVector
GeomConvHelper::parseShapeReporting(const std::string& shpdef, const std::string& objecttype,
                                    const char* objectid, bool& ok, bool allowEmpty, bool report) {
    if (shpdef == "") {
        if (!allowEmpty) {
            emitError(report, "Shape", objecttype, objectid, "the shape is empty");
            ok = false;
        }
        return PositionVector();
    }
    StringTokenizer st(shpdef, " ");
    PositionVector shape;
    while (st.hasNext()) {
        StringTokenizer pos(st.next(), ",");
        if (pos.size() != 2 && pos.size() != 3) {
            emitError(report, "Shape", objecttype, objectid, "the position is neither x,y nor x,y,z");
            ok = false;
            return PositionVector();
        }
        try {
            double x = TplConvert::_2double(pos.next().c_str());
            double y = TplConvert::_2double(pos.next().c_str());
            if (pos.size() == 2) {
                shape.push_back(Position(x, y));
            } else {
                double z = TplConvert::_2double(pos.next().c_str());
                shape.push_back(Position(x, y, z));
            }
        } catch (NumberFormatException&) {
            emitError(report, "Shape", objecttype, objectid, "not numeric position entry");
            ok = false;
            return PositionVector();
        } catch (EmptyData&) {
            emitError(report, "Shape", objecttype, objectid, "empty position entry");
            ok = false;
            return PositionVector();
        }
    }
    return shape;
}


Boundary
GeomConvHelper::parseBoundaryReporting(const std::string& def, const std::string& objecttype,
                                       const char* objectid, bool& ok, bool report) {
    StringTokenizer st(def, ",");
    if (st.size() != 4) {
        emitError(report, "Bounding box", objecttype, objectid, "mismatching entry number");
        ok = false;
        return Boundary();
    }
    try {
        double xmin = TplConvert::_2double(st.next().c_str());
        double ymin = TplConvert::_2double(st.next().c_str());
        double xmax = TplConvert::_2double(st.next().c_str());
        double ymax = TplConvert::_2double(st.next().c_str());
        return Boundary(xmin, ymin, xmax, ymax);
    } catch (NumberFormatException&) {
        emitError(report, "Shape", objecttype, objectid, "not numeric entry");
    } catch (EmptyData&) {
        emitError(report, "Shape", objecttype, objectid, "empty entry");
    }
    ok = false;
    return Boundary();
}


void
GeomConvHelper::emitError(bool report, const std::string& what, const std::string& objecttype,
                          const char* objectid, const std::string& desc) {
    if (!report) {
        return;
    }
    std::ostringstream oss;
    oss << what << " of ";
    if (objectid == 0) {
        oss << "a(n) ";
    }
    oss << objecttype;
    if (objectid != 0) {
        oss << " '" << objectid << "'";
    }
    oss << " is broken: " << desc << ".";
    WRITE_ERROR(oss.str());
}



/****************************************************************************/

