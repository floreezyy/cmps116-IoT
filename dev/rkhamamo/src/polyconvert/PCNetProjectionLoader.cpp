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
/// @file    PCNetProjectionLoader.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Thu, 02.11.2006
/// @version $Id$
///
// A reader for a SUMO network's projection description
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
#include <fstream>
#include <utils/options/OptionsCont.h>
#include <utils/options/Option.h>
#include <utils/common/StdDefs.h>
#include <polyconvert/PCPolyContainer.h>
#include <utils/common/RGBColor.h>
#include <utils/geom/GeomHelper.h>
#include <utils/geom/Boundary.h>
#include <utils/geom/Position.h>
#include <utils/geom/GeoConvHelper.h>
#include <utils/xml/XMLSubSys.h>
#include <utils/xml/SUMOSAXReader.h>
#include <utils/geom/GeomConvHelper.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/FileHelpers.h>
#include <utils/xml/SUMOXMLDefinitions.h>
#include "PCNetProjectionLoader.h"


// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// static interface
// ---------------------------------------------------------------------------
void
PCNetProjectionLoader::load(const std::string& file, double scale) {
    if (!FileHelpers::isReadable(file)) {
        throw ProcessError("Could not open net-file '" + file + "'.");
    }
    // build handler and parser
    PCNetProjectionLoader handler(scale);
    handler.setFileName(file);
    SUMOSAXReader* parser = XMLSubSys::getSAXReader(handler);
    PROGRESS_BEGIN_MESSAGE("Parsing network projection from '" + file + "'");
    if (!parser->parseFirst(file)) {
        delete parser;
        throw ProcessError("Can not read XML-file '" + handler.getFileName() + "'.");
    }
    // parse
    while (parser->parseNext() && !handler.hasReadAll());
    // clean up
    PROGRESS_DONE_MESSAGE();
    if (!handler.hasReadAll()) {
        throw ProcessError("Could not find projection parameter in net.");
    }
    delete parser;
}



// ---------------------------------------------------------------------------
// handler methods
// ---------------------------------------------------------------------------
PCNetProjectionLoader::PCNetProjectionLoader(double scale) :
    SUMOSAXHandler("sumo-network"),
    myFoundLocation(false),
    myScale(scale) {
}


PCNetProjectionLoader::~PCNetProjectionLoader() {}


void
PCNetProjectionLoader::myStartElement(int element,
                                      const SUMOSAXAttributes& attrs) {
    if (element != SUMO_TAG_LOCATION) {
        return;
    }
    // @todo refactor parsing of location since its duplicated in NLHandler and PCNetProjectionLoader
    myFoundLocation = true;
    PositionVector s = attrs.get<PositionVector>(SUMO_ATTR_NET_OFFSET, 0, myFoundLocation);
    Boundary convBoundary = attrs.get<Boundary>(SUMO_ATTR_CONV_BOUNDARY, 0, myFoundLocation);
    Boundary origBoundary = attrs.get<Boundary>(SUMO_ATTR_ORIG_BOUNDARY, 0, myFoundLocation);
    std::string proj = attrs.get<std::string>(SUMO_ATTR_ORIG_PROJ, 0, myFoundLocation);
    if (myFoundLocation) {
        OptionsCont& oc = OptionsCont::getOptions();
        Position networkOffset = s[0] + Position(oc.getFloat("offset.x"), oc.getFloat("offset.y"));
        GeoConvHelper::init(proj, networkOffset, origBoundary, convBoundary, myScale);
    }
}


bool
PCNetProjectionLoader::hasReadAll() const {
    return myFoundLocation;
}


/****************************************************************************/

