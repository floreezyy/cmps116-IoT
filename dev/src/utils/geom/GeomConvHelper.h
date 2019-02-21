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
/// @file    GeomConvHelper.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Sept 2003
/// @version $Id$
///
// Some helping functions for geometry parsing
/****************************************************************************/
#ifndef GeomConvHelper_h
#define GeomConvHelper_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <utils/geom/PositionVector.h>
#include <utils/geom/Boundary.h>


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GeomConvHelper
 * This class holds some helping functions for the parsing of geometries
 */
class GeomConvHelper {
public:
    /** @brief Builds a PositionVector from a string representation, reporting occured errors
     *
     * It is assumed, the vector is stored as "x,y[ x,y]*" where x and y are doubles.
     * @param[in] shpdef The shape definition to parse
     * @param[in] objecttype The name of the parsed object type; used for error message generation
     * @param[in] objectid The name of the parsed object; used for error message generation
     * @param[out] ok Whether the value could be read
     * @param[in] allowEmpty Whether an empty shape definition is valid
     * @param[in] report Whether errors shall be written to msg handler's error instance
     * @return The parsed position vector
     */
    static PositionVector parseShapeReporting(const std::string& shpdef, const std::string& objecttype,
            const char* objectid, bool& ok, bool allowEmpty, bool report = true);


    /** @brief Builds a boundary from its string representation, reporting occured errors
     *
     * It is assumed that the boundary is stored as a quadruple of double, divided by ','.
     * @param[in] def The boundary definition to parse
     * @param[in] objecttype The name of the parsed object type; used for error message generation
     * @param[in] objectid The name of the parsed object; used for error message generation
     * @param[out] ok Whether the value could be read
     * @param[in] report Whether errors shall be written to msg handler's error instance
     * @return The parsed boundary
     */
    static Boundary parseBoundaryReporting(const std::string& def, const std::string& objecttype,
                                           const char* objectid, bool& ok, bool report = true);


private:
    /** @brief Writes an error message into the MessageHandler
     * @param[in] report Whether errors shall be written to msg handler's error instance
     * @param[in] what Name of the parsed object ("Shape", or "Boundary")
     * @param[in] objecttype The name of the parsed object type the error occured at
     * @param[in] objectid The name of the parsed object type the error occured at
     * @param[out] desc Error description
     */
    static void emitError(bool report, const std::string& what, const std::string& objecttype,
                          const char* objectid, const std::string& desc);


};


#endif

/****************************************************************************/

