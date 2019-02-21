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
/// @file    PCNetProjectionLoader.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Thu, 02.11.2006
/// @version $Id$
///
// A reader for a SUMO network's projection description
/****************************************************************************/
#ifndef PCNetProjectionLoader_h
#define PCNetProjectionLoader_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <utils/xml/SUMOSAXHandler.h>
#include <utils/common/UtilExceptions.h>
#include <utils/geom/Position.h>


// ===========================================================================
// class definitions
// ===========================================================================
class OptionsCont;


// ===========================================================================
// class declarations
// ===========================================================================
/**
 * @class PCNetProjectionLoader
 * @brief A reader for a SUMO network's projection description
 */
class PCNetProjectionLoader : public SUMOSAXHandler {
public:
    /** @brief Loads network projection if wished
     *
     * @param[in] file The network file from which to parse the location element
     * @param[in] shift The shift of the decimal point when interpreting loaded coordinates
     */
    static void load(const std::string& file, double scale);


protected:
    /** @brief Constructor
     */
    PCNetProjectionLoader(double scale);


    /// @brief Destructor
    ~PCNetProjectionLoader();


    /** @brief Returns whether all needed values were read
     * @return Whether all needed values were read
     */
    bool hasReadAll() const;


protected:
    /// @name inherited from GenericSAXHandler
    //@{

    /** @brief Called on the opening of a tag;
     *
     * @param[in] element ID of the currently opened element
     * @param[in] attrs Attributes within the currently opened element
     * @exception ProcessError If something fails
     * @see GenericSAXHandler::myStartElement
     */
    virtual void myStartElement(int element,
                                const SUMOSAXAttributes& attrs);
    //@}


private:
    /// @brief Information whether the parameter was read
    bool myFoundLocation;

    /// @brief scaling of input coordinates (not given in the location element)
    double myScale;


};


#endif

/****************************************************************************/

