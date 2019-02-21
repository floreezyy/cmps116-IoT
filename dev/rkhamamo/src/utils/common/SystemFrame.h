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
/// @file    SystemFrame.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Mon, 23.06.2003
/// @version $Id$
///
// A set of actions common to all applications
/****************************************************************************/
#ifndef SystemFrame_h
#define SystemFrame_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif


// ===========================================================================
// class declarations
// ===========================================================================
class OptionsCont;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class SystemFrame
 * @brief A set of actions common to all applications
 *
 * As almost all applications within the SUMO-package share the same
 *  initialisation procedure, it is encapsulated within this class.
 *
 * Only two things are done herein, so far. The first is to insert
 *  options into the given options container that are used for dealing
 *  with the application's configuration.
 *
 * Additionally, a closing method may be found, which closes all used
 *  subsystems.
 */
class SystemFrame {
public:
    /** @brief Adds configuration options to the given container
     *
     * @param[in] oc The options container to add the options to
     * @todo let the container be retrieved
     */
    static void addConfigurationOptions(OptionsCont& oc);


    /** @brief Adds reporting options to the given container
     *
     * @param[in] oc The options container to add the options to
     * @todo let the container be retrieved
     */
    static void addReportOptions(OptionsCont& oc);

    /// @brief checks shared options and sets StdDefs
    static bool checkOptions();

    /** @brief Closes all of an applications subsystems
     *
     * Closes (in this order)
     * @arg The xml subsystem
     * @arg The options subsystem
     * @arg The message subsystem
     * @see XMLSubSys::close()
     * @see OptionsCont::clear()
     * @see MsgHandler::cleanupOnEnd()
     */
    static void close();


};


#endif

/****************************************************************************/

