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
/// @file    Command.h
/// @author  Christian Roessel
/// @author  Daniel Krajzewicz
/// @date    Thu, 20 Dec 2001
/// @version $Id$
///
// Base (microsim) event class
/****************************************************************************/
#ifndef Command_h
#define Command_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <utils/common/SUMOTime.h>
#include <utils/common/UtilExceptions.h>


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class Command
 * @brief Base (microsim) event class
 *
 * Classes derived from Command may be added to MSEventControl instances in
 *  order to be executed at a certain time step.
 *
 * As soon as the simulation reaches the desired time step, the command (event)
 *  is executed by calling "execute" with the current time step. The method must
 *  return either 0, if the event shall not be executed again or a positive value
 *  (in simulation seconds) that described when it shall be executed again. The method
 *  must not return a value below zero, the behaviour is undefined in this case.
 *
 * @warning The EventControl the Command is added to gets responsible for
 *  this command's deletion.
 *
 * @see Design Patterns, Gamma et al.
 * @see WrappingCommand
 * @see MSEventControl
 */
class Command {
public:
    /// @brief Constructor
    Command() { }


    /// @brief Destructor.
    virtual ~Command() { }


    /** @brief Executes the command.
     *
     * The implementations should return 0 if the command shall not be repeated,
     *  or a value larger than 0 that describe the time after which the command
     *  shall be executed again. Values below 0 must not be returned.
     *
     * @param[in] currentTime The current simulation time
     * @return The time after which the command shall be executed again, 0 if this command shall be descheduled.
     * @exception ProcessError Derived actions may throw this exception
     */
    virtual SUMOTime execute(SUMOTime currentTime) = 0;


};


#endif

/****************************************************************************/

