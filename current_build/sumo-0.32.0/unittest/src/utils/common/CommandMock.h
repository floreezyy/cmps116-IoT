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
/// @file    CommandMock.h
/// @author  Matthias Heppner
/// @author  Michael Behrisch
/// @date    2009-11-30
/// @version $Id$
///
// 
/****************************************************************************/

#ifndef Command_Mock_h
#define Command_Mock_h

#include <utils/common/Command.h>

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class CommandMock
 * Mock Implementation for Unit Tests
 */
class CommandMock : public Command {
public:
    /** @brief Constructor
     */
	CommandMock() throw(){
		hasRun = false;
	}

    /// @brief Destructor
	~CommandMock() throw(){}

	/** @brief Executes the command.*/
    SUMOTime execute(SUMOTime currentTime) throw(ProcessError) {
		hasRun = true;
		return currentTime;
	} 

	/** return true, when execute was called, otherwise false */
	bool isExecuteCalled() {
		return hasRun;
	}

private:
	bool hasRun;  

};


#endif

/****************************************************************************/

