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
/// @file    GUIEvent_Message.h
/// @author  Daniel Krajzewicz
/// @date    Wed 18 Jun 2003
/// @version $Id$
///
// Event send when a message (message, warning, error) has to besubmitted
/****************************************************************************/
#ifndef GUIEvent_Message_h
#define GUIEvent_Message_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "GUIEvent.h"
#include <string>
#include <utils/common/MsgHandler.h>


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * GUIEvent_Message
 * Throw from GUIRunThread to GUIApplicationWindow and then further to all
 * displays after a step has been performed
 */
class GUIEvent_Message : public GUIEvent {
public:
    /// constructor
    GUIEvent_Message(const std::string& msg)
        : GUIEvent(EVENT_STATUS_OCCURED), myMsg(msg) {
    }

    /// constructor
    GUIEvent_Message(MsgHandler::MsgType type, const std::string& msg)
        : GUIEvent(EVENT_MESSAGE_OCCURED), myMsg(msg) {
        switch (type) {
            case MsgHandler::MT_MESSAGE:
                myType = EVENT_MESSAGE_OCCURED;
                break;
            case MsgHandler::MT_WARNING:
                myType = EVENT_WARNING_OCCURED;
                break;
            case MsgHandler::MT_ERROR:
                myType = EVENT_ERROR_OCCURED;
                break;
            default:
                throw 1;
        }
    }

    /// destructor
    ~GUIEvent_Message() { }

    /// Returns the message
    const std::string& getMsg() const {
        return myMsg;
    }

protected:

    /// The message
    std::string myMsg;

};


#endif

/****************************************************************************/

