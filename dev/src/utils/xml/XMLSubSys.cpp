/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2002-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    XMLSubSys.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Mon, 1 Jul 2002
/// @version $Id$
///
// Utility methods for initialising, closing and using the XML-subsystem
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <xercesc/util/PlatformUtils.hpp>
#include <utils/common/MsgHandler.h>
#include <utils/common/TplConvert.h>
#include "SUMOSAXHandler.h"
#include "SUMOSAXReader.h"
#include "XMLSubSys.h"


// ===========================================================================
// static member variables
// ===========================================================================
std::vector<SUMOSAXReader*> XMLSubSys::myReaders;
int XMLSubSys::myNextFreeReader;
XERCES_CPP_NAMESPACE::SAX2XMLReader::ValSchemes XMLSubSys::myValidationScheme = XERCES_CPP_NAMESPACE::SAX2XMLReader::Val_Auto;
XERCES_CPP_NAMESPACE::SAX2XMLReader::ValSchemes XMLSubSys::myNetValidationScheme = XERCES_CPP_NAMESPACE::SAX2XMLReader::Val_Auto;


// ===========================================================================
// method definitions
// ===========================================================================
void
XMLSubSys::init() {
    try {
        XERCES_CPP_NAMESPACE::XMLPlatformUtils::Initialize();
        myNextFreeReader = 0;
    } catch (const XERCES_CPP_NAMESPACE::XMLException& e) {
        throw ProcessError("Error during XML-initialization:\n " + TplConvert::_2str(e.getMessage()));
    }
}


void
XMLSubSys::setValidation(const std::string& validationScheme, const std::string& netValidationScheme) {
    if (validationScheme == "never") {
        myValidationScheme = XERCES_CPP_NAMESPACE::SAX2XMLReader::Val_Never;
    } else if (validationScheme == "auto") {
        myValidationScheme = XERCES_CPP_NAMESPACE::SAX2XMLReader::Val_Auto;
    } else if (validationScheme == "always") {
        myValidationScheme = XERCES_CPP_NAMESPACE::SAX2XMLReader::Val_Always;
    } else {
        throw ProcessError("Unknown xml validation scheme + '" + validationScheme + "'.");
    }
    if (netValidationScheme == "never") {
        myNetValidationScheme = XERCES_CPP_NAMESPACE::SAX2XMLReader::Val_Never;
    } else if (netValidationScheme == "auto") {
        myNetValidationScheme = XERCES_CPP_NAMESPACE::SAX2XMLReader::Val_Auto;
    } else if (netValidationScheme == "always") {
        myNetValidationScheme = XERCES_CPP_NAMESPACE::SAX2XMLReader::Val_Always;
    } else {
        throw ProcessError("Unknown network validation scheme + '" + netValidationScheme + "'.");
    }
}


void
XMLSubSys::close() {
    for (std::vector<SUMOSAXReader*>::iterator i = myReaders.begin(); i != myReaders.end(); ++i) {
        delete *i;
    }
    myReaders.clear();
    XERCES_CPP_NAMESPACE::XMLPlatformUtils::Terminate();
}


SUMOSAXReader*
XMLSubSys::getSAXReader(SUMOSAXHandler& handler) {
    return new SUMOSAXReader(handler, myValidationScheme);
}


void
XMLSubSys::setHandler(GenericSAXHandler& handler) {
    myReaders[myNextFreeReader - 1]->setHandler(handler);
}


bool
XMLSubSys::runParser(GenericSAXHandler& handler,
                     const std::string& file, const bool isNet) {
    try {
        XERCES_CPP_NAMESPACE::SAX2XMLReader::ValSchemes validationScheme = isNet ? myNetValidationScheme : myValidationScheme;
        if (myNextFreeReader == (int)myReaders.size()) {
            myReaders.push_back(new SUMOSAXReader(handler, validationScheme));
        } else {
            myReaders[myNextFreeReader]->setValidation(validationScheme);
            myReaders[myNextFreeReader]->setHandler(handler);
        }
        myNextFreeReader++;
        std::string prevFile = handler.getFileName();
        handler.setFileName(file);
        myReaders[myNextFreeReader - 1]->parse(file);
        handler.setFileName(prevFile);
        myNextFreeReader--;
    } catch (ProcessError& e) {
        WRITE_ERROR(std::string(e.what()) != std::string("") ? std::string(e.what()) : std::string("Process Error"));
        return false;
    } catch (const std::runtime_error& re) {
        WRITE_ERROR("Runtime error: " + std::string(re.what()) + " while parsing '" + file + "'");
        return false;
    } catch (const std::exception& ex) {
        WRITE_ERROR("Error occurred: " + std::string(ex.what()) + " while parsing '" + file + "'");
        return false;
    } catch (...) {
        WRITE_ERROR("Unspecified error occured wile parsing '" + file + "'");
        return false;
    }
    return !MsgHandler::getErrorInstance()->wasInformed();
}


/****************************************************************************/

