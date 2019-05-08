/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2010-2017 German Aerospace Center (DLR) and others.
// activitygen module
// Copyright 2010 TUM (Technische Universitaet Muenchen, http://www.tum.de/)
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    AGActivityTripWriter.h
/// @author  Piotr Woznica
/// @author  Daniel Krajzewicz
/// @author  Walter Bamberger
/// @author  Michael Behrisch
/// @date    July 2010
/// @version $Id$
///
// Class for writing Trip objects in a SUMO-route file.
/****************************************************************************/
#ifndef AGACTIVITYTRIPWRITER_H
#define AGACTIVITYTRIPWRITER_H


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
class OutputDevice;
class AGTrip;


// ===========================================================================
// class definitions
// ===========================================================================
class AGActivityTripWriter {
public:
    AGActivityTripWriter(OutputDevice& file);

    void addTrip(const AGTrip& trip);

private:
    OutputDevice& myTripOutput;

private:
    /// @brief Invalidated copy constructor.
    AGActivityTripWriter(const AGActivityTripWriter&);

    /// @brief Invalidated assignment operator.
    AGActivityTripWriter& operator=(const AGActivityTripWriter&);

};


#endif

/****************************************************************************/
