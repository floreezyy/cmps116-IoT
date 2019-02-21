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
/// @file    METriggeredCalibrator.h
/// @author  Daniel Krajzewicz
/// @date    Tue, May 2005
/// @version $Id$
///
// Calibrates the flow on a segment to a specified one
/****************************************************************************/
#ifndef METriggeredCalibrator_h
#define METriggeredCalibrator_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <vector>
#include <microsim/trigger/MSCalibrator.h>
#include <mesosim/MESegment.h>


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class METriggeredCalibrator
 * @brief Calibrates the flow on a segment to a specified one
 */
class METriggeredCalibrator : public MSCalibrator {
public:
    /** constructor */
    METriggeredCalibrator(const std::string& id,
                          const MSEdge* const edge, const double pos,
                          const std::string& aXMLFilename,
                          const std::string& outputFilename,
                          const SUMOTime freq, const double length,
                          const MSRouteProbe* probe);

    /** destructor */
    virtual ~METriggeredCalibrator();


    /** the implementation of the MSTrigger / Command interface.
        Calibrating takes place here. */
    SUMOTime execute(SUMOTime currentTime);

protected:

    bool tryEmit(MESegment* s, MEVehicle* vehicle);

    inline int passed() const {
        // calibrator measures at start of segment
        return myEdgeMeanData.nVehEntered + myEdgeMeanData.nVehDeparted - myEdgeMeanData.nVehVaporized;
    }

    /// @brief returns whether the segment is jammed although it should not be
    bool invalidJam() const;

    /// @brief returns the number of vehicles (of the current type) that still fit onto the segment
    int remainingVehicleCapacity() const;

    /// @brief reset collected vehicle data
    void reset();

    /// @brief do nothing
    void updateMeanData() {}

    /// @brief returns the maximum number of vehicles that could enter from upstream until the calibrator is activated again
    inline int maximumInflow() const {
        return (int)std::ceil((double)myFrequency / (double)mySegment->getMinimumHeadwayTime());
    }

private:
    /// @brief mesoscopic edge segment the calibrator lies on
    MESegment* mySegment;

};

#endif

/****************************************************************************/
