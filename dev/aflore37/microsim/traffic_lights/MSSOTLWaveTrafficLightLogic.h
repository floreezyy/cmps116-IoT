/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2010-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    MSSOTLWaveTrafficLightLogic.h
/// @author  Riccardo Belletti
/// @author  Anna Chiara Bellini
/// @date    2013-10-26
/// @version $Id$
///
// The class for SOTL Platoon logics
/****************************************************************************/
#ifndef MSSOTLWaveTrafficLightLogic_h
#define MSSOTLWaveTrafficLightLogic_h

// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "MSSOTLTrafficLightLogic.h"
class MSSOTLWaveTrafficLightLogic: public MSSOTLTrafficLightLogic {
public:
    /**
     * @brief Constructor without sensors passed
     * @param[in] tlcontrol The tls control responsible for this tls
     * @param[in] id This tls' id
     * @param[in] subid This tls' sub-id (program id)
     * @param[in] phases Definitions of the phases
     * @param[in] step The initial phase index
     * @param[in] delay The time to wait before the first switch
     */
    MSSOTLWaveTrafficLightLogic(MSTLLogicControl& tlcontrol,
                                const std::string& id, const std::string& subid,
                                const Phases& phases, int step, SUMOTime delay,
                                const std::map<std::string, std::string>& parameters) throw();

    /**
     * @brief Constructor with sensors passed
     * @param[in] tlcontrol The tls control responsible for this tls
     * @param[in] id This tls' id
     * @param[in] subid This tls' sub-id (program id)
     * @param[in] phases Definitions of the phases
     * @param[in] step The initial phase index
     * @param[in] delay The time to wait before the first switch
     */
    MSSOTLWaveTrafficLightLogic(MSTLLogicControl& tlcontrol,
                                const std::string& id, const std::string& subid,
                                const Phases& phases, int step, SUMOTime delay,
                                const std::map<std::string, std::string>& parameters,
                                MSSOTLSensors* sensors) throw();

    /** @brief Returns the type of the logic as a string
     * @return The type of the logic
     */
    const std::string getLogicType() const {
        return "waveTrafficLightLogic";
    }
    /// @}

protected:

    /*
     * @brief Contains the logic to decide whether to release the green light
     */
    bool canRelease() throw();

private:

    /*
     * @brief Counts the vehicles on the green lanes of this phase
     */
    int countVehicles();

};

#endif
/****************************************************************************/
