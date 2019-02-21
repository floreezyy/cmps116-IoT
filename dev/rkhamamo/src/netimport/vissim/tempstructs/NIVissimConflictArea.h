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
/// @file    NIVissimConflictArea.h
/// @author  Lukas Grohmann
/// @date    Aug 2015
/// @version $Id$
///
// -------------------
/****************************************************************************/
#ifndef NIVissimConflictArea_h
#define NIVissimConflictArea_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif


#include <map>
#include <string>
#include <utils/common/TplConvert.h>
#include "NIVissimConnection.h"
#include <netbuild/NBEdgeCont.h>
#include <netbuild/NBEdge.h>
#include <netbuild/NBNode.h>


// ===========================================================================
// class declarations
// ===========================================================================



// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class NIVissimConflictArea
 * @brief A temporary storage for conflict areas imported from Vissim
 */
class NIVissimConflictArea {
public:
    /// Constructor
    NIVissimConflictArea(int id, const std::string& link1,
                         const std::string& link2,
                         const std::string& status);


    /// Destructor
    ~NIVissimConflictArea();

public:
    /** @brief Adds the described item to the dictionary
        Builds the conflict area first */
    static bool dictionary(int id, const std::string& link1,
                           const std::string& link2, const std::string& status);

    /// Adds the conflict area to the dictionary
    static bool dictionary(int id, NIVissimConflictArea* ca);

    /// Returns the named dictionary
    static NIVissimConflictArea* dictionary(int id);

    /// Returns the conflict area from the dictionary, which defines
    /// the priority rule of the two given links
    static NIVissimConflictArea* dict_findByLinks(const std::string& link1,
            const std::string& link2);

    /// Clears the dictionary
    static void clearDict();

    /// Returns the dictionary including all conflict areas
    static std::map<int, NIVissimConflictArea*> getConflictAreas() {
        return myDict;
    }

    /// Returns the ID of the conflic area
    int getID() {
        return myConflictID;
    }

    /// Returns the first link of the conflic area
    std::string getFirstLink() {
        return myFirstLink;
    }

    /// Returns the second link of the conflic area
    std::string getSecondLink() {
        return mySecondLink;
    }

    /// Returns the priority regulation of the conflic area
    std::string getStatus() {
        return myStatus;
    }

    /// Sets the priority regulation according to the VISSIM conflict area data
    static void setPriorityRegulation(NBEdgeCont& ec);



private:
    /// The id of the conflict area
    int myConflictID;

    /// The first link of the conflict area
    std::string myFirstLink;

    /// The second link of the conflict area
    std::string mySecondLink;

    /// The priority regulation of the conflict area
    std::string myStatus;

private:
    /// @brief Definition of the dictionary type
    typedef std::map<int, NIVissimConflictArea*> DictType;

    /// @brief The dictionary
    static DictType myDict;
};


#endif

/****************************************************************************/
