/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2004-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    OutputDevice_File.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    2004
/// @version $Id$
///
// An output device that encapsulates an ofstream
/****************************************************************************/
#ifndef OutputDevice_File_h
#define OutputDevice_File_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <fstream>
#include "OutputDevice.h"


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class OutputDevice_File
 * @brief An output device that encapsulates an ofstream
 *
 * Please note that the device gots responsible for the stream and deletes
 *  it (it should not be deleted elsewhere).
 */
class OutputDevice_File : public OutputDevice {
public:
    /** @brief Constructor
     * @param[in] strm The output stream to used
     * @exception IOError Should not be thrown by this implementation
     */
    OutputDevice_File(const std::string& fullName, const bool binary);


    /// @brief Destructor
    ~OutputDevice_File();


protected:
    /// @name Methods that override/implement OutputDevice-methods
    /// @{

    /** @brief Returns the associated ostream
     * @return The used stream
     */
    std::ostream& getOStream();
    /// @}


private:
    /// The wrapped ofstream
    std::ofstream* myFileStream;

};


#endif

/****************************************************************************/

