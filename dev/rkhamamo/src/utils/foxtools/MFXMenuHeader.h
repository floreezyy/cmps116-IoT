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
/// @file    MFXMenuHeader.h
/// @author  Daniel Krajzewicz
/// @date    2004-07-02
/// @version $Id$
///
// Header for menu commands
/****************************************************************************/
#ifndef MFXMenuHeader_h
#define MFXMenuHeader_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <fx.h>

class MFXMenuHeader : public FXMenuCommand {
public:
    /// @brief constructor
    MFXMenuHeader(FXComposite* p, FXFont* fnt, const FXString& text, FXIcon* ic = NULL,
                  FXObject* tgt = NULL, FXSelector sel = 0, FXuint opts = 0);

    /// @brief destructor
    ~MFXMenuHeader();
};


#endif

/****************************************************************************/

