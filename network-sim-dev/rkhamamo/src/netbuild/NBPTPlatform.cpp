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
/// @file    NBPTPlatform.cpp
/// @author  Gregor Laemmel
/// @date    Tue, 24 Aug 2017
/// @version $Id$
///
// The representation of a pt platform
/****************************************************************************/
#include "NBPTPlatform.h"
NBPTPlatform::NBPTPlatform(Position position, double d): myPos(position), myLength(d) {

}
Position* NBPTPlatform::getMyPos()  {
    return &myPos;
}
double NBPTPlatform::getMyLength() const {
    return myLength;
}
