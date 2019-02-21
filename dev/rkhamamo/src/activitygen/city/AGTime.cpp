/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2017 German Aerospace Center (DLR) and others.
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
/// @file    AGTime.cpp
/// @author  Piotr Woznica
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @author  Walter Bamberger
/// @date    July 2010
/// @version $Id$
///
// Time manager: able to manipulate the time using Sumo's format (seconds)
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "AGTime.h"


// ===========================================================================
// method definitions
// ===========================================================================
AGTime::AGTime(const AGTime& time) {
    sec = time.sec;
}

int
AGTime::convert(int days, int hours, int minutes, int seconds) {
    sec = seconds + 60 * (minutes + 60 * (hours + 24 * (days)));
    return sec;
}

int
AGTime::getSecondsOf(double minutes) {
    return static_cast<int>(60.0 * minutes);
}

bool
AGTime::operator==(const AGTime& time) {
    if (this->sec == time.sec) {
        return true;
    } else {
        return false;
    }
}

bool
AGTime::operator<(const AGTime& time) {
    if (this->sec < time.sec) {
        return true;
    } else {
        return false;
    }
}

bool
AGTime::operator<=(const AGTime& time) {
    if (this->sec <= time.sec) {
        return true;
    } else {
        return false;
    }
}

void
AGTime::operator+=(const AGTime& time) {
    this->sec += time.sec;
}

void
AGTime::operator+=(int seconds) {
    this->sec += seconds;
}

void
AGTime::operator-=(const AGTime& time) {
    this->sec -= time.sec;
}

AGTime
AGTime::operator+(const AGTime& time) {
    AGTime newtime(time.sec + this->sec);
    return newtime;
}

int
AGTime::getDay() {
    return (sec / 86400);
}

int
AGTime::getHour() {
    return ((sec / 3600) % 24);
}

int
AGTime::getMinute() {
    return ((sec / 60) % 60);
}

int
AGTime::getSecond() {
    return (sec % 60);
}

int
AGTime::getSecondsInCurrentDay() {
    return (sec % 86400);
}

int
AGTime::getTime() {
    return this->sec;
}

void
AGTime::setDay(int d) {
    if (0 <= d) {
        sec -= 86400 * getDay();
        sec += 86400 * d;
    }
}

void
AGTime::setHour(int h) {
    if (0 <= h && h < 24) {
        sec -= 3600 * getHour();
        sec += 3600 * h;
    }
}

void
AGTime::setMinute(int m) {
    if (0 <= m && m < 60) {
        sec -= 60 * getMinute();
        sec += 60 * m;
    }
}

void
AGTime::setSecond(int s) {
    if (0 <= s && s < 60) {
        sec -= getSecond();
        sec += s;
    }
}

void
AGTime::setTime(int sec) {
    this->sec = sec;
}

void
AGTime::addDays(int d) {
    sec += 86400 * d;
}

void
AGTime::addHours(int h) {
    sec += 3600 * h;
}

void
AGTime::addMinutes(int m) {
    sec += 60 * m;
}

void
AGTime::addSeconds(int s) {
    sec += s;
}

/****************************************************************************/
