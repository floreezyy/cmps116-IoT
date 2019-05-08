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
/// @file    FXWorkerThreadTest.cpp
/// @author  Michael Behrisch
/// @date    Oct 2010
/// @version $Id$
///
// Tests the class FXWorkerThread
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <gtest/gtest.h>
#include <utils/common/StdDefs.h>
#include <utils/foxtools/FXWorkerThread.h>

class TestTask : public FXWorkerThread::Task {
public:
    void run(FXWorkerThread* /* context */) {
    }
};

// ===========================================================================
// test definitions
// ===========================================================================
/* Test the initialization.*/
TEST(FXWorkerThread, test_init) {
    FXWorkerThread::Pool g(4);
}

/* Test retrieving all tasks.*/
TEST(FXWorkerThread, test_get_all) {
    FXWorkerThread::Pool g(4);
    FXWorkerThread::Task* task1 = new TestTask();
    FXWorkerThread::Task* task2 = new TestTask();
    FXWorkerThread::Task* task3 = new TestTask();
    FXWorkerThread::Task* task4 = new TestTask();
    g.add(task1);
    g.add(task2);
    g.add(task3);
    g.add(task4);
    g.waitAll();
}

