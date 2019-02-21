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
/// @file    MSVehicleContainer.h
/// @author  Christian Roessel
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Mon, 12 Mar 2001
/// @version $Id$
///
// vehicles sorted by their departures
/****************************************************************************/
#ifndef MSVehicleContainer_h
#define MSVehicleContainer_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <vector>
#include <iostream>


// ===========================================================================
// class declarations
// ===========================================================================
class MSVehicle;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSVehicleContainer
 * A storage for vehicles, mainly used by the vehicle/route loading structures
 * and the insertion control. Stores vehicles in a heap of vehiclevector/departure-
 * pairs.
 */
class MSVehicleContainer {
public:
    /// definition of a list of vehicles which have the same departure time
    typedef std::vector<SUMOVehicle*> VehicleVector;

    /** definition of a structure storing the departure time and a list
        of vehicles leaving at this time */
    typedef std::pair<SUMOTime, VehicleVector> VehicleDepartureVector;

public:
    /// Constructor
    MSVehicleContainer(int capacity = 10);

    /// Destructor
    ~MSVehicleContainer();

    /// Adds a single vehicle
    void add(SUMOVehicle* veh);

    /// Removes a single vehicle
    void remove(SUMOVehicle* veh);

    /// Adds a container with vehicles departing at the given time
    void add(SUMOTime time, const VehicleVector& cont);

    /// Returns the information whether any vehicles want to depart before the given time
    bool anyWaitingBefore(SUMOTime time) const;

    /// Returns the uppermost vehicle vector
    const VehicleVector& top();

    /// Returns the time the uppermost vehicle vector is assigned to
    SUMOTime topTime() const;

    /// Removes the uppermost vehicle vector
    void pop();

    /// Returns the information whether the container is empty
    bool isEmpty() const;

    /// Returns the size of the container
    int size() const;

    /// Prints the container (the departure times)
    void showArray() const;

    /// Prints the contents of the container
    friend std::ostream& operator << (std::ostream& strm,
                                      MSVehicleContainer& cont);

private:
    /** @brief Replaces the existing single departure time vector by the one given
    */
    void addReplacing(const VehicleDepartureVector& cont);

    /** Returns the information whether the container must be extended */
    bool isFull() const;

    /// Sort-criterion for vehicle departure lists
    class VehicleDepartureVectorSortCrit {
    public:
        /// comparison operator
        bool operator()(const VehicleDepartureVector& e1,
                        const VehicleDepartureVector& e2) const;
    };

    /// Searches for the VehicleDepartureVector with the wished depart
    class DepartFinder {
    public:
        /// constructor
        explicit DepartFinder(SUMOTime time);

        /// comparison operator
        bool operator()(const VehicleDepartureVector& e) const;

    private:
        /// the searched departure time
        SUMOTime myTime;
    };

    /// Number of elements in heap
    int currentSize;

    /// Definition of the heap type
    typedef std::vector<VehicleDepartureVector> VehicleHeap;

    /// The vehicle vector heap
    VehicleHeap array;

    /// Moves the elements down
    void percolateDown(int hole);

};


#endif

/****************************************************************************/

