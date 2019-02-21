/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2016-2017 German Aerospace Center (DLR) and others.
// PHEMlight module
// Copyright 2016 Technische Universitaet Graz, https://www.tugraz.at/
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    CEPHandler.h
/// @author  Martin Dippold
/// @author  Michael Behrisch
/// @date    July 2016
/// @version $Id$
///
//
/****************************************************************************/


#ifndef PHEMlightCEPHANDLER
#define PHEMlightCEPHANDLER

#include <string>
#include <map>
#include <vector>
#include <utility>

//C# TO C++ CONVERTER NOTE: Forward class declarations:
namespace PHEMlightdll { class CEP; }
namespace PHEMlightdll { class Helpers; }


namespace PHEMlightdll {
    class CEPHandler {
        //--------------------------------------------------------------------------------------------------
        // Constructors
        //--------------------------------------------------------------------------------------------------

    public:
        CEPHandler();

        //--------------------------------------------------------------------------------------------------
        // Members 
        //--------------------------------------------------------------------------------------------------

    private:
        std::map<std::string, CEP*> _ceps;
    public:
        const std::map<std::string, CEP*>& getCEPS() const;


        //--------------------------------------------------------------------------------------------------
        // Methods 
        //--------------------------------------------------------------------------------------------------

        bool GetCEP(const std::vector<std::string>& DataPath, Helpers* Helper);


        //--------------------------------------------------------------------------------------------------
        // Methods 
        //--------------------------------------------------------------------------------------------------

    private:
        bool Load(const std::vector<std::string>& DataPath, Helpers* Helper);

        bool ReadVehicleFile(const std::vector<std::string>& DataPath, const std::string& emissionClass, Helpers* Helper, double& vehicleMass, double& vehicleLoading, double& vehicleMassRot, double& crossArea, double& cWValue, double& f0, double& f1, double& f2, double& f3, double& f4, double& axleRatio, double& auxPower, double& ratedPower, double& engineIdlingSpeed, double& engineRatedSpeed, double& effectiveWheelDiameter, std::vector<double>& transmissionGearRatios, std::string& vehicleMassType, std::string& vehicleFuelType, double& pNormV0, double& pNormP0, double& pNormV1, double& pNormP1, std::vector<std::vector<double> >& matrixSpeedInertiaTable, std::vector<std::vector<double> >& normedDragTable);

        bool ReadEmissionData(bool readFC, const std::vector<std::string>& DataPath, const std::string& emissionClass, Helpers* Helper, std::vector<std::string>& header, std::vector<std::vector<double> >& matrix, std::vector<double>& idlingValues);


        //--------------------------------------------------------------------------------------------------
        // Functions 
        //--------------------------------------------------------------------------------------------------

        //Split the string
        std::vector<std::string> split(const std::string& s, char delim);

        //Convert string to double
        double todouble(const std::string& s);

        //Convert string to double list
        std::vector<double> todoubleList(const std::vector<std::string>& s);

        //Read a line from file
        std::string ReadLine(std::ifstream& s);
    };
}


#endif	//#ifndef PHEMlightCEPHANDLER
