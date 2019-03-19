Ryan Hamamoto

edits:

src/utils/common/SUMOVehicleClass.h
	added SVC_ROGUE class
	
src/utils/common/SUMOVehicleClass.cpp
	added string converter for rogue class and shape
erCapacity(0), boardingDuration(500),
      loadingDuration(90000), width(1.8), height(1.5), shape(SVS_UNKNOWN), osgFile("car-normal-citrus.obj"),
src/utils/vehicle/SUMOVTypeParameter.cpp
	added switch case for SVC_ROGUE with:
	SVS_PASSENGER shape
	speedfactor of 2
	impatience of 1
	mingap of 0.5
	
src/guisim_main.cpp
	added print statement

src/microsim/MSVehicle.cpp
setSpeedMode changes:
	myRespectJunctionPriority
	myEmergencyBrakeRedLight
getSpeedMode returns var


microsim/lcmodels/MSLCM_LC2013
traci_testclient/TraCITestClient.cpp
