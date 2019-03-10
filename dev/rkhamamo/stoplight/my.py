#!/usr/bin/env python

import os
import sys
import optparse

#import tools modules from sumo home directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary
import traci

def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true", default=False, help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options


#run TraCI sim controller
def run():

    for step in range(5000): #5000 steps
        traci.simulationStep() #increment sim
        #traci.vehicle.setSpeedMode("veh1", 7); #vehicle 1 runs red lights
        det_vehicles = traci.inductionloop.getLastStepVehicleIDs("det_0")
        for vehicle in det_vehicles:
            print(vehicle)
            traci.vehicle.changeLane(vehicle, 1, 25) #switch lanes

        if step == 100:
            traci.vehicle.setSpeed("veh2", 0); #stop vehicle 2

    traci.close()
    sys.stdout.flush()


# main entry point
if __name__ == "__main__":
    options = get_options()

    # check binary
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # traci starts sumo as a subprocess and then this script connects and runs
    traci.start([sumoBinary, "-c", "my_config.sumocfg",
                             "--tripinfo-output", "tripinfo.xml"])
    run()
