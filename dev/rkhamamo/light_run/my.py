#!/usr/bin/env python

import os
import sys
import optparse

# we need to import some python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


from sumolib import checkBinary  # Checks for the binary in environ vars
import traci


def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options


# contains TraCI control loop
def run():

    step = 0
    while traci.simulation.getMinExpectedNumber() > 0: #when all route files have been exhausted
        traci.simulationStep() #increment sim step
        
        det_vehs = traci.inductionloop.getLastStepVehicleIDs("det_0")
        
        x, y = traci.vehicle.getPosition("veh1") #store x and y coordinates
        print x, y #show position on terminal
        traci.vehicle.setRogue("veh1") #make vehicle rogue
        
        if x > 230 and x < 270 and y > 380 and y < 420:
            traci.vehicle.setRogueException("veh1") #make vehicle stop
            
        step += 1

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
