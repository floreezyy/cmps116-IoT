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


#allow user to run sumo cmd line without visualization tool
def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options


# contains TraCI control loop
def run():

    for step in range(5000): #5000 steps
        #increment sim step
        traci.simulationStep() 
        
        if step > 20:
            #print rogue position to terminal
            print traci.vehicle.getPosition("veh1") 

            #det_vehs = traci.inductionloop.getLastStepVehicleIDs("det_0")

            #traci.vehicle.rogueFollowSpeed("veh1") 

            #make vehicle stop at this intersection
            #x, y = traci.junction.getPosition("n2")
            #traci.vehicle.setRogueNodeException("veh1", x, y)
        
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
