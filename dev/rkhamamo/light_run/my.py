#!/usr/bin/env python

import os, sys, optparse

# check that sumo load path is set, import python modules from tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

print "running SUMO for", sys.argv[1], "steps"

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

    
    for step in range(int(sys.argv[1])): #5000 steps
        
        #increment sim step
        traci.simulationStep() 
        
        #wait until rogue vehicle is deployed
        if step > 20: 
            #print rogue position to terminal
            print traci.vehicle.getPosition("veh1")
            #make vehicle stop at this intersection
            #x, y = traci.junction.getPosition("n2")
            #traci.vehicle.rogueNodeException("veh1", x, y)
            
        if step == 20:

            #traci.vehicle.rogueToggleFollowSpeed("veh1") 
            #traci.vehicle.rogueToggleFollowDistance("veh1") 
        
        #if step == 21:

            #traci.vehicle.rogueToggleFollowSpeed("veh1") 
            #traci.vehicle.rogueToggleFollowDistance("veh1") 
        
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
