#!/usr/bin/env python

import os, sys, optparse

# check that sumo load path is set, import python modules from tools directory
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
           
    for step in range(simRange):
        
        #increment sim step
        traci.simulationStep() 
        
        #wait until rogue vehicle is deployed
        #if step > 10: 
        #addVehicle('dummy', 'passenger', 'route1', '2to3_0', 10, -1)
        #traci.vehicle.add("car", "route1", typeID="passenger")
        #traci.vehicle.moveTo('veh2', '2to3_0', 44.7)
        
        potholeColors = [205,133,63,255]
        #potholeShape = [256.3,450]shape="256.3,450 253.7,450 253.7,453 256.3,453"
        if step == 2:
            #traci.vehicle.add("pothile", "route1")
            traci.vehicle.moveToXY('veh2', '2to3', 0, 254.85, 453)
            traci.vehicle.setSpeed('veh2', 0)
            traci.polygon.add('pothole_1', ([256.3,450],[253.7,450],[253.7,453],[256.3,453]), potholeColors, fill=True, polygonType='building', layer=2);
        
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
    
    #parse command line
    if len(sys.argv) > 1:
        simRange = int(sys.argv[1])
    else:
        simRange = 500000 #default value
           
    if len(sys.argv) > 2:
        rogueVehicle = sys.argv[2]
    else:
        rogueVehicle = 'veh1' #default value
        
    print "running SUMO for", simRange, "steps"
    print "operating on rogue vehicle:", rogueVehicle
    
    run()
