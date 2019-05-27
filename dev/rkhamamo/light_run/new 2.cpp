bool
MSVehicle::ignoreRed(const MSLink* link, bool canBrake) const {
    if(getVehicleType().getID() == "Rogue")
    {
        std::cout << "RED";
        return true; //run red lights if you are rogue
    }
    if ((myInfluencer != 0 && !myInfluencer->getEmergencyBrakeRedLight())) {
        return true;
    }
    
    const double ignoreRedTime = getVehicleType().getParameter().getJMParam(SUMO_ATTR_JM_DRIVE_AFTER_RED_TIME, -1);
#ifdef DEBUG_IGNORE_RED
    if (DEBUG_COND) {
        std::cout << SIMTIME << " veh=" << getID() << " link=" << link->getViaLaneOrLane()->getID() << " state=" << toString(link->getState()) << "\n";
    }
#endif
    if (ignoreRedTime < 0) {
        return false;
    } else if (link->haveYellow()) {
        // always drive at yellow when ignoring red
        return true;
    } else if (link->haveRed()) {
        assert(link->getTLLogic() != 0);
        const double redDuration = STEPS2TIME(MSNet::getInstance()->getCurrentTimeStep() - link->getLastStateChange());
#ifdef DEBUG_IGNORE_RED
        if (DEBUG_COND) {
            std::cout
            // << SIMTIME << " veh=" << getID() << " link=" << link->getViaLaneOrLane()->getID()
                    << "   ignoreRedTime=" << ignoreRedTime
                    << " spentRed=" << redDuration
                    << " canBrake=" << canBrake << "\n";
        }
#endif
        // when activating ignoreRed behavior, vehicles will always drive if they cannot brake
        return !canBrake || ignoreRedTime > redDuration;
    } else {
        return false;
    }
}