/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2002-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    MSLaneChangerSublane.cpp
/// @author  Jakob Erdmann
/// @author  Leonhard Luecken
/// @date    Oct 2015
/// @version $Id$
///
// Performs sub-lane changing of vehicles
/****************************************************************************/

// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "MSLaneChangerSublane.h"
#include "MSNet.h"
#include "MSVehicle.h"
#include "MSVehicleType.h"
#include "MSVehicleTransfer.h"
#include "MSGlobals.h"
#include <cassert>
#include <iterator>
#include <cstdlib>
#include <cmath>
#include <microsim/lcmodels/MSAbstractLaneChangeModel.h>
#include <utils/common/MsgHandler.h>
#include <utils/geom/GeomHelper.h>


// ===========================================================================
// DEBUG constants
// ===========================================================================
#define DEBUG_COND (vehicle->getLaneChangeModel().debugVehicle())
//#define DEBUG_COND (vehicle->getID() == "disabled")
//#define DEBUG_ACTIONSTEPS
//#define DEBUG_STATE
//#define DEBUG_MANEUVER
//#define DEBUG_SURROUNDING

// ===========================================================================
// member method definitions
// ===========================================================================
MSLaneChangerSublane::MSLaneChangerSublane(const std::vector<MSLane*>* lanes, bool allowChanging) :
    MSLaneChanger(lanes, allowChanging) {
    // initialize siblings
    if (myChanger.front().lane->isInternal()) {
        for (ChangerIt ce = myChanger.begin(); ce != myChanger.end(); ++ce) {
            for (ChangerIt ce2 = myChanger.begin(); ce2 != myChanger.end(); ++ce2) {
                if (ce != ce2 && ce->lane->getIncomingLanes().front().lane == ce2->lane->getIncomingLanes().front().lane) {
                    //std::cout << "addSibling lane=" << ce->lane->getID() << " offset=" << ce2->lane->getIndex() - ce->lane->getIndex() << "\n";
                    ce->siblings.push_back(ce2->lane->getIndex() - ce->lane->getIndex());
                }
            }
        }
    }
}


MSLaneChangerSublane::~MSLaneChangerSublane() {}

void
MSLaneChangerSublane::initChanger() {
    MSLaneChanger::initChanger();
    // Prepare myChanger with a safe state.
    for (ChangerIt ce = myChanger.begin(); ce != myChanger.end(); ++ce) {
        ce->ahead = ce->lane->getPartialBeyond();
//        std::cout << SIMTIME << " initChanger lane=" << ce->lane->getID() << " vehicles=" << toString(ce->lane->myVehicles) << "\n";
//        std::cout << SIMTIME << " initChanger lane=" << ce->lane->getID() << " partial vehicles=" << toString(ce->lane->myPartialVehicles) << "\n";
//        std::cout << SIMTIME << " initChanger lane=" << ce->lane->getID() << " partial vehicles beyond=" << toString(ce->ahead.toString()) << "\n";
    }
}



void
MSLaneChangerSublane::updateChanger(bool vehHasChanged) {
    MSLaneChanger::updateChanger(vehHasChanged);
    if (!vehHasChanged) {
        MSVehicle* lead = myCandi->lead;
        //std::cout << SIMTIME << " updateChanger lane=" << myCandi->lane->getID() << " lead=" << Named::getIDSecure(lead) << "\n";
        myCandi->ahead.addLeader(lead, false, 0);
        MSLane* shadowLane = lead->getLaneChangeModel().getShadowLane();
        if (shadowLane != 0) {
            const double latOffset = lead->getLane()->getRightSideOnEdge() - shadowLane->getRightSideOnEdge();
            //std::cout << SIMTIME << " updateChanger shadowLane=" << shadowLane->getID() << " lead=" << Named::getIDSecure(lead) << "\n";
            (myChanger.begin() + shadowLane->getIndex())->ahead.addLeader(lead, false, latOffset);
        }
    }
    //std::cout << SIMTIME << " updateChanger: lane=" << myCandi->lane->getID() << " lead=" << Named::getIDSecure(myCandi->lead) << " ahead=" << myCandi->ahead.toString() << " vehHasChanged=" << vehHasChanged << "\n";
    //for (ChangerIt ce = myChanger.begin(); ce != myChanger.end(); ++ce) {
    //    std::cout << " lane=" << ce->lane->getID() << " vehicles=" << toString(ce->lane->myVehicles) << "\n";
    //}
}


bool
MSLaneChangerSublane::change() {
    // variant of change() for the sublane case
    myCandi = findCandidate();
    MSVehicle* vehicle = veh(myCandi);
    if DEBUG_COND {
    std::cout << "\nCHANGE" << std::endl;
}
assert(vehicle->getLane() == (*myCandi).lane);
    assert(!vehicle->getLaneChangeModel().isChangingLanes());
    if (/*!myAllowsChanging || vehicle->getLaneChangeModel().alreadyChanged() ||*/ vehicle->isStoppedOnLane()) {
        registerUnchanged(vehicle);
        return false;
    }
#ifndef NO_TRACI
    if (vehicle->isRemoteControlled()) {
        registerUnchanged(vehicle);
        return false;
    }
#endif
    if (!vehicle->isActive()) {
#ifdef DEBUG_ACTIONSTEPS
        if DEBUG_COND {
        std::cout << SIMTIME << " veh '" << vehicle->getID() << "' skips regular change checks." << std::endl;
        }
#endif

        bool changed;
#ifndef NO_TRACI
        // let TraCI influence the wish to change lanes during non-actionsteps
        checkTraCICommands(vehicle);
#endif

        // Resume change
        changed = continueChangeSublane(vehicle, myCandi);
#ifdef DEBUG_ACTIONSTEPS
        if DEBUG_COND {
        std::cout << SIMTIME << " veh '" << vehicle->getID() << "' lcm->maneuverDist=" << vehicle->getLaneChangeModel().getManeuverDist()
            << " lcm->speedLat=" << vehicle->getLaneChangeModel().getSpeedLat() << std::endl;
        }
#endif
        return changed;
    }

#ifdef DEBUG_ACTIONSTEPS
    if DEBUG_COND {
    std::cout << "\n" << SIMTIME << " veh '" << vehicle->getID() << "' at plans sublane maneuver."
        << std::endl;
    }
#endif
    vehicle->updateBestLanes(); // needed?
    for (int i = 0; i < (int) myChanger.size(); ++i) {
        vehicle->adaptBestLanesOccupation(i, myChanger[i].dens);
    }
    // update leaders beyond the current edge for all lanes
    for (ChangerIt ce = myChanger.begin(); ce != myChanger.end(); ++ce) {
        ce->aheadNext = getLeaders(ce, vehicle);
    }

    // update expected speeds
    int sublaneIndex = 0;
    for (ChangerIt ce = myChanger.begin(); ce != myChanger.end(); ++ce) {
        vehicle->getLaneChangeModel().updateExpectedSublaneSpeeds(ce->aheadNext, sublaneIndex, ce->lane->getIndex());
        for (int offset : ce->siblings) {
            // treat sibling lanes (internal lanes with the same origin lane) as if they have the same geometry
            ChangerIt ceSib = ce + offset;
            vehicle->getLaneChangeModel().updateExpectedSublaneSpeeds(ceSib->aheadNext, sublaneIndex, ceSib->lane->getIndex());
        }
        sublaneIndex += ce->ahead.numSublanes();
    }

    LaneChangeAction alternatives = (LaneChangeAction)((mayChange(-1) ? LCA_RIGHT : LCA_NONE)
                                    | (mayChange(1) ? LCA_LEFT : LCA_NONE));

    StateAndDist right = checkChangeHelper(vehicle, -1, alternatives);
    StateAndDist left = checkChangeHelper(vehicle, 1, alternatives);
    StateAndDist current = checkChangeHelper(vehicle, 0, alternatives);

    StateAndDist decision = vehicle->getLaneChangeModel().decideDirection(current,
                            vehicle->getLaneChangeModel().decideDirection(right, left));
    if (vehicle->getLaneChangeModel().debugVehicle()) {
        std::cout << "\n" << SIMTIME << " decision=" << toString((LaneChangeAction)decision.state) << " dir=" << decision.dir << " latDist=" << decision.latDist << " maneuverDist=" << decision.maneuverDist << "\n";
    }
    vehicle->getLaneChangeModel().setOwnState(decision.state);
    vehicle->getLaneChangeModel().setManeuverDist(decision.maneuverDist);
    if ((decision.state & LCA_WANTS_LANECHANGE) != 0 && (decision.state & LCA_BLOCKED) == 0) {
        // change if the vehicle wants to and is allowed to change
#ifdef DEBUG_MANEUVER
        if DEBUG_COND {
        std::cout << SIMTIME << " veh '" << vehicle->getID() << "' performing sublane change..." << std::endl;
        }
#endif
        return startChangeSublane(vehicle, myCandi, decision.latDist);
    }
    // @note this assumes vehicles can instantly abort any maneuvre in case of emergency
    abortLCManeuver(vehicle);

    if ((right.state & (LCA_URGENT)) != 0 && (left.state & (LCA_URGENT)) != 0) {
        // ... wants to go to the left AND to the right
        // just let them go to the right lane...
        left.state = 0;
    }
    return false;
}


void
MSLaneChangerSublane::abortLCManeuver(MSVehicle* vehicle) {
#ifdef DEBUG_MANEUVER
    if DEBUG_COND {
    std::cout << SIMTIME << " veh '" << vehicle->getID() << "' aborts LC-continuation."
        << std::endl;
    }
#endif
    vehicle->getLaneChangeModel().setSpeedLat(0);
    vehicle->getLaneChangeModel().setManeuverDist(0.);
    registerUnchanged(vehicle);
}


MSLaneChangerSublane::StateAndDist
MSLaneChangerSublane::checkChangeHelper(MSVehicle* vehicle, int laneOffset, LaneChangeAction alternatives) {
    StateAndDist result = StateAndDist(0, 0, 0, 0);
    if (mayChange(laneOffset)) {
        const std::vector<MSVehicle::LaneQ>& preb = vehicle->getBestLanes();
        result.state = checkChangeSublane(laneOffset, alternatives, preb, result.latDist, result.maneuverDist);
        result.dir = laneOffset;
        if ((result.state & LCA_WANTS_LANECHANGE) != 0 && (result.state & LCA_URGENT) != 0 && (result.state & LCA_BLOCKED) != 0) {
            (myCandi + laneOffset)->lastBlocked = vehicle;
            if ((myCandi + laneOffset)->firstBlocked == 0) {
                (myCandi + laneOffset)->firstBlocked = vehicle;
            }
        }
    }
    return result;
}


///  @brief Continue a sublane-lane change maneuver and return whether the midpoint was passed in this step
//          (used to continue sublane changing in non-action steps).
bool
MSLaneChangerSublane::continueChangeSublane(MSVehicle* vehicle, ChangerIt& from) {
    // lateral distance to complete maneuver
    double remLatDist = vehicle->getLaneChangeModel().getManeuverDist();
    if (remLatDist == 0) {
        registerUnchanged(vehicle);
        return false;
    }
    const double nextLatDist = SPEED2DIST(vehicle->getLaneChangeModel().computeSpeedLat(remLatDist, remLatDist));
#ifdef DEBUG_MANEUVER
    if DEBUG_COND {
    std::cout << SIMTIME << " vehicle '" << vehicle->getID() << "' continueChangeSublane()"
        << " remLatDist=" << remLatDist << " nextLatDist=" << nextLatDist
        << std::endl;
    }
#endif

    const bool changed = startChangeSublane(vehicle, from, nextLatDist);
    return changed;
}


bool
MSLaneChangerSublane::startChangeSublane(MSVehicle* vehicle, ChangerIt& from, double latDist) {
    // Prevent continuation of LC beyond lane borders if change is not allowed
    const double distToRightLaneBorder = latDist < 0 ? vehicle->getLane()->getWidth() * 0.5 + vehicle->getLateralPositionOnLane() - vehicle->getWidth() * 0.5 : 0.;
    const double distToLeftLaneBorder = latDist > 0 ? vehicle->getLane()->getWidth() * 0.5 - vehicle->getLateralPositionOnLane() - vehicle->getWidth() * 0.5 : 0.;
    // determine direction of LC
    const int direction = (latDist >= -distToRightLaneBorder && latDist <= distToLeftLaneBorder) ? 0 : (latDist < 0 ? -1 : 1);
    ChangerIt to = from;
    if (mayChange(direction)) {
        to = from + direction;
    } else {
        // This may occur during maneuver continuation in non-actionsteps.
        // TODO: Understand better why and test later if additional sublane actionstep debugging resolves this
        // (XXX: perhaps one should try to extrapolate check for this case before to avoid maneuver initialization
        //       similar as for continuous LC in MSLaneChanger::checkChange())
        //assert(false);
        abortLCManeuver(vehicle);
        return false;
    }

    // The following does:
    // 1) update vehicles lateral position according to latDist and target lane
    // 2) distinguish several cases
    //   a) vehicle moves completely within the same lane
    //   b) vehicle intersects another lane
    //      - vehicle must be moved to the lane where it's midpoint is (either old or new)
    //      - shadow vehicle must be created/moved to the other lane if the vehicle intersects it
    // 3) updated dens of all lanes that hold the vehicle or its shadow

    vehicle->myState.myPosLat += latDist;
    vehicle->myCachedPosition = Position::INVALID;
    vehicle->getLaneChangeModel().setSpeedLat(DIST2SPEED(latDist));
#ifdef DEBUG_MANEUVER
    if DEBUG_COND {
    std::cout << SIMTIME << " vehicle '" << vehicle->getID() << "' with maneuverDist=" << vehicle->getLaneChangeModel().getManeuverDist()
        << " and committedSpeed=" << vehicle->getLaneChangeModel().getCommittedSpeed()
        << " increments lateral position by latDist=" << latDist << std::endl;
    }
#endif
#ifdef DEBUG_SURROUNDING
    if DEBUG_COND {
    std::cout << SIMTIME << " vehicle '" << vehicle->getID() << "'\n    to->ahead=" << to->ahead.toString()
        << "'\n    to->aheadNext=" << to->aheadNext.toString()
        << std::endl;
    }
#endif
    const bool completedManeuver = vehicle->getLaneChangeModel().getManeuverDist() - latDist == 0.;
    vehicle->getLaneChangeModel().setManeuverDist(vehicle->getLaneChangeModel().getManeuverDist() - latDist);
    vehicle->getLaneChangeModel().updateSafeLatDist(latDist);

    outputLCStarted(vehicle, from, to, direction);
    const bool changedToNewLane = checkChangeToNewLane(vehicle, direction, from, to);

    MSLane* oldShadowLane = vehicle->getLaneChangeModel().getShadowLane();
    vehicle->getLaneChangeModel().updateShadowLane();
    MSLane* shadowLane = vehicle->getLaneChangeModel().getShadowLane();
    if (shadowLane != 0 && shadowLane != oldShadowLane) {
        assert(to != from || oldShadowLane == 0);
        const double latOffset = vehicle->getLane()->getRightSideOnEdge() - shadowLane->getRightSideOnEdge();
        (myChanger.begin() + shadowLane->getIndex())->ahead.addLeader(vehicle, false, latOffset);
    }
    if (completedManeuver) {
        outputLCEnded(vehicle, from, to, direction);
    }

    // Update maneuver reservations on target lanes
    vehicle->getLaneChangeModel().updateTargetLane();

    // compute new angle of the vehicle from the x- and y-distances travelled within last time step
    // (should happen last because primaryLaneChanged() also triggers angle computation)
    // this part of the angle comes from the orientation of our current lane
    double laneAngle = vehicle->getLane()->getShape().rotationAtOffset(vehicle->getLane()->interpolateLanePosToGeometryPos(vehicle->getPositionOnLane())) ;
    // this part of the angle comes from the vehicle's lateral movement
    double changeAngle = 0;
    // avoid flicker
    if (fabs(latDist) > NUMERICAL_EPS) {
        // angle is between vehicle front and vehicle back (and depending on travelled distance)
        changeAngle = atan2(latDist, vehicle->getVehicleType().getLength() + SPEED2DIST(vehicle->getSpeed()));
    }
#ifdef DEBUG_MANEUVER
    if (vehicle->getLaneChangeModel().debugVehicle()) {
        MSLane* targetLane = vehicle->getLaneChangeModel().getTargetLane();
        std::cout << SIMTIME << " startChangeSublane()"
                  << " shadowLane=" << (shadowLane != nullptr ? shadowLane->getID() : "NULL")
                  << " targetLane=" << (targetLane != nullptr ? targetLane->getID() : "NULL")
                  << " maneuverDist=" << vehicle->getLaneChangeModel().getManeuverDist()
                  << " latDist=" << latDist
                  << " old=" << Named::getIDSecure(oldShadowLane)
                  << " new=" << Named::getIDSecure(vehicle->getLaneChangeModel().getShadowLane())
                  << " laneA=" << RAD2DEG(laneAngle)
                  << " changeA=" << RAD2DEG(changeAngle)
                  << " oldA=" << RAD2DEG(vehicle->getAngle())
                  << " newA=" << RAD2DEG(laneAngle + changeAngle)
                  << "\n";
    }
#endif
    vehicle->setAngle(laneAngle + changeAngle, completedManeuver);

    // check if a traci maneuver must continue
    if ((vehicle->getLaneChangeModel().getOwnState() & LCA_TRACI) != 0) {
        if (vehicle->getLaneChangeModel().debugVehicle()) {
            std::cout << SIMTIME << " continue TraCI-maneuver remainingLatDist=" << vehicle->getLaneChangeModel().getManeuverDist() << "\n";
        }
        vehicle->getInfluencer().setSublaneChange(vehicle->getLaneChangeModel().getManeuverDist());
    }
    return changedToNewLane;
}

bool
MSLaneChangerSublane::checkChangeToNewLane(MSVehicle* vehicle, const int direction, ChangerIt from, ChangerIt to) {
    const bool changedToNewLane = to != from && fabs(vehicle->getLateralPositionOnLane()) > 0.5 * vehicle->getLane()->getWidth() && mayChange(direction);
    if (changedToNewLane) {
        vehicle->myState.myPosLat -= direction * 0.5 * (from->lane->getWidth() + to->lane->getWidth());
        to->lane->myTmpVehicles.insert(to->lane->myTmpVehicles.begin(), vehicle);
        to->dens += vehicle->getVehicleType().getLengthWithGap();
        if (MSAbstractLaneChangeModel::haveLCOutput()) {
            if (!vehicle->isActive()) {
                // update leaders beyond the current edge for all lanes
                // @note to->aheadNext and from->aheadNext are only needed for output in non-action steps.
                to->aheadNext = getLeaders(to, vehicle);
                from->aheadNext = getLeaders(from, vehicle);
            }
            vehicle->getLaneChangeModel().setLeaderGaps(to->aheadNext);
            vehicle->getLaneChangeModel().setFollowerGaps(to->lane->getFollowersOnConsecutive(vehicle, vehicle->getBackPositionOnLane(), true));
            vehicle->getLaneChangeModel().setOrigLeaderGaps(from->aheadNext);
        }
        vehicle->getLaneChangeModel().startLaneChangeManeuver(from->lane, to->lane, direction);
        to->ahead.addLeader(vehicle, false, 0);
    } else {
        registerUnchanged(vehicle);
        from->ahead.addLeader(vehicle, false, 0);
    }
    return changedToNewLane;
}

void
MSLaneChangerSublane::outputLCStarted(MSVehicle* vehicle, ChangerIt& from, ChangerIt& to, int direction) {
    if (MSAbstractLaneChangeModel::haveLCOutput() && MSAbstractLaneChangeModel::outputLCStarted()
            // non-sublane change started
            && ((vehicle->getLaneChangeModel().getOwnState() & (LCA_CHANGE_REASONS & ~LCA_SUBLANE)) != 0)
            && ((vehicle->getLaneChangeModel().getOwnState() & LCA_STAY) == 0)
            // no changing in previous step (either not wanted or blocked)
            && (((vehicle->getLaneChangeModel().getPrevState() & (LCA_CHANGE_REASONS & ~LCA_SUBLANE)) == 0)
                || ((vehicle->getLaneChangeModel().getPrevState() & LCA_STAY) != 0)
                || ((vehicle->getLaneChangeModel().getPrevState() & LCA_BLOCKED) != 0))
       ) {
#ifdef DEBUG_STATE
        if DEBUG_COND {
        std::cout << SIMTIME << " veh=" << vehicle->getID() << " laneChangeStarted state=" << toString((LaneChangeAction)vehicle->getLaneChangeModel().getOwnState())
            << " prevState=" << toString((LaneChangeAction)vehicle->getLaneChangeModel().getPrevState())
            << " filter=" << toString((LaneChangeAction)(LCA_CHANGE_REASONS & ~LCA_SUBLANE))
            << " filtered=" << toString((LaneChangeAction)(vehicle->getLaneChangeModel().getOwnState() & (LCA_CHANGE_REASONS & ~LCA_SUBLANE)))
            << "\n";
        }
#endif
        vehicle->getLaneChangeModel().setLeaderGaps(to->aheadNext);
        vehicle->getLaneChangeModel().setFollowerGaps(to->lane->getFollowersOnConsecutive(vehicle, vehicle->getBackPositionOnLane(), true));
        vehicle->getLaneChangeModel().setOrigLeaderGaps(from->aheadNext);
        vehicle->getLaneChangeModel().laneChangeOutput("changeStarted", from->lane, to->lane, direction);
    }
}

void
MSLaneChangerSublane::outputLCEnded(MSVehicle* vehicle, ChangerIt& from, ChangerIt& to, int direction) {
    if (MSAbstractLaneChangeModel::haveLCOutput() && MSAbstractLaneChangeModel::outputLCEnded()
            // non-sublane change ended
            && ((vehicle->getLaneChangeModel().getOwnState() & (LCA_CHANGE_REASONS & ~LCA_SUBLANE)) != 0)) {
        vehicle->getLaneChangeModel().setLeaderGaps(to->aheadNext);
        vehicle->getLaneChangeModel().setFollowerGaps(to->lane->getFollowersOnConsecutive(vehicle, vehicle->getBackPositionOnLane(), true));
        vehicle->getLaneChangeModel().setOrigLeaderGaps(from->aheadNext);
        vehicle->getLaneChangeModel().laneChangeOutput("changeEnded", from->lane, to->lane, direction);
    }
}


MSLeaderDistanceInfo
MSLaneChangerSublane::getLeaders(const ChangerIt& target, const MSVehicle* vehicle) const {
    // get the leading vehicle on the lane to change to
#ifdef DEBUG_SURROUNDING
    if (DEBUG_COND) {
        std::cout << SIMTIME << " getLeaders lane=" << target->lane->getID() << " ego=" << vehicle->getID() << " ahead=" << target->ahead.toString() << "\n";
    }
#endif
    MSLeaderDistanceInfo result(target->lane, 0, 0);
    for (int i = 0; i < target->ahead.numSublanes(); ++i) {
        const MSVehicle* veh = target->ahead[i];
        if (veh != 0) {
            const double gap = veh->getBackPositionOnLane(target->lane) - vehicle->getPositionOnLane() - vehicle->getVehicleType().getMinGap();
#ifdef DEBUG_SURROUNDING
            if (DEBUG_COND) {
                std::cout << " ahead lead=" << veh->getID() << " leadBack=" << veh->getBackPositionOnLane() << " gap=" << gap << "\n";
            }
#endif
            result.addLeader(veh, gap, 0, i);
        }
    }
    // if there are vehicles on the target lane with the same position as ego,
    // they may not have been added to 'ahead' yet
    const MSLeaderInfo& aheadSamePos = target->lane->getLastVehicleInformation(0, 0, vehicle->getPositionOnLane(), false);
    for (int i = 0; i < aheadSamePos.numSublanes(); ++i) {
        const MSVehicle* veh = aheadSamePos[i];
        if (veh != 0 && veh != vehicle) {
            const double gap = veh->getBackPositionOnLane(target->lane) - vehicle->getPositionOnLane() - vehicle->getVehicleType().getMinGap();
#ifdef DEBUG_SURROUNDING
            if (DEBUG_COND) {
                std::cout << " further lead=" << veh->getID() << " leadBack=" << veh->getBackPositionOnLane(target->lane) << " gap=" << gap << "\n";
            }
#endif
            result.addLeader(veh, gap, 0, i);
        }
    }

    if (result.numFreeSublanes() > 0) {
        MSLane* targetLane = target->lane;

        double seen = vehicle->getLane()->getLength() - vehicle->getPositionOnLane();
        double speed = vehicle->getSpeed();
        double dist = vehicle->getCarFollowModel().brakeGap(speed) + vehicle->getVehicleType().getMinGap();
        if (seen > dist) {
#ifdef DEBUG_SURROUNDING
            if (DEBUG_COND) {
                std::cout << " aborting forward search. dist=" << dist << " seen=" << seen << "\n";
            }
#endif
            return result;
        }
        const std::vector<MSLane*>& bestLaneConts = veh(myCandi)->getBestLanesContinuation(targetLane);
#ifdef DEBUG_SURROUNDING
        if (DEBUG_COND) {
            std::cout << " add consecutive before=" << result.toString() << " dist=" << dist;
        }
#endif
        target->lane->getLeadersOnConsecutive(dist, seen, speed, vehicle, bestLaneConts, result);
#ifdef DEBUG_SURROUNDING
        if (DEBUG_COND) {
            std::cout << " after=" << result.toString() << "\n";
        }
#endif
    }
    return result;
}


int
MSLaneChangerSublane::checkChangeSublane(
    int laneOffset,
    LaneChangeAction alternatives,
    const std::vector<MSVehicle::LaneQ>& preb,
    double& latDist,
    double& maneuverDist) const {

    ChangerIt target = myCandi + laneOffset;
    MSVehicle* vehicle = veh(myCandi);
    const MSLane& neighLane = *(target->lane);
    int blocked = 0;

    MSLeaderDistanceInfo neighLeaders = target->aheadNext;
    MSLeaderDistanceInfo neighFollowers = target->lane->getFollowersOnConsecutive(vehicle, vehicle->getBackPositionOnLane(), true);
    MSLeaderDistanceInfo neighBlockers(&neighLane, vehicle, vehicle->getLane()->getRightSideOnEdge() - neighLane.getRightSideOnEdge());
    MSLeaderDistanceInfo leaders = myCandi->aheadNext;
    MSLeaderDistanceInfo followers = myCandi->lane->getFollowersOnConsecutive(vehicle, vehicle->getBackPositionOnLane(), true);
    MSLeaderDistanceInfo blockers(vehicle->getLane(), vehicle, 0);

#ifdef DEBUG_SURROUNDING
    if (DEBUG_COND) std::cout << SIMTIME
                                  << " checkChangeSublane: veh=" << vehicle->getID()
                                  << " laneOffset=" << laneOffset
                                  << "\n  leaders=" << leaders.toString()
                                  << "\n  neighLeaders=" << neighLeaders.toString()
                                  << "\n  followers=" << followers.toString()
                                  << "\n  neighFollowers=" << neighFollowers.toString()
                                  << "\n";
#endif


    const int wish = vehicle->getLaneChangeModel().wantsChangeSublane(
                         laneOffset, alternatives,
                         leaders, followers, blockers,
                         neighLeaders, neighFollowers, neighBlockers,
                         neighLane, preb,
                         &(myCandi->lastBlocked), &(myCandi->firstBlocked), latDist, maneuverDist, blocked);
    int state = blocked | wish;

    // XXX
    // do are more careful (but expensive) check to ensure that a
    // safety-critical leader is not being overlooked

    // XXX
    // ensure that a continuous lane change manoeuvre can be completed
    // before the next turning movement

#ifndef NO_TRACI
    // let TraCI influence the wish to change lanes and the security to take
    const int oldstate = state;
    state = vehicle->influenceChangeDecision(state);
#ifdef DEBUG_STATE
    if (DEBUG_COND && state != oldstate) {
        std::cout << SIMTIME << " veh=" << vehicle->getID() << " stateAfterTraCI=" << toString((LaneChangeAction)state) << " original=" << toString((LaneChangeAction)oldstate) << "\n";
    }
#endif
#endif
    vehicle->getLaneChangeModel().saveState(laneOffset, oldstate, state);
    return state;
}

/****************************************************************************/

