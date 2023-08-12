#pragma once
#include <ompl/base/State.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <boost/functional/hash.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <unordered_map>
#include <unordered_set>

#include "neighbor.hpp"
#include "planresult.hpp"
#include <vector>



namespace Constants
{
    // [m] --- The minimum turning radius of the vehicle
    static float r = 3;
    static float deltat = 6.75 * 6 / 180.0 * M_PI;
    // [#] --- A movement cost penalty for turning (choosing non straight motion
    // primitives)
    static float penaltyTurning = 1.5;
    // [#] --- A movement cost penalty for reversing (choosing motion primitives >
    // 2)
    static float penaltyReversing = 2.0;
    // [#] --- A movement cost penalty for change of direction (changing from
    // primitives < 3 to primitives > 2)
    static float penaltyCOD = 2.0;

    static float penaltyHeadon=3.0;

    static float oneStepWeight = 0.3;
    //
    static float penaltyWait = 1.0;
    // map resolution
    static float mapResolution = 2.0;
    // change to set calcIndex resolution
    static float xyResolution = r * deltat;
    static float yawResolution = deltat;

    static float greedyBonus = 1.0;

    static float gammaHistory = 0.99;

    // width of car
    static float carWidth = 2.0;
    // distance from rear to vehicle front end
    static float LF = 2.0;
    // distance from rear to vehicle back end
    static float LB = 1.0;
    // obstacle default radius
    static float obsRadius = 1;
    // least time to wait for constraint
    static int constraintWaitTime = 2;

    static double BSTIEBREAKER = 1e-3;

    
                                                    

    // R = 3, 6.75 DEG
    std::vector<double> dyaw = {0, deltat, -deltat, 0, -deltat, deltat};
    std::vector<double> dx = {r * deltat, r *sin(deltat), r *sin(deltat),
                              -r *deltat, -r *sin(deltat), -r *sin(deltat)};
    std::vector<double> dy = {0, -r * (1 - cos(deltat)), r * (1 - cos(deltat)),
                              0, -r * (1 - cos(deltat)), r * (1 - cos(deltat))};

    static inline float normalizeHeadingRad(float t)
    {
        if (t < 0)
        {
            t = t - 2.f * M_PI * static_cast<int>(t / (2.f * M_PI));
            return 2.f * M_PI + t;
        }

        return t - 2.f * M_PI * static_cast<int>(t / (2.f * M_PI));
    }


} // namespace Constants

namespace libMultiRobotPlanning
{

    using libMultiRobotPlanning::Neighbor;
    using libMultiRobotPlanning::PlanResult;
    using namespace libMultiRobotPlanning;
    typedef ompl::base::SE2StateSpace::StateType OmplState;
    typedef boost::geometry::model::d2::point_xy<double> Point;
    typedef boost::geometry::model::segment<Point> Segment;

}