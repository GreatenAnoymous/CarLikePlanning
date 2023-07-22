#include "pibt.hpp"
#include <fstream>
#include <iostream>
#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include "config.hpp"
#include "timer.hpp"
#include <boost/numeric/ublas/matrix.hpp>

using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;

struct Location
{
    Location(double x, double y) : x(x), y(y) {}
    double x;
    double y;

    bool operator<(const Location &other) const
    {
        return std::tie(x, y) < std::tie(other.x, other.y);
    }

    bool operator==(const Location &other) const
    {
        return std::tie(x, y) == std::tie(other.x, other.y);
    }

    friend std::ostream &operator<<(std::ostream &os, const Location &c)
    {
        return os << "(" << c.x << "," << c.y << ")";
    }
};

namespace std
{
    template <>
    struct hash<Location>
    {
        size_t operator()(const Location &s) const
        {
            size_t seed = 0;
            boost::hash_combine(seed, s.x);
            boost::hash_combine(seed, s.y);
            return seed;
        }
    };
} // namespace std

struct State
{
    State(double x, double y, double yaw, int time = 0)
        : time(time), x(x), y(y), yaw(yaw)
    {
        rot.resize(2, 2);
        rot(0, 0) = cos(-this->yaw);
        rot(0, 1) = -sin(-this->yaw);
        rot(1, 0) = sin(-this->yaw);
        rot(1, 1) = cos(-this->yaw);
#ifdef PRCISE_COLLISION
        corner1 = Point(
            this->x -
                sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                     pow(Constants::LB * 1.1, 2)) *
                    cos(atan2(Constants::carWidth / 2, Constants::LB) - this->yaw),
            this->y -
                sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                     pow(Constants::LB * 1.1, 2)) *
                    sin(atan2(Constants::carWidth / 2, Constants::LB) - this->yaw));
        corner2 = Point(
            this->x -
                sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                     pow(Constants::LB * 1.1, 2)) *
                    cos(atan2(Constants::carWidth / 2, Constants::LB) + this->yaw),
            this->y +
                sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                     pow(Constants::LB * 1.1, 2)) *
                    sin(atan2(Constants::carWidth / 2, Constants::LB) + this->yaw));
        corner3 = Point(
            this->x +
                sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                     pow(Constants::LF * 1.1, 2)) *
                    cos(atan2(Constants::carWidth / 2, Constants::LF) - this->yaw),
            this->y +
                sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                     pow(Constants::LF * 1.1, 2)) *
                    sin(atan2(Constants::carWidth / 2, Constants::LF) - this->yaw));
        corner4 = Point(
            this->x +
                sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                     pow(Constants::LF * 1.1, 2)) *
                    cos(atan2(Constants::carWidth / 2, Constants::LF) + this->yaw),
            this->y -
                sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                     pow(Constants::LF * 1.1, 2)) *
                    sin(atan2(Constants::carWidth / 2, Constants::LF) + this->yaw));
#endif
    }

    State() = default;

    bool operator==(const State &s) const
    {
        return std::tie(time, x, y, yaw) == std::tie(s.time, s.x, s.y, s.yaw);
    }

    bool agentCollision(const State &other) const
    {
#ifndef PRCISE_COLLISION
        if (pow(this->x - other.x, 2) + pow(this->y - other.y, 2) <
            pow(2 * Constants::LF, 2) + pow(Constants::carWidth, 2))
            return true;
        return false;
#else
        std::vector<Segment> rectangle1{Segment(this->corner1, this->corner2),
                                        Segment(this->corner2, this->corner3),
                                        Segment(this->corner3, this->corner4),
                                        Segment(this->corner4, this->corner1)};
        std::vector<Segment> rectangle2{Segment(other.corner1, other.corner2),
                                        Segment(other.corner2, other.corner3),
                                        Segment(other.corner3, other.corner4),
                                        Segment(other.corner4, other.corner1)};
        for (auto seg1 = rectangle1.begin(); seg1 != rectangle1.end(); seg1++)
            for (auto seg2 = rectangle2.begin(); seg2 != rectangle2.end(); seg2++)
            {
                if (boost::geometry::intersects(*seg1, *seg2))
                    return true;
            }
        return false;
#endif
    }

    bool obsCollision(const Location &obstacle) const
    {
        boost::numeric::ublas::matrix<double> obs(1, 2);
        obs(0, 0) = obstacle.x - this->x;
        obs(0, 1) = obstacle.y - this->y;

        auto rotated_obs = boost::numeric::ublas::prod(obs, rot);
        if (rotated_obs(0, 0) > -Constants::LB - Constants::obsRadius &&
            rotated_obs(0, 0) < Constants::LF + Constants::obsRadius &&
            rotated_obs(0, 1) > -Constants::carWidth / 2.0 - Constants::obsRadius &&
            rotated_obs(0, 1) < Constants::carWidth / 2.0 + Constants::obsRadius)
            return true;
        return false;
    }

    friend std::ostream &operator<<(std::ostream &os, const State &s)
    {
        return os << "(" << s.x << "," << s.y << ":" << s.yaw << ")@" << s.time;
    }

    int time;
    double x;
    double y;
    double yaw;

private:
    boost::numeric::ublas::matrix<double> rot;
    libMultiRobotPlanning::Point corner1, corner2, corner3, corner4;
};

namespace std
{
    template <>
    struct hash<State>
    {
        size_t operator()(const State &s) const
        {
            size_t seed = 0;
            boost::hash_combine(seed, s.time);
            boost::hash_combine(seed, s.x);
            boost::hash_combine(seed, s.y);
            boost::hash_combine(seed, s.yaw);
            return seed;
        }
    };
} // namespace std

using Action = int; // int<7 int ==6 wait

// struct Conflict
// {
//     int time;
//     size_t agent1;
//     size_t agent2;

//     State s1;
//     State s2;

//     friend std::ostream &operator<<(std::ostream &os, const Conflict &c)
//     {
//         os << c.time << ": Collision [ " << c.agent1 << c.s1 << " , " << c.agent2
//            << c.s2 << " ]";
//         return os;
//     }
// };





class Environment
{
public:
    Environment(size_t maxx, size_t maxy, std::unordered_set<Location> obstacles,
                std::multimap<int, State> dynamic_obstacles,
                std::vector<State> goals)
        : m_obstacles(std::move(obstacles)),
          m_dynamic_obstacles(std::move(dynamic_obstacles)),
          m_agentIdx(0),
          m_constraints(nullptr),
          m_lastGoalConstraint(-1),
          m_highLevelExpanded(0),
          m_lowLevelExpanded(0)
    {
        m_dimx = static_cast<int>(maxx / Constants::mapResolution);
        m_dimy = static_cast<int>(maxy / Constants::mapResolution);
        // std::cout << "env build " << m_dimx << " " << m_dimy << " "
        //           << m_obstacles.size() << std::endl;
        holonomic_cost_maps = std::vector<std::vector<std::vector<double>>>(
            goals.size(), std::vector<std::vector<double>>(
                              m_dimx, std::vector<double>(m_dimy, 0)));
        m_goals.clear();
        for (const auto &g : goals)
        {
            if (g.x < 0 || g.x > maxx || g.y < 0 || g.y > maxy)
            {
                std::cout << "\033[1m\033[31m Goal out of boundary, Fail to build "
                             "environment \033[0m\n";
                return;
            }
            m_goals.emplace_back(
                State(g.x, g.y, Constants::normalizeHeadingRad(g.yaw)));
        }
        updateCostmap();
    }

    Environment(const Environment &) = delete;
    Environment &operator=(const Environment &) = delete;


    // update the robot's goal
    void setRobotGoal(size_t m_agentIdx, int gx,int gy, double yaw){
        m_goals[m_agentIdx]=State(gx,gy,yaw);
    }

    double admissibleHeuristic(const State &s)
    {
        // non-holonomic-without-obstacles heuristic: use a Reeds-Shepp
        ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
        OmplState *rsStart = (OmplState *)reedsSheppPath.allocState();
        OmplState *rsEnd = (OmplState *)reedsSheppPath.allocState();
        rsStart->setXY(s.x, s.y);
        rsStart->setYaw(s.yaw);
        rsEnd->setXY(m_goals[m_agentIdx].x, m_goals[m_agentIdx].y);
        rsEnd->setYaw(m_goals[m_agentIdx].yaw);
        double reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
        // std::cout << "ReedsShepps cost:" << reedsSheppCost << std::endl;
        // Euclidean distance
        double euclideanCost = sqrt(pow(m_goals[m_agentIdx].x - s.x, 2) +
                                    pow(m_goals[m_agentIdx].y - s.y, 2));
        // std::cout << "Euclidean cost:" << euclideanCost << std::endl;
        // holonomic-with-obstacles heuristic
        double twoDoffset = sqrt(pow((s.x - static_cast<int>(s.x)) -
                                         (m_goals[m_agentIdx].x -
                                          static_cast<int>(m_goals[m_agentIdx].x)),
                                     2) +
                                 pow((s.y - static_cast<int>(s.y)) -
                                         (m_goals[m_agentIdx].y -
                                          static_cast<int>(m_goals[m_agentIdx].y)),
                                     2));
        double twoDCost =
            holonomic_cost_maps[m_agentIdx]
                               [static_cast<int>(s.x / Constants::mapResolution)]
                               [static_cast<int>(s.y / Constants::mapResolution)] -
            twoDoffset;
        // std::cout << "holonomic cost:" << twoDCost << std::endl;

        return std::max({reedsSheppCost, euclideanCost, twoDCost});
    }



   
   

    bool isSolution(
        const State &state, double gscore,
        std::unordered_map<State, std::tuple<State, Action, double, double>,
                           std::hash<State>> &_camefrom)
    {
        double goal_distance =
            sqrt(pow(state.x - getGoal().x, 2) + pow(state.y - getGoal().y, 2));
        if (goal_distance > 3 * (Constants::LB + Constants::LF))
            return false;
        ompl::base::ReedsSheppStateSpace reedsSheppSpace(Constants::r);
        OmplState *rsStart = (OmplState *)reedsSheppSpace.allocState();
        OmplState *rsEnd = (OmplState *)reedsSheppSpace.allocState();
        rsStart->setXY(state.x, state.y);
        rsStart->setYaw(-state.yaw);
        rsEnd->setXY(getGoal().x, getGoal().y);
        rsEnd->setYaw(-getGoal().yaw);
        ompl::base::ReedsSheppStateSpace::ReedsSheppPath reedsShepppath =
            reedsSheppSpace.reedsShepp(rsStart, rsEnd);

        std::vector<State> path;
        std::unordered_map<State, std::tuple<State, Action, double, double>,
                           std::hash<State>>
            cameFrom;
        cameFrom.clear();
        path.emplace_back(state);
        for (auto pathidx = 0; pathidx < 5; pathidx++)
        {
            if (fabs(reedsShepppath.length_[pathidx]) < 1e-6)
                continue;
            double deltat = 0, dx = 0, act = 0, cost = 0;
            switch (reedsShepppath.type_[pathidx])
            {
            case 0: // RS_NOP
                continue;
                break;
            case 1: // RS_LEFT
                deltat = -reedsShepppath.length_[pathidx];
                dx = Constants::r * sin(-deltat);
                // dy = Constants::r * (1 - cos(-deltat));
                act = 2;
                cost = reedsShepppath.length_[pathidx] * Constants::r *
                       Constants::penaltyTurning;
                break;
            case 2: // RS_STRAIGHT
                deltat = 0;
                dx = reedsShepppath.length_[pathidx] * Constants::r;
                // dy = 0;
                act = 0;
                cost = dx;
                break;
            case 3: // RS_RIGHT
                deltat = reedsShepppath.length_[pathidx];
                dx = Constants::r * sin(deltat);
                // dy = -Constants::r * (1 - cos(deltat));
                act = 1;
                cost = reedsShepppath.length_[pathidx] * Constants::r *
                       Constants::penaltyTurning;
                break;
            default:
                std::cout << "\033[1m\033[31m"
                          << "Warning: Receive unknown ReedsSheppPath type"
                          << "\033[0m\n";
                break;
            }
            if (cost < 0)
            {
                cost = -cost * Constants::penaltyReversing;
                act = act + 3;
            }
            State s = path.back();
            std::vector<std::pair<State, double>> next_path;
            if (generatePath(s, act, deltat, dx, next_path))
            {
                for (auto iter = next_path.begin(); iter != next_path.end(); iter++)
                {
                    State next_s = iter->first;
                    gscore += iter->second;
                    if (!(next_s == path.back()))
                    {
                        cameFrom.insert(std::make_pair<>(
                            next_s,
                            std::make_tuple<>(path.back(), act, iter->second, gscore)));
                    }
                    path.emplace_back(next_s);
                }
            }
            else
            {
                return false;
            }
        }

        if (path.back().time <= m_lastGoalConstraint)
        {
            return false;
        }

        m_goals[m_agentIdx] = path.back();

        _camefrom.insert(cameFrom.begin(), cameFrom.end());
        return true;
    }

    void getNeighbors(const State &s, Action action,
                      std::vector<Neighbor<State, Action, double>> &neighbors)
    {
        neighbors.clear();
        double g = Constants::dx[0];
        for (Action act = 0; act < 6; act++)
        { // has 6 directions for Reeds-Shepp
            double xSucc, ySucc, yawSucc;
            g = Constants::dx[0];
            xSucc = s.x + Constants::dx[act] * cos(-s.yaw) -
                    Constants::dy[act] * sin(-s.yaw);
            ySucc = s.y + Constants::dx[act] * sin(-s.yaw) +
                    Constants::dy[act] * cos(-s.yaw);
            yawSucc = Constants::normalizeHeadingRad(s.yaw + Constants::dyaw[act]);
            // if (act != action) {  // penalize turning
            //   g = g * Constants::penaltyTurning;
            //   if (act >= 3)  // penalize change of direction
            //     g = g * Constants::penaltyCOD;
            // }
            // if (act > 3) {  // backwards
            //   g = g * Constants::penaltyReversing;
            // }
            if (act % 3 != 0)
            { // penalize turning
                g = g * Constants::penaltyTurning;
            }
            if ((act < 3 && action >= 3) || (action < 3 && act >= 3))
            {
                // penalize change of direction
                g = g * Constants::penaltyCOD;
            }
            if (act >= 3)
            { // backwards
                g = g * Constants::penaltyReversing;
            }
            State tempState(xSucc, ySucc, yawSucc, s.time + 1);
            if (stateValid(tempState))
            {
                neighbors.emplace_back(
                    Neighbor<State, Action, double>(tempState, act, g));
            }
        }
        // wait
        g = Constants::dx[0];
        State tempState(s.x, s.y, s.yaw, s.time + 1);
        if (stateValid(tempState))
        {
            neighbors.emplace_back(Neighbor<State, Action, double>(tempState, 6, g));
        }
    }
    State getGoal() { return m_goals[m_agentIdx]; }
    uint64_t calcIndex(const State &s)
    {
        return (uint64_t)s.time * (2 * M_PI / Constants::deltat) *
                   (m_dimx / Constants::xyResolution) *
                   (m_dimy / Constants::xyResolution) +
               (uint64_t)(Constants::normalizeHeadingRad(s.yaw) /
                          Constants::yawResolution) *
                   (m_dimx / Constants::xyResolution) *
                   (m_dimy / Constants::xyResolution) +
               (uint64_t)(s.y / Constants::xyResolution) *
                   (m_dimx / Constants::xyResolution) +
               (uint64_t)(s.x / Constants::xyResolution);
    }


    bool startAndGoalValid(const std::vector<State> &m_starts, const size_t iter,
                           const int batchsize)
    {
        assert(m_goals.size() == m_starts.size());
        for (size_t i = 0; i < m_goals.size(); i++)
            for (size_t j = i + 1; j < m_goals.size(); j++)
            {
                if (m_goals[i].agentCollision(m_goals[j]))
                {
                    std::cout << "ERROR: Goal point of " << i + iter * batchsize << " & "
                              << j + iter * batchsize << " collide!\n";
                    return false;
                }
                if (m_starts[i].agentCollision(m_starts[j]))
                {
                    std::cout << "ERROR: Start point of " << i + iter * batchsize << " & "
                              << j + iter * batchsize << " collide!\n";
                    return false;
                }
            }
        return true;
    }

private:
    State getState(size_t agentIdx,
                   const std::vector<PlanResult<State, Action, double>> &solution,
                   size_t t)
    {
        assert(agentIdx < solution.size());
        if (t < solution[agentIdx].states.size())
        {
            return solution[agentIdx].states[t].first;
        }
        assert(!solution[agentIdx].states.empty());
        return solution[agentIdx].states.back().first;
    }

    bool stateValid(const State &s)
    {
        double x_ind = s.x / Constants::mapResolution;
        double y_ind = s.y / Constants::mapResolution;
        if (x_ind < 0 || x_ind >= m_dimx || y_ind < 0 || y_ind >= m_dimy)
            return false;

        for (auto it = m_obstacles.begin(); it != m_obstacles.end(); it++)
        {
            if (s.obsCollision(*it))
                return false;
        }

        auto it = m_dynamic_obstacles.equal_range(s.time);
        for (auto itr = it.first; itr != it.second; ++itr)
        {
            if (s.agentCollision(itr->second))
                return false;
        }
        auto itlow = m_dynamic_obstacles.lower_bound(-s.time);
        auto itup = m_dynamic_obstacles.upper_bound(-1);
        for (auto it = itlow; it != itup; ++it)
            if (s.agentCollision(it->second))
                return false;

        for (auto it = m_constraints->constraints.begin();
             it != m_constraints->constraints.end(); it++)
        {
            if (!it->satisfyConstraint(s))
                return false;
        }

        return true;
    }

private:
    struct compare_node
    {
        bool operator()(const std::pair<State, double> &n1,
                        const std::pair<State, double> &n2) const
        {
            return (n1.second > n2.second);
        }
    };
    void updateCostmap()
    {
        boost::heap::fibonacci_heap<std::pair<State, double>,
                                    boost::heap::compare<compare_node>>
            heap;

        std::set<std::pair<int, int>> temp_obs_set;
        for (auto it = m_obstacles.begin(); it != m_obstacles.end(); it++)
        {
            temp_obs_set.insert(
                std::make_pair(static_cast<int>(it->x / Constants::mapResolution),
                               static_cast<int>(it->y / Constants::mapResolution)));
        }

        for (size_t idx = 0; idx < m_goals.size(); idx++)
        {
            heap.clear();
            int goal_x = static_cast<int>(m_goals[idx].x / Constants::mapResolution);
            int goal_y = static_cast<int>(m_goals[idx].y / Constants::mapResolution);
            heap.push(std::make_pair(State(goal_x, goal_y, 0), 0));

            while (!heap.empty())
            {
                std::pair<State, double> node = heap.top();
                heap.pop();

                int x = node.first.x;
                int y = node.first.y;
                for (int dx = -1; dx <= 1; dx++)
                    for (int dy = -1; dy <= 1; dy++)
                    {
                        if (dx == 0 && dy == 0)
                            continue;
                        int new_x = x + dx;
                        int new_y = y + dy;
                        if (new_x == goal_x && new_y == goal_y)
                            continue;
                        if (new_x >= 0 && new_x < m_dimx && new_y >= 0 && new_y < m_dimy &&
                            holonomic_cost_maps[idx][new_x][new_y] == 0 &&
                            temp_obs_set.find(std::make_pair(new_x, new_y)) ==
                                temp_obs_set.end())
                        {
                            holonomic_cost_maps[idx][new_x][new_y] =
                                holonomic_cost_maps[idx][x][y] +
                                sqrt(pow(dx * Constants::mapResolution, 2) +
                                     pow(dy * Constants::mapResolution, 2));
                            heap.push(std::make_pair(State(new_x, new_y, 0),
                                                     holonomic_cost_maps[idx][new_x][new_y]));
                        }
                    }
            }
        }

        // for (size_t idx = 0; idx < m_goals.size(); idx++) {
        //   std::cout << "---------Cost Map -------Agent: " << idx
        //             << "------------\n";
        //   for (size_t i = 0; i < m_dimx; i++) {
        //     for (size_t j = 0; j < m_dimy; j++)
        //       std::cout << holonomic_cost_maps[idx][i][j] << "\t";
        //     std::cout << std::endl;
        //   }
        // }
    }

    bool generatePath(State startState, int act, double deltaSteer,
                      double deltaLength,
                      std::vector<std::pair<State, double>> &result)
    {
        double xSucc, ySucc, yawSucc, dx, dy, dyaw, ratio;
        result.emplace_back(std::make_pair<>(startState, 0));
        if (act == 0 || act == 3)
        {
            for (size_t i = 0; i < (size_t)(deltaLength / Constants::dx[act]); i++)
            {
                State s = result.back().first;
                xSucc = s.x + Constants::dx[act] * cos(-s.yaw) -
                        Constants::dy[act] * sin(-s.yaw);
                ySucc = s.y + Constants::dx[act] * sin(-s.yaw) +
                        Constants::dy[act] * cos(-s.yaw);
                yawSucc = Constants::normalizeHeadingRad(s.yaw + Constants::dyaw[act]);
                State nextState(xSucc, ySucc, yawSucc, result.back().first.time + 1);
                if (!stateValid(nextState))
                    return false;
                result.emplace_back(std::make_pair<>(nextState, Constants::dx[0]));
            }
            ratio =
                (deltaLength - static_cast<int>(deltaLength / Constants::dx[act]) *
                                   Constants::dx[act]) /
                Constants::dx[act];
            dyaw = 0;
            dx = ratio * Constants::dx[act];
            dy = 0;
        }
        else
        {
            for (size_t i = 0; i < (size_t)(deltaSteer / Constants::dyaw[act]); i++)
            {
                State s = result.back().first;
                xSucc = s.x + Constants::dx[act] * cos(-s.yaw) -
                        Constants::dy[act] * sin(-s.yaw);
                ySucc = s.y + Constants::dx[act] * sin(-s.yaw) +
                        Constants::dy[act] * cos(-s.yaw);
                yawSucc = Constants::normalizeHeadingRad(s.yaw + Constants::dyaw[act]);
                State nextState(xSucc, ySucc, yawSucc, result.back().first.time + 1);
                if (!stateValid(nextState))
                    return false;
                result.emplace_back(std::make_pair<>(
                    nextState, Constants::dx[0] * Constants::penaltyTurning));
            }
            ratio =
                (deltaSteer - static_cast<int>(deltaSteer / Constants::dyaw[act]) *
                                  Constants::dyaw[act]) /
                Constants::dyaw[act];
            dyaw = ratio * Constants::dyaw[act];
            dx = Constants::r * sin(dyaw);
            dy = -Constants::r * (1 - cos(dyaw));
            if (act == 2 || act == 5)
            {
                dx = -dx;
                dy = -dy;
            }
        }
        State s = result.back().first;
        xSucc = s.x + dx * cos(-s.yaw) - dy * sin(-s.yaw);
        ySucc = s.y + dx * sin(-s.yaw) + dy * cos(-s.yaw);
        yawSucc = Constants::normalizeHeadingRad(s.yaw + dyaw);
        // std::cout << m_agentIdx << " ratio::" << ratio << std::endl;
        State nextState(xSucc, ySucc, yawSucc, result.back().first.time + 1);
        if (!stateValid(nextState))
            return false;
        result.emplace_back(std::make_pair<>(nextState, ratio * Constants::dx[0]));

        // std::cout << "Have generate " << result.size() << " path segments:\n\t";
        // for (auto iter = result.begin(); iter != result.end(); iter++)
        //   std::cout << iter->first << ":" << iter->second << "->";
        // std::cout << std::endl;

        return true;
    }

private:
    int m_dimx;
    int m_dimy;
    std::vector<std::vector<std::vector<double>>> holonomic_cost_maps;
    std::unordered_set<Location> m_obstacles;
    std::multimap<int, State> m_dynamic_obstacles;
    std::vector<State> m_goals;
    // std::vector< std::vector<int> > m_heuristic;
    std::vector<double> m_vel_limit;
    size_t m_agentIdx;
    const Constraints *m_constraints;
    int m_lastGoalConstraint;
    int m_highLevelExpanded;
    int m_lowLevelExpanded;
};
