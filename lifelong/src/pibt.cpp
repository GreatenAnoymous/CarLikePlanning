#include "pibt.hpp"
#include <fstream>
#include <iostream>
#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include "config.hpp"
#include "timer.hpp"
#include "json.hpp"
#include <boost/numeric/ublas/matrix.hpp>
#include <random>

using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::OmplState;
using libMultiRobotPlanning::PIBT;
using libMultiRobotPlanning::PlanResult;
using libMultiRobotPlanning::Point;
using libMultiRobotPlanning::Segment;

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
    State(double x = 0, double y = 0, double yaw = 0, int time = 0, int greedy = 0, int headon = 0)
        : time(time), x(x), y(y), yaw(yaw), greedy(greedy), headon(headon)
    {
        rot.resize(2, 2);
        rot(0, 0) = cos(-this->yaw);
        rot(0, 1) = -sin(-this->yaw);
        rot(1, 0) = sin(-this->yaw);
        rot(1, 1) = cos(-this->yaw);
    }

    State() = default;

    bool operator==(const State &s) const
    {
        return std::tie(time, x, y, yaw) == std::tie(s.time, s.x, s.y, s.yaw);
    }

    bool agentCollision(const State &other) const
    {

        if (pow(this->x - other.x, 2) + pow(this->y - other.y, 2) <
            pow(2 * Constants::LF, 2) + pow(Constants::carWidth, 2))
            return true;
        return false;
    }

    bool equalWithoutTime(const State &other) const
    {
        double dist;
        double dyaw = pow(cos(yaw) - cos(other.yaw), 2) + pow(sin(yaw) - sin(other.yaw), 2);
        dist = sqrt(pow(x - other.x, 2) + pow(y - other.y, 2) + dyaw);
        return (dist < 1e-1);
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
    int greedy;
    int headon;

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

struct AStarNode
{
    State state;
    double f;
    double cost;
    std::shared_ptr<AStarNode> parent;
    Action act = 6;
    uint64_t sid;
    AStarNode(State &state, double f, double cost, Action act = 6, std::shared_ptr<AStarNode> parent = nullptr) : state(state), f(f), cost(cost), act(act), parent(parent) {}
};

class Environment
{
public:
    Environment(size_t maxx, size_t maxy, std::unordered_set<Location> obstacles,
                std::vector<State> goals)
        : m_obstacles(std::move(obstacles)),
          m_agentIdx(0)
    //   m_constraints(nullptr),
    {
        m_dimx = static_cast<int>(maxx / Constants::mapResolution);
        m_dimy = static_cast<int>(maxy / Constants::mapResolution);
        // std::cout << "env build " << m_dimx << " " << m_dimy << " "
        //           << m_obstacles.size() << std::endl;
        holonomic_cost_maps = std::vector<std::vector<std::vector<double>>>(
            m_dimx * m_dimy, std::vector<std::vector<double>>(
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

    size_t getLocationId(size_t xIndex, size_t yIndex)
    {
        return xIndex * m_dimy + yIndex;
    }

    size_t getLocationId(const State &state)
    {
        size_t xIndex = static_cast<size_t>(state.x / Constants::mapResolution);
        size_t yIndex = static_cast<size_t>(state.y / Constants::mapResolution);
        return getLocationId(xIndex, yIndex);
    }

    Environment(const Environment &) = delete;
    Environment &operator=(const Environment &) = delete;

    // update the robot's goal
    void setRobotGoal(size_t agentIdx, int gx, int gy, double yaw)
    {
        m_goals[agentIdx] = State(gx, gy, yaw);
    }

    void setLowLevelContext(size_t agentIdx)
    {
        m_agentIdx = agentIdx;
    }

    double admissibleHeuristic(const State &s, int agentIdx = 0)
    {
        // non-holonomic-without-obstacles heuristic: use a Reeds-Shepp
        ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
        OmplState *rsStart = (OmplState *)reedsSheppPath.allocState();
        OmplState *rsEnd = (OmplState *)reedsSheppPath.allocState();
        rsStart->setXY(s.x, s.y);
        rsStart->setYaw(-s.yaw);
        rsEnd->setXY(m_goals[agentIdx].x, m_goals[agentIdx].y);
        rsEnd->setYaw(-m_goals[agentIdx].yaw);
        double reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
        // std::cout << "ReedsShepps cost:" << reedsSheppCost << std::endl;
        // Euclidean distance
        double euclideanCost = sqrt(pow(m_goals[agentIdx].x - s.x, 2) +
                                    pow(m_goals[agentIdx].y - s.y, 2));
        // std::cout << "Euclidean cost:" << euclideanCost << std::endl;
        // holonomic-with-obstacles heuristic
        double twoDoffset = sqrt(pow((s.x - static_cast<int>(s.x)) -
                                         (m_goals[agentIdx].x -
                                          static_cast<int>(m_goals[agentIdx].x)),
                                     2) +
                                 pow((s.y - static_cast<int>(s.y)) -
                                         (m_goals[agentIdx].y -
                                          static_cast<int>(m_goals[agentIdx].y)),
                                     2));
        auto vid = getLocationId(s);
        double twoDCost =
            holonomic_cost_maps[vid]
                               [static_cast<int>(s.x / Constants::mapResolution)]
                               [static_cast<int>(s.y / Constants::mapResolution)] -
            twoDoffset;
        // std::cout << "holonomic cost:" << twoDCost << std::endl;

        return std::max({reedsSheppCost, euclideanCost, twoDCost});
    }

    size_t getNumAgents()
    {
        return m_goals.size();
    }

    void markHeadonNeighbor(size_t agent, State &curr, std::vector<Neighbor<State, Action, double>> &neighbors, int parent_agent = -1, Action parentAct = -1)
    {
        if (parent_agent == -1)
            return;
        if (parentAct != 0 and parentAct != 3)
            return;
        for (auto &nbr : neighbors)
        {
            if (nbr.action == 0 or nbr.action == 3)
            {
                if (admissibleHeuristic(nbr.state, agent) >= admissibleHeuristic(curr, agent))
                {
                    nbr.state.headon = 1;
                }
            }
        }
    }

    void getNeighbors(State &s, Action action,
                      std::vector<Neighbor<State, Action, double>> &neighbors, int agent = -1, bool canwait = true)
    {

        neighbors.clear();
        double g = Constants::dx[0];

#if DEBUG_ENABLE
        // std::cout << "current state of agent " << agent << "=" << s << "  goal state=" << getGoal(agent) << std::endl;
#endif
        for (Action act = 0; act < 6; act++)
        { // has 6 directions for Reeds-Shepp
            double xSucc, ySucc, yawSucc;
            g = Constants::dx[0];
            xSucc = s.x + Constants::dx[act] * cos(-s.yaw) -
                    Constants::dy[act] * sin(-s.yaw);
            ySucc = s.y + Constants::dx[act] * sin(-s.yaw) +
                    Constants::dy[act] * cos(-s.yaw);
            yawSucc = Constants::normalizeHeadingRad(s.yaw + Constants::dyaw[act]);

            if (act % 3 != 0)
            { // penalize turning
                // if (act < 3)
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
                std::random_device rd;                   // Obtain a random seed from the hardware if available
                std::default_random_engine engine(rd()); // Seed the random number engine
                std::uniform_real_distribution<double> distribution(0.0, 1.0);
                double random_number = distribution(engine);
                g += Constants::BSTIEBREAKER * random_number; // TIE BREAK THE BACKWARD ACTIONS
            }

            State tempState(xSucc, ySucc, yawSucc, s.time + 1);
            if (stateValid(tempState))
            {

                neighbors.emplace_back(
                    Neighbor<State, Action, double>(tempState, act, g));
            }
            else
            {
                // std::cout << "This neighbor is not valid!" << std::endl;
            }
        }
        if (canwait == true)
        {
            // wait
            g = Constants::dx[0];

            State tempState(s.x, s.y, s.yaw, s.time + 1);
            if (stateValid(tempState))
            {
                if (arrivedGoal(tempState, agent) == false)
                {
                    g = g * Constants::penaltyWait;
#if DEBUG_ENABLE
                    // std::cout << " penalty to wait  g=" << g << " penalty=" << Constants::penaltyWait << std::endl;
#endif
                }
                neighbors.emplace_back(Neighbor<State, Action, double>(tempState, 6, g));
            }
        }
        // analytic expansioned neighbor
    }

    State getGoal() { return m_goals[m_agentIdx]; }
    State getGoal(int agent) { return m_goals[agent]; }

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

    uint64_t calcIndexWithoutTIme(const State &s)
    {
        return (uint64_t)(Constants::normalizeHeadingRad(s.yaw) /
                          Constants::yawResolution) *
                   (m_dimx / Constants::xyResolution) *
                   (m_dimy / Constants::xyResolution) +
               (uint64_t)(s.y / Constants::xyResolution) *
                   (m_dimx / Constants::xyResolution) +
               (uint64_t)(s.x / Constants::xyResolution);
    }

    void updateGoal(State &new_goal, size_t agentId)
    {
        m_goals[agentId] = new_goal;
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

    void singleRobotMotionGuide(State &state, Action action, std::vector<Neighbor<State, Action, double>> &neighbors, int agent)
    {

        Neighbor<State, Action, double> next(state, action, Constants::dx[0]);

        windowedSearch(state, action, agent, next);

        next.state.greedy = 1;
        next.state.time = state.time + 1;
        neighbors.push_back(next);
    }

    void singleRobotPathPlanning(State &start, int agentId, std::vector<Neighbor<State, Action, double>> &path)
    {
        path.clear();
        double goal_distance = sqrt(pow(start.x - getGoal(agentId).x, 2) + pow(start.y - getGoal(agentId).y, 2));
        if (goal_distance < 1e-1)
        {
            Neighbor<State, Action, double> next(getGoal(agentId), 0, 1);
            next.state.time = start.time + 1;
            path.push_back(next);
            return;
        }
        using AStarNode_p = std::shared_ptr<AStarNode>;
        auto compare = [&](AStarNode_p a1, AStarNode_p a2)
        {
            if (a1->f != a2->f)
                return a1->f > a2->f;
            return a1->cost < a2->cost;
        };

        std::priority_queue<AStarNode_p, std::vector<AStarNode_p>, decltype(compare)> open(compare);
        std::unordered_map<uint64_t, AStarNode_p> openSet;
        std::unordered_set<uint64_t> closed;
        auto startNode = std::make_shared<AStarNode>(start, admissibleHeuristic(start, agentId), 0);
        startNode->sid = calcIndexWithoutTIme(start);
        startNode->act = 6;
        open.push(startNode);
        openSet.insert({startNode->sid, startNode});
        int t0 = start.time;
        while (!open.empty())
        {
            auto n = open.top();

            // std::cout << "state=" << n->state << "  cost=" << n->cost << " f=" << n->f<<"  h="<<n->f-n->cost << "  goal=" << getGoal(agentId) << std::endl;
            double goal_distance =
                sqrt(pow(n->state.x - getGoal(agentId).x, 2) + pow(n->state.y - getGoal(agentId).y, 2));
            if (goal_distance <= 3 * (Constants::LB + Constants::LF))
            {
                ompl::base::ReedsSheppStateSpace reedsSheppSpace(Constants::r);
                OmplState *rsStart = (OmplState *)reedsSheppSpace.allocState();
                OmplState *rsEnd = (OmplState *)reedsSheppSpace.allocState();
                rsStart->setXY(n->state.x, n->state.y);
                rsStart->setYaw(-n->state.yaw);
                rsEnd->setXY(getGoal(agentId).x, getGoal(agentId).y);
                rsEnd->setYaw(-getGoal(agentId).yaw);
                ompl::base::ReedsSheppStateSpace::ReedsSheppPath reedsShepppath =
                    reedsSheppSpace.reedsShepp(rsStart, rsEnd);
                double gScore = n->cost;
                auto tmp = n;
                std::vector<State> tmp_path;
                tmp_path.emplace_back(n->state);
                bool is_solution = true;
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
                    State s = tmp_path.back();
                    std::vector<std::pair<State, double>> next_path;

                    if (generatePath(s, act, deltat, dx, next_path))
                    {
                        for (auto iter = next_path.begin(); iter != next_path.end(); iter++)
                        {
                            State next_s = iter->first;
                            gScore += iter->second;
                            if (!(next_s == tmp_path.back()))
                            {
                                auto child = std::make_shared<AStarNode>(next_s, gScore, gScore, act, tmp);
                                tmp = child;
                            }
                            tmp_path.emplace_back(next_s);
                        }
                    }
                    else
                    {
                        is_solution = false;
                        break;
                    }
                }
                if (is_solution)
                {
                    while (tmp->parent != nullptr)
                    {
                        path.emplace_back(Neighbor<State, Action, double>(tmp->state, tmp->act, tmp->cost));
                        tmp = tmp->parent;
                    }
                    return;
                }
            }

            closed.insert(n->sid);
            open.pop();
            openSet.erase(n->sid);
            std::vector<Neighbor<State, Action, double>> neighbors;
            getNeighbors(n->state, n->act, neighbors, agentId, false);
            for (auto &nbr : neighbors)
            {

                int cid = calcIndexWithoutTIme(nbr.state);
                if (closed.find(cid) != closed.end())
                    continue;

                double g = n->cost + nbr.cost;

                if (openSet.find(cid) == openSet.end())
                {
                    double h = admissibleHeuristic(nbr.state, agentId);
                    auto child = std::make_shared<AStarNode>(nbr.state, g + h, g, nbr.action, n);
                    child->sid = cid;
                    open.push(child);
                    openSet.insert({cid, child});
                }
                else
                {
                    auto oldNode = openSet[cid];
                    if (oldNode->cost < g)
                        continue;
                    double h = admissibleHeuristic(nbr.state, agentId);
                    auto child = std::make_shared<AStarNode>(nbr.state, g + h, g, nbr.action, n);
                    child->sid = cid;
                    openSet[cid] = child;
                }
            }
        }
        std::cout << "path not found for agent  " << agentId << std::endl;
        // throw std::runtime_error("did not find solution");
        return;
    }

    bool arrivedGoal(State s, int agent)
    {
        double dist;
        auto goal = getGoal(agent);
        double dyaw = pow(cos(s.yaw) - cos(goal.yaw), 2) + pow(sin(s.yaw) - sin(goal.yaw), 2);
        dist = sqrt(pow(s.x - goal.x, 2) + pow(s.y - goal.y, 2) + dyaw);
        return (dist < 1e-1);
    }

    void addGoalToNeighborIfClose(State &state, Action action, std::vector<Neighbor<State, Action, double>> &neighbors, int agent)
    {
        // check if the s is close enough to the goal state, if so, add the goal state to neighbors
        double goal_distance = sqrt(pow(state.x - getGoal(agent).x, 2) + pow(state.y - getGoal(agent).y, 2));

        // if (goal_distance > 3 * (Constants::LB + Constants::LF))
        //     return;
        ompl::base::ReedsSheppStateSpace reedsSheppSpace(Constants::r);
        OmplState *rsStart = (OmplState *)reedsSheppSpace.allocState();
        OmplState *rsEnd = (OmplState *)reedsSheppSpace.allocState();
        rsStart->setXY(state.x, state.y);
        rsStart->setYaw(-state.yaw);
        rsEnd->setXY(getGoal(agent).x, getGoal(agent).y);
        rsEnd->setYaw(-getGoal(agent).yaw);
        ompl::base::ReedsSheppStateSpace::ReedsSheppPath reedsShepppath = reedsSheppSpace.reedsShepp(rsStart, rsEnd);
        std::vector<State> path;
        auto goal = getGoal(agent);

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

            auto candidate = getNextStateCloseToGoal(state, act, deltat, dx);
            if (stateValid(candidate.first))
            {
                auto dyaw = pow(cos(getGoal(agent).yaw) - cos(candidate.first.yaw), 2) + pow(sin(getGoal(agent).yaw) - sin(candidate.first.yaw), 2);
                double dist = sqrt(pow(candidate.first.x - getGoal(agent).x, 2) + pow(candidate.first.y - getGoal(agent).y, 2) + dyaw);
                if (dist < 1e-1)
                {
                    candidate.first = getGoal(agent);

                    candidate.first.time = state.time + 1;
                }
                for (auto it = neighbors.begin(); it != neighbors.end(); it++)
                {
                    if (it->state == candidate.first)
                    {
                        neighbors.erase(it);
                        break;
                    }
                }
                candidate.first.greedy = 1;
#if DEBUG
                std::cout << "reeds-shepp  state is" << candidate.first << std::endl;
#endif
                // neighbors.push_back(Neighbor<State, Action, double>(candidate.first, act, candidate.second));
                neighbors.insert(neighbors.begin(), Neighbor<State, Action, double>(candidate.first, act, cost));
                break;
            }
            else
            {
                // std::cout << "This neighbor is not valid!" << std::endl;
            }
        }
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

        return true;
    }

    void windowedSearch(State &start, Action action, int agentId, Neighbor<State, Action, double> &next, int window = 25)
    {

        if (arrivedGoal(start, agentId))
        {
            next.state = getGoal(agentId);
            next.state.time = start.time + 1;
            next.cost = 0;
            next.action = 1;
            return;
        }
        using AStarNode_p = std::shared_ptr<AStarNode>;
        auto compare = [&](AStarNode_p a1, AStarNode_p a2)
        {
            if (a1->f != a2->f)
                return a1->f > a2->f;
            return a1->cost < a2->cost;
        };

        std::priority_queue<AStarNode_p, std::vector<AStarNode_p>, decltype(compare)> open(compare);
        std::unordered_map<uint64_t, AStarNode_p> openSet;
        std::unordered_set<uint64_t> closed;
        auto startNode = std::make_shared<AStarNode>(start, admissibleHeuristic(start, agentId), 0);
        startNode->sid = calcIndexWithoutTIme(start);
        startNode->act = action;
        open.push(startNode);
        openSet.insert({startNode->sid, startNode});
        int t0 = start.time;
        while (!open.empty())
        {
            auto n = open.top();
            // if (n->state.time >= t0 + window)
            // {
            //     auto curr = n;
            //     while (curr->parent != startNode)
            //     {
            //         curr = curr->parent;
            //     }
            //     next.action = curr->act;
            //     next.cost = curr->cost;
            //     next.state = curr->state;
            //     // std::reverse(path.begin(), path.end());
            //     return;
            // }
            double goal_distance =
                sqrt(pow(n->state.x - getGoal(agentId).x, 2) + pow(n->state.y - getGoal(agentId).y, 2));
            if (goal_distance <= 3 * (Constants::LB + Constants::LF))
            {
                ompl::base::ReedsSheppStateSpace reedsSheppSpace(Constants::r);
                OmplState *rsStart = (OmplState *)reedsSheppSpace.allocState();
                OmplState *rsEnd = (OmplState *)reedsSheppSpace.allocState();
                rsStart->setXY(n->state.x, n->state.y);
                rsStart->setYaw(-n->state.yaw);
                rsEnd->setXY(getGoal(agentId).x, getGoal(agentId).y);
                rsEnd->setYaw(-getGoal(agentId).yaw);
                ompl::base::ReedsSheppStateSpace::ReedsSheppPath reedsShepppath =
                    reedsSheppSpace.reedsShepp(rsStart, rsEnd);
                std::vector<State> tmp_path;
                double gScore = n->cost;
                tmp_path.emplace_back(n->state);
                auto tmp = n;
                bool is_solution = true;
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
                    State s = tmp_path.back();
                    std::vector<std::pair<State, double>> next_path;

                    if (generatePath(s, act, deltat, dx, next_path))
                    {
                        for (auto iter = next_path.begin(); iter != next_path.end(); iter++)
                        {
                            State next_s = iter->first;
                            gScore += iter->second;
                            if (!(next_s == tmp_path.back()))
                            {
                                auto child = std::make_shared<AStarNode>(next_s, gScore, gScore, act, tmp);
                                tmp = child;
                            }
                            tmp_path.emplace_back(next_s);
                        }
                    }
                    else
                    {
                        is_solution = false;
                        break;
                    }
                }
                if (is_solution)
                {
                    // std::cout << "debug is solution" << tmp->state << "  " << getGoal(agentId) << std::endl;
                    while (tmp->parent != nullptr)
                    {
                        if (tmp->parent->state == start)
                            break;
                        tmp = tmp->parent;
                    }
                    next.action = tmp->act;
                    next.cost = tmp->cost;
                    next.state = tmp->state;

                    return;
                    // std::cout << "Debug windowed search found solution!!!!" << std::endl;
                    // auto curr = n;
                    // if (n != startNode)
                    // {
                    //     while (curr->parent != startNode)
                    //     {
                    //         // path.push_back(curr->state);
                    //         curr = curr->parent;
                    //     }
                    //     next.action = curr->act;
                    //     next.cost = curr->cost;
                    //     next.state = curr->state;
                    //     // if (arrivedGoal(next.state, agentId))
                    //     // {
                    //     //     std::cout << "debug windowed search arrived goals" << next.state << "   " << getGoal(agentId) << std::endl;
                    //     //     next.state = getGoal(agentId);
                    //     // }
                    //     // std::reverse(path.begin(), path.end());
                    //     return;
                    // }
                    // else
                    // {
                    //     std::cout << "debug windowed search arrived goals" << next.state << "   " << getGoal(agentId) << std::endl;
                    //     std::cout << " debug reeds shepp path, goal=" << getGoal(agentId) << std::endl;
                    //     for (auto &v : tmp_path)
                    //     {
                    //         std::cout << v << std::endl;
                    //     }
                    //     std::cout << std::endl;
                    //     assert(tmp_path.size() > 1);
                    //     next.action = 1;
                    //     next.cost = Constants::dx[0];
                    //     next.state = tmp_path[1];
                    //     if (arrivedGoal(next.state, agentId))
                    //     {
                    //         next.state = getGoal(agentId);
                    //         next.state.time = n->state.time + 1;
                    //     }
                    //     // for (int k = 1; k < tmp_path.size(); k++)
                    //     // {
                    //     //     if (tmp_path[k].time > t0 + window)
                    //     //         break;
                    //     //     path.push_back(tmp_path[k]);
                    //     // }
                    //     return;
                    // }
                }
            }

            closed.insert(n->sid);
            open.pop();
            openSet.erase(n->sid);
            std::vector<Neighbor<State, Action, double>> neighbors;
            getNeighbors(n->state, n->act, neighbors, agentId, false);
            for (auto &nbr : neighbors)
            {

                int cid = calcIndexWithoutTIme(nbr.state);
                if (closed.find(cid) != closed.end())
                    continue;

                double g = n->cost + nbr.cost;
                if (openSet.find(cid) != openSet.end())
                {
                    double h = admissibleHeuristic(nbr.state, agentId);
                    auto child = std::make_shared<AStarNode>(nbr.state, g + h, g, nbr.action, n);
                    child->sid = cid;

                    open.push(child);
                    openSet.insert({cid, child});
                }
                else
                {
                    auto oldNode = openSet[cid];
                    if (oldNode == nullptr)
                        continue;
                    // assert(oldNode!=nullptr);
                    if (oldNode->cost < g)
                        continue;
                    double h = admissibleHeuristic(nbr.state, agentId);
                    auto child = std::make_shared<AStarNode>(nbr.state, g + h, g, nbr.action, n);
                    child->sid = cid;
                    openSet[cid] = child;
                }
            }
        }
        // throw std::runtime_error("did not find solution");
        return;
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

        for (size_t goal_x = 0; goal_x < m_dimx; goal_x++)
        {
            for (size_t goal_y = 0; goal_y < m_dimy; goal_y++)
            {
                auto idx = getLocationId(goal_x, goal_y);
                heap.clear();
                heap.push(std::make_pair(State(goal_x, goal_y, 0), 0));

                while (!heap.empty())
                {
                    std::pair<State, double> node = heap.top();
                    heap.pop();

                    int x = node.first.x;
                    int y = node.first.y;
                    for (int dx = -1; dx <= 1; dx++)
                    {
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
            }
        }
    }

    std::pair<State, double> getNextStateCloseToGoal(State startState, int act, double deltaSteer, double deltaLength)
    {
        // std::cout << "debug next state  " << startState << "    " << act << "   " << deltaSteer << "    " << deltaLength << std::endl;
        double xSucc, ySucc, yawSucc, dx, dy, dyaw;
        if (act == 0 || act == 3)
        {
            // double L = std::min(Constants::dx[act], deltaLength);
            // std::cout<<"  debug get next state close to goal:" << Constants::dx[act] << "    " << deltaLength << std::endl;
            if (fabs(Constants::dx[act]) < fabs(deltaLength))
            {
                State s = startState;
                xSucc = s.x + Constants::dx[act] * cos(-s.yaw) -
                        Constants::dy[act] * sin(-s.yaw);
                ySucc = s.y + Constants::dx[act] * sin(-s.yaw) +
                        Constants::dy[act] * cos(-s.yaw);
                yawSucc = Constants::normalizeHeadingRad(s.yaw + Constants::dyaw[act]);
                State nextState(xSucc, ySucc, yawSucc, startState.time + 1);
                // return {nextState, Constants::dx[act]};

                return {nextState, 0};
            }
            else
            {
                dyaw = 0;
                dx = deltaLength;
                dy = 0;
            }
        }
        else
        {
            if (fabs(Constants::dyaw[act]) < fabs(deltaSteer))
            {
                double dyaw = std::min(Constants::dyaw[act], deltaSteer);
                State s = startState;
                xSucc = s.x + Constants::dx[act] * cos(-s.yaw) -
                        Constants::dy[act] * sin(-s.yaw);
                ySucc = s.y + Constants::dx[act] * sin(-s.yaw) +
                        Constants::dy[act] * cos(-s.yaw);
                yawSucc = Constants::normalizeHeadingRad(s.yaw + Constants::dyaw[act]);
                State nextState(xSucc, ySucc, yawSucc, s.time + 1);
                // return {nextState, Constants::dx[0] * Constants::penaltyTurning};
                return {nextState, 0};
            }
            else
            {
                dyaw = deltaSteer;
                dx = Constants::r * sin(dyaw);
                dy = -Constants::r * (1 - cos(dyaw));
                if (act == 2 || act == 5)
                {
                    dx = -dx;
                    dy = -dy;
                }
            }
        }
        State s = startState;
        xSucc = s.x + dx * cos(-s.yaw) - dy * sin(-s.yaw);
        ySucc = s.y + dx * sin(-s.yaw) + dy * cos(-s.yaw);
        yawSucc = Constants::normalizeHeadingRad(s.yaw + dyaw);
        State nextState(xSucc, ySucc, yawSucc, s.time + 1);

        // return {nextState, sqrt(dx * dx + dy * dy)};
        return {nextState, 0};
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

    std::vector<State> m_goals;
    // std::vector< std::vector<int> > m_heuristic;
    // std::vector<double> m_vel_limit;
    size_t m_agentIdx;
    // const Constraints *m_constraints;
    // int m_lastGoalConstraint;

public:
};

void readMapsFromJson(std::string inputFile, size_t &xmax, size_t &ymax,
                      std::vector<State> &starts,
                      std::vector<std::vector<State>> &tasks,
                      std::unordered_set<Location> &obstacles)
{
    std::ifstream file(inputFile);
    if (!file.is_open())
    {
        std::cerr << "Error opening file: " << inputFile << std::endl;
        return;
    }

    // Parse the JSON data
    nlohmann::json data;
    try
    {
        file >> data;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error parsing JSON: " << e.what() << std::endl;
        file.close();
        return;
    }

    // Extract data from JSON
    try
    {
        xmax = data["xmax"];
        ymax = data["ymax"];

        // Read obstacles
        if (data.count("obstacles") > 0)
        {
            for (const auto &obstacle : data["obstacles"])
            {
                std::vector<double> obstacleCoords = obstacle;
                // obstacle_vec.push_back(obstacleCoords);
                obstacles.insert(Location(obstacleCoords[0], obstacleCoords[1]));
            }
        }

        // Read starts
        if (data.count("starts") > 0)
        {
            std::cout << "starts size=" << data["starts"].size() << std::endl;
            for (const auto &svec : data["starts"])
            {
                // obstacle_vec.push_back(obstacleCoords);
                starts.push_back(State(svec[0], svec[1], svec[2]));
            }
        }

        // Read goals
        if (data.count("goals") > 0)
        {
            std::cout << "goals size=" << data["goals"].size() << std::endl;
            for (size_t k = 0; k < data["goals"].size(); k++)
            {
                // std::cout<<"total tasks="<<goal.size()<<std::endl;
                std::vector<std::vector<double>> goalCoords = data["goals"][k];
                std::cout << "total tasks=" << goalCoords.size() << std::endl;
                //
                std::vector<State> gs;
                for (auto &gi : goalCoords)
                {
                    gs.push_back(State(gi[0], gi[1], gi[2]));
                }
                tasks.push_back(gs);
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error extracting data from JSON: " << e.what() << std::endl;
    }

    // Close the file
    file.close();
}

void test_demo()
{
    std::string tasks_file = "./test.json";
    size_t xmax, ymax;
    std::unordered_set<Location> obstacles;
    std::vector<std::vector<State>> tasks;
    std::vector<State> starts;
    std::vector<State> firstGoals;
    readMapsFromJson(tasks_file, xmax, ymax, starts, tasks, obstacles);
    for (size_t i = 0; i < tasks.size(); i++)
    {
        firstGoals.push_back(tasks[i][0]);
    }

    Environment environment(xmax, ymax, obstacles, firstGoals);
    PIBT<State, Action, double, Environment> pibt(environment, tasks);
    pibt.setInitialStates(starts);
    pibt.PIBT_sim();
    pibt.dumpOutputToJson("./demo_json.json");
}

int PIBT_exe(int argc, char *argv[])
{
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    std::string inputFile;
    std::string outputFile;
    float w;
    desc.add_options()("help", "produce help message")(
        "input,i", po::value<std::string>(&inputFile)->required(),
        "input file (YAML)")("output,o",
                             po::value<std::string>(&outputFile)->required(),
                             "output file (YAML)")(
        "suboptimality,w", po::value<float>(&w)->default_value(1.0),
        "suboptimality bound");

    try
    {
        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);

        if (vm.count("help") != 0u)
        {
            std::cout << desc << "\n";
            return 0;
        }
    }
    catch (po::error &e)
    {
        std::cerr << e.what() << std::endl
                  << std::endl;
        std::cerr << desc << std::endl;
        return 1;
    }

    size_t dimx, dimy;

    std::unordered_set<Location> obstacles;

    std::vector<std::vector<State>> tasks;
    std::vector<State> startStates;
    readMapsFromJson(inputFile, dimx, dimy, startStates, tasks, obstacles);
    std::vector<State> firstGoals;
    for (size_t i = 0; i < tasks.size(); i++)
    {
        firstGoals.push_back(tasks[i][0]);
    }
    Environment environment(dimx, dimy, obstacles, firstGoals);
    PIBT<State, Action, double, Environment> pibt(environment, tasks);
    pibt.setInitialStates(startStates);
    pibt.PIBT_sim();
    pibt.dumpOutputToJson(outputFile);
    return 0;
}

int main(int argc, char *argv[])
{
    // PIBT_exe(argc, argv);
    test_demo();
    return 0;
}