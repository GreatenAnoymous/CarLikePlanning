#include <fstream>
#include <iostream>
// #include "environment.hpp"
#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <yaml-cpp/yaml.h>

#include "multilabel.hpp"
#include "timer.hpp"
#include "config.hpp"

#include <boost/numeric/ublas/matrix.hpp>

using libMultiRobotPlanning::MultiLabelHybridAstar;
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
struct Conflict
{
    int time;
    size_t agent1;
    size_t agent2;

    State s1;
    State s2;

    friend std::ostream &operator<<(std::ostream &os, const Conflict &c)
    {
        os << c.time << ": Collision [ " << c.agent1 << c.s1 << " , " << c.agent2
           << c.s2 << " ]";
        return os;
    }
};

struct Constraint
{
    Constraint(int time, State s, size_t agentid)
        : time(time), s(s), agentid(agentid) {}
    Constraint() = default;
    int time;
    State s;
    size_t agentid;

    bool operator<(const Constraint &other) const
    {
        return std::tie(time, s.x, s.y, s.yaw, agentid) <
               std::tie(other.time, other.s.x, other.s.y, other.s.yaw,
                        other.agentid);
    }

    bool operator==(const Constraint &other) const
    {
        return std::tie(time, s.x, s.y, s.yaw, agentid) ==
               std::tie(other.time, other.s.x, other.s.y, other.s.yaw,
                        other.agentid);
    }

    friend std::ostream &operator<<(std::ostream &os, const Constraint &c)
    {
        return os << "Constraint[" << c.time << "," << c.s << "from " << c.agentid
                  << "]";
    }

    bool satisfyConstraint(const State &state) const
    {
        if (state.time < this->time ||
            state.time > this->time + Constants::constraintWaitTime)
            return true;
        return !this->s.agentCollision(state);
    }
};

namespace std
{
    template <>
    struct hash<Constraint>
    {
        size_t operator()(const Constraint &s) const
        {
            size_t seed = 0;
            boost::hash_combine(seed, s.time);
            boost::hash_combine(seed, s.s.x);
            boost::hash_combine(seed, s.s.y);
            boost::hash_combine(seed, s.s.yaw);
            boost::hash_combine(seed, s.agentid);
            return seed;
        }
    };
} // namespace std

// FIXME: modidy data struct, it's not the best option
struct Constraints
{
    std::unordered_set<Constraint> constraints;

    void add(const Constraints &other)
    {
        constraints.insert(other.constraints.begin(), other.constraints.end());
    }

    bool overlap(const Constraints &other)
    {
        for (const auto &c : constraints)
        {
            if (other.constraints.count(c) > 0)
                return true;
        }
        return false;
    }

    friend std::ostream &operator<<(std::ostream &os, const Constraints &cs)
    {
        for (const auto &c : cs.constraints)
        {
            os << c << std::endl;
        }
        return os;
    }
};

class Environment
{
public:
    Environment(size_t maxx, size_t maxy, std::unordered_set<Location> obstacles,
                std::multimap<int, State> dynamic_obstacles,
                std::vector<std::vector<State>> goals) : m_obstacles(std::move(obstacles)),
                                                         m_dynamic_obstacles(std::move(dynamic_obstacles)),
                                                         m_agentIdx(0),
                                                         m_constraints(nullptr),
                                                         m_lastGoalConstraint(-1),
                                                         m_highLevelExpanded(0),
                                                         m_lowLevelExpanded(0)
    {
        m_dimx = static_cast<int>(maxx / Constants::mapResolution);
        m_dimy = static_cast<int>(maxy / Constants::mapResolution);
        // holonomic_cost_maps = std::vector<std::vector<std::vector<double>>>(
        //     m_dimx*m_dimy, std::vector<std::vector<double>>(
        //                       m_dimx, std::vector<double>(m_dimy, 0)));
        m_goals.clear();
        for (const auto &gVec : goals)
        {
            std::vector<State> m_vec;
            for (auto g : gVec)
            {
                if (g.x < 0 || g.x > maxx || g.y < 0 || g.y > maxy)
                {
                    std::cout << "\033[1m\033[31m Goal out of boundary, Fail to build "
                                 "environment \033[0m\n";
                    return;
                }
                m_vec.emplace_back(
                    State(g.x, g.y, Constants::normalizeHeadingRad(g.yaw)));
            }
            m_goals.push_back(m_vec);
        }
    }
    updateCostmap();
    Environment(const Environment &) = delete;
    Environment &operator=(const Environment &) = delete;

    bool loadPreprocessedData(std::vector<std::vector<std::vector<double>>> &data)
    {
        std::ifstream file("./tmp/preprocessed.bin", std::ios::binary);
        if (!file)
        {
            return false; // File does not exist
        }

        // Read the data from the file
        file.read(reinterpret_cast<char *>(data.data()), data.size() * sizeof(data[0]));
        file.close();

        return true; // Successfully loaded the data
    }

    void savePreprocessedData(const std::vector<std::vector<std::vector<double>>> &data)
    {
        std::ofstream file("./tmp/preprocessed.bin", std::ios::binary);
        file.write(reinterpret_cast<const char *>(data.data()), data.size() * sizeof(data[0]));
        file.close();
    }

    void updateCostmap()
    {

        std::ifstream file("preprocessed.bin", std::ios::binary);
        if (loadPreprocessedData(holonomic_cost_maps))
        {
            return;
        }

        holonomic_cost_maps = std::vector<std::vector<std::vector<double>>>(
            m_dimx*m_dimy, std::vector<std::vector<double>>(
                              m_dimx, std::vector<double>(m_dimy, 0)));
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

        //create a set containing all the waypoints
        std::set<std::pair<int,int>> all_waypoints;
        for(auto &goal_vec:m_goals){
            for(auto &goal:goal_vec){
                all_waypoints.insert({static_cast<int>(goal.x/Constants::mapResolution),static_cast<int>(goal.y/Constants::mapResolution)});
            }
        }

        for (auto &waypoint:all_waypoints)
        {
            int idx=getVid(waypoint.x,waypoint.y);
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
        savePreprocessedData(holonomic_cost_maps);
    }

private:
    int m_dimx;
    int m_dimy;
    int getVid(int x, int y)
    {
        return x * m_dimy + y;
    }

    std::pair<int, int> getVertex(int vid)
    {
        int x, y;
        x = Vid / m_dimy;
        y = Vid % m_dimy;
        return {x, y};
    }

    std::vector<std::vector<std::vector<double>>> holonomic_cost_maps; //
    std::unordered_set<Location> m_obstacles;
    std::multimap<int, State> m_dynamic_obstacles;
    std::vector<std::vector<State>> m_goals;

    std::vector<double> m_vel_limit;
    size_t m_agentIdx;
    const Constraints *m_constraints;
    int m_lastGoalConstraint;
    int m_highLevelExpanded;
    int m_lowLevelExpanded;
};