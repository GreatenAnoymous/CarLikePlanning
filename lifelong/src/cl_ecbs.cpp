#include <fstream>
#include <iostream>
#include "environment.hpp"
#include "json.hpp"
#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include "config.hpp"
#include "cl_ecbs.hpp"
#include "timer.hpp"
#include <boost/numeric/ublas/matrix.hpp>

using libMultiRobotPlanning::ECBS;
using libMultiRobotPlanning::Environment;
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
        float safe_factor = 1.1;
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
///
// enum class Action
// {
//     Up,
//     Down,
//     Left,
//     Right,
//     Wait,
// };

// std::ostream &operator<<(std::ostream &os, const Action &a)
// {
//     switch (a)
//     {
//     case Action::Up:
//         os << "Up";
//         break;
//     case Action::Down:
//         os << "Down";
//         break;
//     case Action::Left:
//         os << "Left";
//         break;
//     case Action::Right:
//         os << "Right";
//         break;
//     case Action::Wait:
//         os << "Wait";
//         break;
//     }
//     return os;
// }
///

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

void readAgentConfig()
{
    YAML::Node car_config;
    std::string test(__FILE__);
    boost::replace_all(test, "cl_ecbs.cpp", "config.yaml");
    try
    {
        car_config = YAML::LoadFile(test.c_str());
    }
    catch (std::exception &e)
    {
        std::cerr << "\033[1m\033[33mWARNING: Failed to load agent config file: "
                  << test << "\033[0m , Using default params. \n";
    }
    // int car_r = car_config["r"].as<int>();
    Constants::r = car_config["r"].as<double>();
    Constants::deltat = car_config["deltat"].as<double>();
    Constants::penaltyTurning = car_config["penaltyTurning"].as<double>();
    Constants::penaltyReversing = car_config["penaltyReversing"].as<double>();
    Constants::penaltyCOD = car_config["penaltyCOD"].as<double>();
    // map resolution
    Constants::mapResolution = car_config["mapResolution"].as<double>();
    // change to set calcIndex resolution
    Constants::xyResolution = Constants::r * Constants::deltat;
    Constants::yawResolution = Constants::deltat;

    Constants::carWidth = car_config["carWidth"].as<double>();
    Constants::LF = car_config["LF"].as<double>();
    Constants::LB = car_config["LB"].as<double>();
    // obstacle default radius
    Constants::obsRadius = car_config["obsRadius"].as<double>();
    // least time to wait for constraint
    Constants::constraintWaitTime = car_config["constraintWaitTime"].as<double>();

    Constants::dx = {Constants::r * Constants::deltat,
                     Constants::r * sin(Constants::deltat),
                     Constants::r * sin(Constants::deltat),
                     -Constants::r * Constants::deltat,
                     -Constants::r * sin(Constants::deltat),
                     -Constants::r * sin(Constants::deltat)};
    Constants::dy = {0,
                     -Constants::r * (1 - cos(Constants::deltat)),
                     Constants::r * (1 - cos(Constants::deltat)),
                     0,
                     -Constants::r * (1 - cos(Constants::deltat)),
                     Constants::r * (1 - cos(Constants::deltat))};
    Constants::dyaw = {0, Constants::deltat, -Constants::deltat,
                       0, -Constants::deltat, Constants::deltat};
}

void readMapsFromJson(std::string inputFile, size_t &xmax, size_t &ymax,
                      std::vector<State> &starts,
                      std::vector<State> &goals,
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

    // std::vector<std::vector<double>> obstacle_vec;
    // std::vector<std::vector<double>>  start_vec;
    // std::vector<std::vector<std::vector<double>>> goal_vec;
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
            for (const auto &goal : data["goals"])
            {
                // obstacle_vec.push_back(obstacleCoords);
                goals.push_back(State(goal[0], goal[1], goal[2]));
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

void dumpOutputToJson(std::string outputFile,
                      std::vector<PlanResult<State, Action, double>> &solution, double timer = 0, bool savepath = false)
{
    double makespan = 0, flowtime = 0, cost = 0;
    for (const auto &s : solution)
        cost += s.cost;

    for (size_t a = 0; a < solution.size(); ++a)
    {
        // calculate makespan
        double current_makespan = 0;
        for (size_t i = 0; i < solution[a].actions.size(); ++i)
        {
            // some action cost have penalty coefficient

            if (solution[a].actions[i].second < Constants::dx[0])
                current_makespan += solution[a].actions[i].second;
            else if (solution[a].actions[i].first % 3 == 0)
                current_makespan += Constants::dx[0];
            else
                current_makespan += Constants::r * Constants::deltat;
        }
        flowtime += current_makespan;
        if (current_makespan > makespan)
            makespan = current_makespan;
    }

    nlohmann::json output;
    // Add statistics data to the JSON object
    output["cost"] = cost;
    output["makespan"] = makespan;
    output["flowtime"] = flowtime;
    output["runtime"] = timer;
    std::vector<std::vector<std::vector<double>>> paths;
    std::cout << "solution size=" << solution.size() << std::endl;
    // Add schedule data to the JSON object
    if (savepath)
    {
        for (size_t a = 0; a < solution.size(); ++a)
        {
            // nlohmann::json agentData;
            std::vector<std::vector<double>> path;
            for (const auto &state : solution[a].states)
            {
                path.push_back({state.first.x, state.first.y, state.first.yaw});
            }
            paths.push_back(path);
        }
        output["paths"] = paths;
    }
    // Save the JSON data to a file
    std::ofstream file(outputFile);
    if (file.is_open())
    {
        file << std::setw(4) << output << std::endl;
        file.close();
        std::cout << "JSON data saved successfully." << std::endl;
    }
    else
    {
        std::cerr << "Error opening file." << std::endl;
    }
}

int ECBS_exe(int argc, char *argv[])
{
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    std::string inputFile;
    std::string outputFile;
    float w;
    int batchSize;
    desc.add_options()("help", "produce help message")(
        "input,i", po::value<std::string>(&inputFile)->required(),
        "input file (YAML)")("output,o",
                             po::value<std::string>(&outputFile)->required(),
                             "output file (YAML)")(
        "suboptimality,w", po::value<float>(&w)->default_value(1.0),
        "suboptimality bound")(
        "batchsize,b", po::value<int>(&batchSize)->default_value(10), "batch size for iter");

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

    readAgentConfig();

    size_t dimx, dimy;

    std::unordered_set<Location> obstacles;
    std::multimap<int, State> dynamic_obstacles;
    std::vector<State> goals;
    std::vector<State> startStates;
    readMapsFromJson(inputFile, dimx, dimy, startStates, goals, obstacles);

    std::cout << "Calculating Solution...\n";
    double timer = 0;

    Environment<Location, State, Action, double, Conflict, Constraint,
                Constraints>
        mapf(dimx, dimy, obstacles, dynamic_obstacles, goals);

    ECBS<State, Action, double, Conflict, Constraints,
         Environment<Location, State, Action, double, Conflict, Constraint,
                     Constraints>>
        cbsHybrid(mapf);
    std::vector<PlanResult<State, Action, double>> solution;
    Timer iterTimer;
    bool success = cbsHybrid.search(startStates, solution);
    iterTimer.stop();
    timer=iterTimer.elapsedSeconds();
    if (success)
    {
        std::cout << "\033[1m\033[32m Successfully find solution! \033[0m\n";

        // double makespan = 0, flowtime = 0, cost = 0;
        // for (const auto &s : solution)
        //     cost += s.cost;

        // for (size_t a = 0; a < solution.size(); ++a)
        // {
        //     // calculate makespan
        //     double current_makespan = 0;
        //     for (size_t i = 0; i < solution[a].actions.size(); ++i)
        //     {
        //         // some action cost have penalty coefficient

        //         if (solution[a].actions[i].second < Constants::dx[0])
        //             current_makespan += solution[a].actions[i].second;
        //         else if (solution[a].actions[i].first % 3 == 0)
        //             current_makespan += Constants::dx[0];
        //         else
        //             current_makespan += Constants::r * Constants::deltat;
        //     }
        //     flowtime += current_makespan;
        //     if (current_makespan > makespan)
        //         makespan = current_makespan;
        // }
        // std::cout << " Runtime: " << timer << std::endl
        //           << " Makespan:" << makespan << std::endl
        //           << " Flowtime:" << flowtime << std::endl
        //           << " cost:" << cost << std::endl;
        // output to file
        dumpOutputToJson(outputFile, solution, timer);
    }
    else
    {
        std::cout << "\033[1m\033[31m Fail to find paths \033[0m\n";
    }
}

void eval()
{
    std::string data_file = "./data/ecbs.csv";
    std::string input_folder = "./instances/one_shot/empty/";

    std::vector<int> num_agents = {10, 20, 30, 40, 50, 60};
    int num_instances = 30;
    std::ofstream out(data_file);
    out << "num_robots,makespan,cost,runtime,success_rate" << std::endl;
    for (auto n : num_agents)
    {
        double makespan_sum = 0;
        double cost_sum = 0;
        double runtime_sum = 0;
        double succ_sum = 0;

        for (int k = 0; k < num_instances; k++)
        {
            // std::string instance_prefix =
            std::string instance = input_folder + "agents_" + std::to_string(n) + "_" + std::to_string(k) + ".json";
            std::cout << instance << std::endl;
            size_t dimx, dimy;
            std::unordered_set<Location> obstacles;
            // std::multimap<int, State> dynamic_obstacles;
            std::vector<State> goals;
            std::vector<State> startStates;
            std::multimap<int, State> dynamic_obstacles;
            readMapsFromJson(instance, dimx, dimy, startStates, goals, obstacles);
            // readMapsFromYaml(instance, dimx, dimy, startStates, goals, obstacles);
            // std::cout << startStates.size() << "   " << goals.size() << std::endl;
            Environment<Location, State, Action, double, Conflict, Constraint,
                        Constraints>
                mapf(dimx, dimy, obstacles, dynamic_obstacles, goals);

            ECBS<State, Action, double, Conflict, Constraints,
                 Environment<Location, State, Action, double, Conflict, Constraint,
                             Constraints>>
                cbsHybrid(mapf);
            std::vector<PlanResult<State, Action, double>> solution;
            Timer iterTimer;
            bool success = cbsHybrid.search(startStates, solution);
            iterTimer.stop();
            if (success)
            {
                double makespan = 0, flowtime = 0, cost = 0;
                succ_sum++;
                for (const auto &s : solution)
                    cost += s.cost;
                for (size_t a = 0; a < solution.size(); ++a)
                {
                    // calculate makespan
                    double current_makespan = 0;
                    for (size_t i = 0; i < solution[a].actions.size(); ++i)
                    {
                        // some action cost have penalty coefficient

                        if (solution[a].actions[i].second < Constants::dx[0])
                            current_makespan += solution[a].actions[i].second;
                        else if (solution[a].actions[i].first % 3 == 0)
                            current_makespan += Constants::dx[0];
                        else
                            current_makespan += Constants::r * Constants::deltat;
                    }
                    flowtime += current_makespan;
                    if (current_makespan > makespan)
                        makespan = current_makespan;
                }
                makespan_sum += makespan;
                cost_sum += cost;
                runtime_sum += iterTimer.elapsedSeconds();
            }
        }

        if (succ_sum != 0)
        {
            out << std::to_string(n) << "," << std::to_string(makespan_sum / succ_sum)
                << "," << std::to_string(cost_sum / succ_sum)
                << "," << std::to_string(runtime_sum / succ_sum)
                << "," << std::to_string(succ_sum / num_instances) << std::endl;
        }
    }
    out.close();
}

int main(int argc, char *argv[])
{
    // eval();
    ECBS_exe(argc, argv);
}