#pragma once

#include <chrono>
#include <map>
#include <vector>
#include <queue>
#include <set>
#include <unordered_set>
#include "neighbor.hpp"
#include "planresult.hpp"
#include <functional>
#include <algorithm>
#include <cassert>
#include <iostream>
#include "json.hpp"
#include <cmath>
#include "config.hpp"
#include <fstream>
#include "timer.hpp"

// #define DEBUG_ENABLE 1
#define COLLISION_RANGE 8

namespace libMultiRobotPlanning
{
    template <typename State, typename Action, typename Cost, typename Environment>

    class PIBT
    {
    private:
        Environment &m_env;
        // std::vector<PlanResult<State, Action, Cost>> solution;
        std::vector<std::vector<State>> trajectories;
        std::unordered_set<State> occupied;
        std::vector<size_t> undecided;
        std::unordered_set<size_t> undecided_set;
        std::vector<std::vector<size_t>> nearby_agents;
        std::vector<std::vector<Neighbor<State, Action, Cost>>> greedy_paths;

        std::vector<State> currentStates;
        std::vector<State> lastStates;
        std::vector<State> nextPlan;
        std::vector<Action> lastActions;
        std::vector<int> lastGoalReleasedTime;
        std::vector<double> distance_heuristic;
        std::vector<size_t> agents_list;
        size_t num_tasks_finished = 0;
        std::vector<size_t> labels;
        std::vector<std::vector<State>> &all_tasks;

        std::vector<std::map<int, float>> history;

        // std::priority_queue<int,std::vector<int>,sc
        int time_steps;
        int max_timesteps = MAXSIMTIME;
        bool solved = false;
        double runtime = 0;

        void updateHistory(int agent, State &curr)
        {
            int sIndex = m_env.calcIndexWithoutTIme(curr);
            if (history[agent].find(sIndex) == history[agent].end())
            {
                history[agent][sIndex] = 0;
            }
            else
            {
                history[agent][sIndex] = Constants::gammaHistory * history[agent][sIndex] + 1;
            }
            // std::cout << "update history  " << agent << "   " << sIndex << "   " << history[agent][sIndex] << std::endl;
        }

        float getStateHistory(int agent, State &curr)
        {
            int sIndex = m_env.calcIndexWithoutTIme(curr);
            if (m_env.arrivedGoal(curr, agent))
                return 0;
            if (history[agent].find(sIndex) == history[agent].end())
            {
                history[agent][sIndex] = 0;
            }
            return history[agent][sIndex];
        }

        void update_priorities()
        {
            agents_list.clear();
            // distance_heuristic = std::vector<double>(m_env.getNumAgents(), 0);
            for (size_t i = 0; i < m_env.getNumAgents(); i++)
            {
                agents_list.push_back(i);
            }

            sort_list(agents_list);
#if DEBUG_ENABLE
            std::cout << "debug agent list" << std::endl;
            for (auto agent : agents_list)
            {
                std::cout << agent << "  ";
            }
            std::cout << std::endl;
#endif
        }

        void sort_list(std::vector<size_t> &list)
        {
            for (auto k : list)
            {
                distance_heuristic[k] = m_env.admissibleHeuristic(currentStates[k], k);
            }
            auto comparator = [&](size_t ai, size_t aj)
            {
                if ((time_steps - lastGoalReleasedTime[ai]) != (time_steps - lastGoalReleasedTime[aj]))
                {
                    return ((time_steps - lastGoalReleasedTime[ai]) > (time_steps - lastGoalReleasedTime[aj]));
                }
                return distance_heuristic[ai] < distance_heuristic[aj];
            };

            std::sort(list.begin(), list.end(), comparator);
        }

        bool checkCollision()
        {
            for (int i = 0; i < currentStates.size(); i++)
            {
                for (int j = i + 1; j < currentStates.size(); j++)
                {
                    if (currentStates[i].agentCollision(currentStates[j]))
                    {
                        std::cout << "found collision between agent " << i << "  and  " << j << std::endl;
                        return true;
                    }
                }
            }
            return false;
        }

        void find_initial_paths()
        {
            size_t num_agents = m_env.getNumAgents();
            greedy_paths = std::vector<std::vector<Neighbor<State, Action, Cost>>>(num_agents, std::vector<Neighbor<State, Action, Cost>>());
            for (size_t i = 0; i < num_agents; i++)
            {
                std::cout << "Find Initial path for agent " << i << std::endl;
                m_env.singleRobotPathPlanning(currentStates[i], i, greedy_paths[i]);
                std::cout << "HYbrid A* completed" << std::endl;
                exit(0);
            }
        }

        void compute_nearby_agents()
        {
            size_t num_agents = m_env.getNumAgents();
            nearby_agents = std::vector<std::vector<size_t>>(num_agents, std::vector<size_t>());
            for (size_t i = 0; i < num_agents; i++)
            {
                for (size_t j = i + 1; j < num_agents; j++)
                {
                    if (sqrt(pow(currentStates[i].x - currentStates[j].x, 2) + pow(currentStates[j].y - currentStates[j].y, 2)) < COLLISION_RANGE)
                    {
                        nearby_agents[i].push_back(j);
                        nearby_agents[j].push_back(i);
                    }
                }
            }
        }

        void sortNeighbors(std::vector<Neighbor<State, Action, double>> &neighbors, int agent)
        {
            std::random_device rd;
            std::default_random_engine engine(rd());
            std::uniform_real_distribution<double> distribution(0.0, 1.0);
            double bonus = Constants::greedyBonus;

            auto comparator = [&](Neighbor<State, Action, double> &a, Neighbor<State, Action, double> &b)
            {
                // auto scoreA = m_env.admissibleHeuristic(a.state, agent) + Constants::oneStepWeight * a.cost * (1 + getStateHistory(agent, a.state) - bonus * a.state.greedy);
                // auto scoreB = m_env.admissibleHeuristic(b.state, agent) + Constants::oneStepWeight * b.cost * (1 + getStateHistory(agent, b.state) - bonus * b.state.greedy);
                // auto scoreA = admissibleHeuristic(a.state, agent) + Constants::oneStepWeight * a.cost * (1 - bonus * a.state.greedy);
                // auto scoreB = admissibleHeuristic(b.state, agent) + Constants::oneStepWeight * b.cost * (1 - bonus * b.state.greedy);
                auto scoreA = m_env.admissibleHeuristic(a.state, agent) + Constants::oneStepWeight * a.cost * (1 + getStateHistory(agent, a.state) - bonus * a.state.greedy + Constants::penaltyHeadon * a.state.headon);
                auto scoreB = m_env.admissibleHeuristic(b.state, agent) + Constants::oneStepWeight * b.cost * (1 + getStateHistory(agent, b.state) - bonus * b.state.greedy + Constants::penaltyHeadon * b.state.headon);
                if (scoreA != scoreB)
                    return scoreA < scoreB;
                return false;
            };
            std::sort(neighbors.begin(), neighbors.end(), comparator);
        }

        /* data */
    public:
        PIBT(Environment &environment, std::vector<std::vector<State>> &all_tasks) : m_env(environment), all_tasks(all_tasks)
        {
        }

        void setInitialStates(std::vector<State> &initialStates)
        {

            auto num_agents = initialStates.size();
            currentStates = initialStates;
            nextPlan = currentStates;
            lastActions = std::vector<Action>(num_agents, 6);
            lastGoalReleasedTime = std::vector<int>(num_agents, 0);
            distance_heuristic = std::vector<double>(num_agents, 0);
            for (int i = 0; i < num_agents; i++)
            {
                distance_heuristic[i] = m_env.admissibleHeuristic(initialStates[i], i);
            }
            trajectories = std::vector<std::vector<State>>(num_agents, std::vector<State>());
            for (auto k = 0; k < num_agents; k++)
            {
                trajectories[k].push_back(initialStates[k]);
            }
            labels = std::vector<size_t>(num_agents, 0);
            update_priorities();
            history = std::vector<std::map<int, float>>(num_agents, std::map<int, float>());
        }

        bool isSolved()
        {
            return solved;
        }

        void dumpOutputToJson(std::string outputFile)
        {
            nlohmann::json output;
            if (SAVE_PATH)
            {
                std::vector<std::vector<std::vector<double>>> paths;
                for (auto &traj : trajectories)
                {
                    std::vector<std::vector<double>> path;
                    for (const auto &state : traj)
                    {
                        path.push_back({state.x, state.y, state.yaw});
                    }
                    paths.push_back(path);
                }
                output["paths"] = paths;
            }
            output["num_tasks_finished"] = num_tasks_finished;
            output["runtime"] = runtime;

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

        void setStates(int agent, State &state)
        {
            assert(agent < currentStates.size());
            currentStates[agent] = state;
        }

        void PIBT_sim()
        {
            std::cout << " starting to solve" << std::endl;
            time_steps = 0;
            // find_initial_paths();
            // for (size_t i = 0; i < m_env.getNumAgents(); i++)
            // {
            //     solution[i].states.push_back({currentStates[i], 0});
            //     for (int k = greedy_paths[i].size() - 1; k >= 0; k--)
            //     {
            //         // std::cout << k << "   " << greedy_paths[i].size() << std::endl;
            //         auto nbr = greedy_paths[i][k];
            //         // std::cout << "????" << std::endl;
            //         solution[i].states.push_back({nbr.state, nbr.cost});
            //         solution[i].actions.push_back({nbr.action, nbr.cost});
            //         solution[i].cost += nbr.cost;
            //     }
            // }
            Timer iterTimer;
            // return;
            while (time_steps <= max_timesteps)
            {

                // std::cout << "current time step=" << time_steps << std::endl;
                if (iterTimer.elapsedSeconds() > TIMEOUT)
                {
                    std::cout << "TIME OUT" << std::endl;
                }
                update_priorities();
                // #if DEBUG_ENABLE
                // std::cout << "current time step=" << time_steps << std::endl;
                // #endif
                time_steps++;

                PIBT_loop();
                execute();
                // std::cout << "current state=" << currentStates[0] << "  goal state= " << m_env.getGoal(0) << std::endl;
                // if (arrivedGoals() == true)
                // {
                //     solved = true;
                //     std::cout << "All robots have arrived the goal states" << std::endl;
                //     break;
                // }
#if DEBUG_ENABLE
                checkCollision();
                std::cout << "===================" << std::endl;
#endif
            }
            iterTimer.stop();
            runtime = iterTimer.elapsedSeconds();
        }

        void PIBT_loop()
        {
            // initialize undecided and sort the agents
            compute_nearby_agents();
            undecided = agents_list;
            undecided_set = std::unordered_set<size_t>(undecided.begin(), undecided.end());
            occupied.clear();
            while (undecided.empty() == false)
            {
                int agent = undecided[0];
                PIBT_func(agent);
            }

            // update

            currentStates = nextPlan;
        }

        void updateNewGoal(size_t agentId)
        {
            // if(agentId>=labels.size()){
            //     std::cout<<"agentId = "<<agentId<<"    labels size="<<labels.size()<<std::endl;
            // }
            labels[agentId]++;
            if (all_tasks[agentId].size() <= labels[agentId])
            {
                labels[agentId] = all_tasks[agentId].size() - 1;
            }
            auto goal = all_tasks[agentId][labels[agentId]];
            m_env.updateGoal(goal, agentId);
            history[agentId].clear();
        }

        void execute()
        {
            // update
            currentStates = nextPlan;

            for (int i = 0; i < m_env.getNumAgents(); i++)
            {
                if (m_env.arrivedGoal(currentStates[i], i))
                {
                    distance_heuristic[i] = 0;
                    lastGoalReleasedTime[i] = time_steps;
                    updateNewGoal(i);
                    num_tasks_finished++;
                }
                // trajectories[i].push_back(currentStates[i]);
            }
        }

        bool PIBT_func(int agent, int parent_agent = -1, std::vector<double> parent_state = {-1, -1, -1}, int parent_act = -1)
        {
#if DEBUG_ENABLE
            std::cout << "PIBT calls to agent " << agent << "  current=" << currentStates[agent] << std::endl;
#endif
            undecided.erase(std::remove(undecided.begin(), undecided.end(), agent));
            undecided_set.erase(agent);
            std::vector<Neighbor<State, Action, double>> neighbors;
            m_env.getNeighbors(currentStates[agent], lastActions[agent], neighbors, agent);
            // addGreedyNeighbor(agent, currentStates[agent], neighbors);
            // m_env.addGoalToNeighborIfClose(currentStates[agent], lastActions[agent], neighbors, agent);
            m_env.singleRobotMotionGuide(currentStates[agent], lastActions[agent], neighbors, agent);
            m_env.markHeadonNeighbor(agent, currentStates[agent], neighbors, parent_agent, parent_act);
            sortNeighbors(neighbors, agent);

            std::vector<int> nearby_undecided;
            std::vector<State> nearby_occupied;
            for (auto &s : occupied)
            {
                if (sqrt(pow(s.x - currentStates[agent].x, 2) + pow(s.y - currentStates[agent].y, 2)) < COLLISION_RANGE)
                {
                    nearby_occupied.push_back(s);
                }
            }

#if DEBUG_ENABLE
            for (auto nbr : neighbors)
            {
                std::cout << "nbr " << nbr.state << "  greedy=" << nbr.state.greedy << "  act= " << nbr.action << "  f= " << m_env.admissibleHeuristic(nbr.state, agent) + 0.3 * nbr.cost * (1 + getStateHistory(agent, nbr.state)) << "admissible H=  " << m_env.admissibleHeuristic(nbr.state, agent) << "   cost=" << nbr.cost << "  state history=" << getStateHistory(agent, nbr.state) << std::endl;
            }
            std::cout << std::endl;
#endif
            for (auto nbr : neighbors)
            {

                auto next = nbr.state;
                bool occupied_flag = false;
                for (auto s : nearby_occupied)
                {
                    if (s.agentCollision(next))
                    {

#if DEBUG_ENABLE
                        std::cout << "next  " << next << "  has already been occupied" << std::endl;
#endif
                        occupied_flag = true;
                        break;
                    }
                }
                if (occupied_flag)
                {
                    continue;
                }

                // avoid colliding with parent agents
                if (parent_agent != -1)
                {
                    if (currentStates[parent_agent].agentCollision(next) == true)
                    {
#if DEBUG_ENABLE
                        std::cout << "invalid due to collide with parent agent next plan" << std::endl;
#endif
                        continue;
                    }

                    auto ps = State(parent_state[0], parent_state[1], parent_state[2]);
                    if (ps.agentCollision(next) == true)
                    {
#if DEBUG_ENABLE
                        std::cout << "invalid due to collide with parent original state" << std::endl;
#endif
                        continue;
                    }
#if DEBUG_ENABLE
                    std::cout << "avoid collision with parent agent " << parent_agent << std::endl;
#endif
                }
                occupied.insert(next);
                bool valid = true;
                // bool robust = true;
                for (auto ak : nearby_agents[agent])
                {
                    if (undecided_set.find(ak) == undecided_set.end())
                        continue;
                    auto sk = currentStates[ak];
                    if (sk.agentCollision(next))
                    {
// robust = false;
#if DEBUG_ENABLE
                        std::cout << "parent agent " << agent << "  to  " << next << " trying to move child agent " << ak << std::endl;
#endif
                        if (PIBT_func(ak, agent, {next.x, next.y, next.yaw}, nbr.action) == false)
                        {
                            valid = false;
                            break;
                        }
                    }
                }

                if (valid)
                {
                    nextPlan[agent] = next;
                    updateHistory(agent, next);
                    trajectories[agent].push_back(next);
                    // trajectories[agent].push_back({nbr.action, nbr.cost});
                    // trajectories[agent] += nbr.cost;
                    lastActions[agent] = nbr.action;

                    // moveRobot(agent, nbr);
#if DEBUG_ENABLE
                    std::cout << "agent " << agent << "   move to " << next << std::endl;
#endif
                    return true;
                }
                else
                {
                    occupied.erase(next);
                }
            }

#if DEBUG_ENABLE
            std::cout << "PIBT for agent " << agent << "  invalid" << std::endl;
#endif
            undecided.push_back(agent);
            undecided_set.insert(agent);

            sort_list(undecided);

            return false;
        }
    };
}