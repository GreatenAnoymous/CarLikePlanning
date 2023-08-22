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
#include <cmath>
#include "config.hpp"
#define DEBUG_ENABLE 0
#define COLLISION_RANGE 8

namespace libMultiRobotPlanning
{
    template <typename State, typename Action, typename Cost, typename Environment>

    class PIBT
    {
    private:
        Environment &m_env;
        std::vector<PlanResult<State, Action, Cost>> solution;
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


        int time_steps;
        int max_timesteps = 500;
        bool solved = false;

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

        void checkCollision()
        {
            for (int i = 0; i < currentStates.size(); i++)
            {
                for (int j = i + 1; j < currentStates.size(); j++)
                {
                    if (currentStates[i].agentCollision(currentStates[j]))
                    {
                        std::cout << "found collision between agent " << i << "  and  " << j << std::endl;
                    }
                }
            }
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

        void addGreedyNeighbor(size_t agentId, State &curr, std::vector<Neighbor<State, Action, double>> &neighbors)
        {
            Neighbor<State, Action, Cost> nbr(curr, 0, 0);
            if (greedy_paths[agentId].empty() == false)
            {
                nbr = greedy_paths[agentId].back();
            }
            else
            {
                nbr.state = m_env.getGoal(agentId);
                nbr.cost = 0;
            }
            nbr.state.time = curr.time + 1;
            for (auto it = neighbors.begin(); it != neighbors.end(); it++)
            {
                if (it->state == nbr.state)
                {
                    neighbors.erase(it);
                    break;
                }
            }
            nbr.state.greedy = 1;
            neighbors.push_back(nbr);
        }

        void moveRobot(size_t agentId, Neighbor<State, Action, Cost> &nbr)
        {
            nextPlan[agentId] = nbr.state;
            m_env.updateHistory(agentId, nbr.state);
            solution[agentId].states.push_back({nbr.state, nbr.cost});
            solution[agentId].actions.push_back({nbr.action, nbr.cost});
            solution[agentId].cost += nbr.cost;
            lastActions[agentId] = nbr.action;
            if (nbr.state.greedy == 1)
            {
                if (greedy_paths[agentId].empty() == false)
                    greedy_paths[agentId].pop_back();
            }
            else
            {
                auto curr = currentStates[agentId];
                auto act = nbr.action;
                if (act < 3)
                    act += 3;
                if (act >= 3 and act <= 5)
                    act -= 3;
                auto g = Constants::dx[0];
                if (act % 3 != 0)
                { // penalize turning
                    g = g * Constants::penaltyTurning;
                }
                if ((act < 3 && nbr.action >= 3) || (nbr.action < 3 && act >= 3))
                {
                    // penalize change of direction
                    g = g * Constants::penaltyCOD;
                }
                if (act >= 3)
                { // backwards
                    g = g * Constants::penaltyReversing;
                }
                greedy_paths[agentId].push_back(Neighbor<State, Action, Cost>(curr, act, g));
            }
            double dist;
            auto goal = m_env.getGoal(agentId);
            auto s = currentStates[agentId];
            double dyaw = pow(cos(s.yaw) - cos(goal.yaw), 2) + pow(sin(s.yaw) - sin(goal.yaw), 2);
            dist = sqrt(pow(s.x - goal.x, 2) + pow(s.y - goal.y, 2) + dyaw);
            if (dist < 1e-2)
            {
                greedy_paths[agentId].clear();
            }
        }

        bool arrivedGoals()
        {
            bool flag = true;
            for (int i = 0; i < m_env.getNumAgents(); i++)
            {
                auto goal = m_env.getGoal(i);
                auto curr = currentStates[i];

                auto dyaw = pow(cos(goal.yaw) - cos(curr.yaw), 2) + pow(sin(goal.yaw) - sin(curr.yaw), 2);
                auto dist = sqrt(pow(goal.x - curr.x, 2) + pow(goal.y - curr.y, 2) + dyaw);
                if (dist > 1e-1)
                    flag = false;
                else
                {
                    distance_heuristic[i] = 0;
                    lastGoalReleasedTime[i] = time_steps;
                    m_env.clearHistory(i);
                }
            }
            return flag;
        }


        /* data */
    public:
        PIBT(Environment &environment, std::vector<State> &initialStates) : m_env(environment)
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
            solution = std::vector<PlanResult<State, Action, Cost>>(num_agents, PlanResult<State, Action, Cost>());
            update_priorities();
        }

        bool isSolved()
        {
            return solved;
        }

        double calcArrivalRate()
        {
            double count = 0;
            for (int i = 0; i < solution.size(); i++)
            {
                auto goal = m_env.getGoal(i);
                auto curr = currentStates[i];

                auto dyaw = pow(cos(goal.yaw) - cos(curr.yaw), 2) + pow(sin(goal.yaw) - sin(curr.yaw), 2);
                auto dist = sqrt(pow(goal.x - curr.x, 2) + pow(goal.y - curr.y, 2) + dyaw);
                if (dist < 1e-1)
                    count++;
            }
            return count / ((double)solution.size());
        }

        void setStates(int agent, State &state)
        {
            assert(agent < currentStates.size());
            currentStates[agent] = state;
        }

        void PIBT_solve()
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

            // return;
            while (time_steps <= max_timesteps)
            {
                update_priorities();
#if DEBUG_ENABLE
                std::cout << "current time step=" << time_steps << std::endl;
#endif
                time_steps++;
                PIBT_loop();
                // std::cout << "current state=" << currentStates[0] << "  goal state= " << m_env.getGoal(0) << std::endl;
                if (arrivedGoals() == true)
                {
                    solved = true;
                    std::cout << "All robots have arrived the goal states" << std::endl;
                    break;
                }
#if DEBUG_ENABLE
                checkCollision();
                std::cout << "===================" << std::endl;
#endif
            }
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

        std::vector<PlanResult<State, Action, Cost>> getSolution()
        {
            return solution;
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
            // m_env.markHeadonNeighbor(agent, currentStates[agent], neighbors, parent_agent, parent_act);
            m_env.sortNeighbors(neighbors, agent);

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
                std::cout << "nbr " << nbr.state << "  greedy=" << nbr.state.greedy << "  act= " << nbr.action << "  f= " << m_env.admissibleHeuristic(nbr.state, agent) + 0.3 * nbr.cost * (1 + m_env.getStateHistory(agent, nbr.state)) << "admissible H=  " << m_env.admissibleHeuristic(nbr.state, agent) << "   cost=" << nbr.cost << "  state history=" << m_env.getStateHistory(agent, nbr.state) << std::endl;
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
                // for (size_t k = 0; k < undecided.size();)
                // {
                //     int ak = undecided[k];
                //     auto sk = currentStates[ak];

                //     if (sk.agentCollision(next))
                //     {
                //         // robust = false;
                //         std::cout << "parent agent " << agent << "  to  " << next << " trying to move child agent " << ak << std::endl;
                //         if (PIBT_func(ak, agent, {next.x, next.y, next.yaw}) == false)
                //         {
                //             valid = false;
                //             break;
                //         }
                //     }
                //     else
                //     {
                //         k++;
                //     }
                // }
                if (valid)
                {
                    nextPlan[agent] = next;
                    m_env.updateHistory(agent, next);
                    solution[agent].states.push_back({next, nbr.cost});
                    solution[agent].actions.push_back({nbr.action, nbr.cost});
                    solution[agent].cost += nbr.cost;
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
            // if(parent_agent==-1){
            //     nextPlan[agent] = currentStates[agent];
            //     occupied.insert(currentStates[agent]);
            //     // lastActions[agent] = nbr.action;
            //     return false;
            // }
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