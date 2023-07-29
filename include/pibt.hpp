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

        std::vector<State> currentStates;
        std::vector<State> lastStates;
        std::vector<State> nextPlan;
        std::vector<Action> lastActions;
        std::vector<int> lastGoalReleasedTime;
        std::vector<double> distance_heuristic;
        std::vector<size_t> agents_list;

        // std::priority_queue<int,std::vector<int>,sc
        int time_steps;
        int max_timesteps = 300;

        void update_priorities()
        {
            agents_list.clear();
            // distance_heuristic = std::vector<double>(m_env.getNumAgents(), 0);
            for (size_t i = 0; i < m_env.getNumAgents(); i++)
            {
                agents_list.push_back(i);
            }

            sort_list(agents_list);
            // debug agent list
            std::cout << "debug agent list" << std::endl;
            for (auto agent : agents_list)
            {
                std::cout << agent << "  ";
            }
            std::cout << std::endl;
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
                }
            }
            return flag;
        }

        void sortNeighbors(std::vector<Neighbor<State, Action, double>> &neighbors, int agent)
        {
            auto comparator = [&](Neighbor<State, Action, double> &a, Neighbor<State, Action, double> &b)
            {
                return m_env.admissibleHeuristic(a.state, agent) < m_env.admissibleHeuristic(b.state, agent);
            };
            std::sort(neighbors.begin(), neighbors.end(), comparator);
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

        void setStates(int agent, State &state)
        {
            assert(agent < currentStates.size());
            currentStates[agent] = state;
        }

        void PIBT_solve()
        {
            time_steps = 0;
            while (time_steps <= max_timesteps)
            {
                update_priorities();
                std::cout << "current time step=" << time_steps << std::endl;
                time_steps++;
                PIBT_loop();
                // std::cout << "current state=" << currentStates[0] << "  goal state= " << m_env.getGoal(0) << std::endl;
                if (arrivedGoals() == true)
                {
                    std::cout << "All robots have arrived the goal states" << std::endl;
                    break;
                }
                checkCollision();
                std::cout << "===================" << std::endl;
            }
        }
        void PIBT_loop()
        {
            // initialize undecided and sort the agents
            undecided = agents_list;
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

        bool PIBT_func(int agent, int parent_agent = -1, std::vector<double> parent_state = {-1, -1, -1})
        {

            std::cout << "PIBT calls to agent " << agent << std::endl;
            // if (std::find(undecided.begin(), undecided.end(), agent) == undecided.end())
            // {
            //     std::cout << "agent " << agent << " not in undecided" << std::endl;
            //     for (auto agent : undecided)
            //     {
            //         std::cout << agent << "  ";
            //     }
            //     std::cout << std::endl;
            // }

            // assert(std::find(undecided.begin(), undecided.end(), agent) != undecided.end());
            undecided.erase(std::remove(undecided.begin(), undecided.end(), agent));
            std::vector<Neighbor<State, Action, double>> neighbors;
            m_env.getNeighbors(currentStates[agent], lastActions[agent], neighbors, agent);

            // to do: sort neighbors based on distance heuristic
            for (auto nbr : neighbors)
            {
                std::cout << "nbr " << nbr.state << "  f= " << m_env.admissibleHeuristic(nbr.state, agent) + nbr.cost << "admissible H=  " << m_env.admissibleHeuristic(nbr.state, agent) << "   cost=" << nbr.cost << std::endl;
            }
            std::cout << std::endl;
            for (auto nbr : neighbors)
            {

                auto next = nbr.state;
                bool occupied_flag = false;
                for (auto s : occupied)
                {
                    if (s.agentCollision(next))
                    {
                        std::cout<<"next  "<<next<<"  has already been occupied"<<std::endl;
                        occupied_flag = true;
                        break;
                    }
                }
                if (occupied_flag){
                    continue;
                }

                // avoid colliding with parent agents
                if (parent_agent != -1)
                {
                    if (currentStates[parent_agent].agentCollision(next) == true){
                        std::cout<<"invalid due to collide with parent agent next plan"<<std::endl;
                        continue;
                    }
                        
                    auto ps = State(parent_state[0], parent_state[1], parent_state[2]);
                    if (ps.agentCollision(next) == true){
                        std::cout<<"invalid due to collide with parent original state"<<std::endl;
                        continue;
                    }
                        
                    // std::cout << "avoid collision with parent agent " << parent_agent << std::endl;
                }
                bool valid = true;
                // bool robust = true;
                for (size_t k = 0; k < undecided.size();)
                {
                    int ak = undecided[k];
                    auto sk = currentStates[ak];

                    if (sk.agentCollision(next))
                    {
                        // robust = false;
                        std::cout << "parent agent " << agent << "  to  " << next << " trying to move child agent " << ak << std::endl;
                        if (PIBT_func(ak, agent, {next.x, next.y, next.yaw}) == false)
                        {
                            valid = false;
                            break;
                        }
                    }
                    else
                    {
                        k++;
                    }
                }
                if (valid )
                {
                    nextPlan[agent] = next;
                    solution[agent].states.push_back({next, nbr.cost});
                    solution[agent].actions.push_back({nbr.action, nbr.cost});
                    solution[agent].cost += nbr.cost;
                    lastActions[agent] = nbr.action;
                    occupied.insert(next);
                    std::cout << "agent " << agent << "   move to " << next << std::endl;
                    return true;
                }
            }
            std::cout<<"PIBT for agent "<<agent<<"  invalid"<<std::endl;
            undecided.push_back(agent);
            sort_list(undecided);
            // throw std::runtime_error("PIBT explored all of the neighbors, none of them is collision-free");
            return false;
        }
    };
}