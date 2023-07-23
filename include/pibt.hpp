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
            distance_heuristic=std::vector<double>(m_env.getNumAgents(),0);
            for (size_t i = 0; i < m_env.getNumAgents(); i++)
            {
                agents_list.push_back(i);
                distance_heuristic[i] = m_env.admissibleHeuristic(currentStates[i], i);
            }

            auto comparator = [&](size_t ai, size_t aj)
            {
                if ((time_steps - lastGoalReleasedTime[ai]) != (time_steps - lastGoalReleasedTime[aj]))
                {
                    return ((time_steps - lastGoalReleasedTime[ai]) > (time_steps - lastGoalReleasedTime[aj]));
                }
                return distance_heuristic[ai] > distance_heuristic[aj];
            };

            std::sort(agents_list.begin(), agents_list.end(), comparator);


            // debug agent list
            // for(auto agent:agents_list){
            //     std::cout<<agent<<"  ";
            // }
            // std::cout<<std::endl;
        }

   

        bool arrivedGoals(){
            for(int i=0;i<m_env.getNumAgents();i++){
                auto goal=m_env.getGoal(i);
                auto curr=currentStates[i];
                if((goal.x!=curr.x) or (goal.y!=curr.y) or (goal.yaw!=curr.yaw)){
                    return false;
                }
            
            }
            return true;
        }

        void sortNeighbors(std::vector<Neighbor<State, Action, double>> &neighbors,int agent)
        {
            auto comparator=[&](Neighbor<State, Action, double>&a, Neighbor<State, Action, double>&b){
                return m_env.admissibleHeuristic(a.state,agent)>m_env.admissibleHeuristic(b.state,agent);
            };
            std::sort(neighbors.begin(),neighbors.end(),comparator);
        }

        /* data */
    public:
        PIBT(Environment &environment, std::vector<State> &initialStates) : m_env(environment)
        {
            auto num_agents = initialStates.size();
            currentStates = initialStates;
            nextPlan = currentStates;
            lastActions = std::vector<Action>(num_agents, 0);
            lastGoalReleasedTime = std::vector<int>(num_agents, 0);
            solution=std::vector<PlanResult<State,Action,Cost>>(num_agents,PlanResult<State,Action,Cost>());
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
                std::cout<<"current time step="<<time_steps<<std::endl;
                time_steps++;
                PIBT_loop();
                std::cout<<"current state="<<currentStates[0]<<"  goal state= "<<m_env.getGoal(0)<<std::endl;
                if (arrivedGoals() == true)
                {
                    std::cout << "All robots have arrived the goal states" << std::endl;
                    break;
                }
                std::cout<<"==================="<<std::endl;
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

            //update
            currentStates=nextPlan;
        }

        

        bool PIBT_func(int agent, int parent_agent = -1)
        {
            
            std::cout<<"PIBT calls to agent "<<agent<<std::endl;
            if(std::find(undecided.begin(),undecided.end(),agent)==undecided.end()){
                std::cout<<"agent "<<agent<<" not in undecided"<<std::endl;
                for(auto agent :undecided){
                    std::cout<<agent<<"  ";
                }
                std::cout<<std::endl;
            }

            assert(std::find(undecided.begin(),undecided.end(),agent)!=undecided.end());
            undecided.erase(std::remove(undecided.begin(), undecided.end(), agent));
            std::vector<Neighbor<State, Action, double>> neighbors;
            m_env.getNeighbors(currentStates[agent], lastActions[agent], neighbors);

            // to do: sort neighbors based on distance heuristic
            
            sortNeighbors(neighbors,agent);
            for(auto nbr:neighbors){
                std::cout<<"nbr "<<nbr.state<<"  ";
            }
            std::cout<<std::endl;
            for (auto nbr : neighbors)
            {

                auto next = nbr.state;

                //avoid colliding with parent agents
                if (parent_agent != -1 and currentStates[parent_agent].agentCollision(next) == true){
                    std::cout<<"avoid collision with parent agent "<<parent_agent<<std::endl;
                    continue;
                }
                bool valid = true;
                for (size_t k=0;k<undecided.size();)
                {
                    int ak=undecided[k];
                    auto sk = currentStates[ak];
                    if (sk.agentCollision(next))
                    {                                          
                        if (PIBT_func(ak, agent) == false)
                        {
                            valid = false;
                            break;
                        }
                    }else{
                        k++;
                    }
                }
                if (valid)
                {
                    nextPlan[agent] = next;
                    solution[agent].states.push_back({next,nbr.cost});
                    solution[agent].actions.push_back({nbr.action,nbr.cost});
                    solution[agent].cost+=nbr.cost;
                    lastActions[agent]=nbr.action;
                    occupied.insert(next);
                    return true;
                }
            }
            return false;
        }

       
    };
}