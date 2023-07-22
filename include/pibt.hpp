#pragma once

#include <chrono>
#include <map>
#include <vector>
#include <queue>
#include<set>
#include "neighbor.hpp"
#include "planresult.hpp"
#include <functional>
namespace libMultiRobotPlanning
{
    template <typename State, typename Action, typename Cost, typename Environment>

    class PIBT
    {
    private:
        Environment &m_env;
        std::vector<PlanResult<State, Action, Cost>> solution;
        std::set<State> occupied;
        std::vector<size_t> undecided;

        std::vector<State> currentStates;
        std::vector<State> nextPlan;
        std::vector<Action> lastActions;
        std::vector<int> lastGoalReleasedTime;
        std::vector<double> distance_heuristic;
        std::vector<size_t> agents_list;

        // std::priority_queue<int,std::vector<int>,sc
        int time_steps;
        int max_timesteps=300;

        void update_priorities(){
            agents_list.clear();
            for(size_t i=0;i<m_env.getNumAgents();i++){
                agents_list.push_back(i);
                distance_heuristic[i]=m_env.admissibleHeuristic(currentStates[i],i);
            }


            auto comparator=[&](size_t ai,size_t aj){
                if((time_steps-lastGoalReleasedTime[ai])!=(time_steps-lastGoalReleasedTime[aj])){
                    return ((time_steps-lastGoalReleasedTime[ai])>(time_steps-lastGoalReleasedTime[aj]));
                }
                return distance_heuristic[ai]>distance_heuristic[aj];
            };

            std::sort(agents_list.begin(),agents_list.end(),comparator);
        }

        bool arrivedGoals();

        void sortNeighbors( std::vector<Neighbor<State, Action, double>> &neighbors){


        }

        /* data */
    public:
        PIBT(Environment &environment,std::vecotr<State> &initialStates):m_env(environment){
            auto num_agents=initialStates.size();
            currentStates=initialStates;
            nextPlan=currentStates;
            lastActions=std::vector<Action>(num_agents,0);
            lastGoalReleasedTime=std::vector<int>(num_agents,0);
            update_priorities();

            
        }

        void setStates(int agent,State &state){
            assert(agent<currentStates.size());
            currentStates[agent]=state;
        }




        void PUBT_solve(){
            time_steps=0;
            while(time_steps<=max_timesteps){
                time_steps++;
                PIBT_loop();
                if(arrivedGoals()==true) {
                    std::cout<<"All robots have arrived the goal states"<<std::endl;
                    break;
                }
            }
        }
        void PIBT_loop(){
            //initialize undecided and sort the agents
            undecided=agents_list;
            occupied.clear();
            while (undecided.empty()==false)
            {
                int agent=undecided[0];
                PIBT_func(agent);
            }
            
        }
        
        bool PIBT_func(int agent,int parent_agent=-1){
          
            
            undecided.erase(std::remove(undecided.begin(),undecided.end(),agent));
            std::vector<Neighbor<State, Action, double>> neighbors;
            m_env.getNeighbors(currentStates[agent],lastActions[agent],neighbors);
            

            // to do: sort neighbors based on distance heuristic
        
            for(auto nbr:neighbors){
                
                auto next=nbr.state;
                if(agent !=-1 and currentStates[parent_agent].agentCollision(next)==true) continue;
                bool valid=true;
                for(auto ak :undecided){
                    auto sk=currentStates[ak];
                    if(sk.agentCollision(next)){
                        if(PIBT_func((ak,agent))==false){
                            valid=false;
                            break;
                        }
                    }
                }
                if(valid){
                    nextPlan[ak]=next;
                    occupied.insert(next);
                    return true;
                }
            }
            return false;

        }


        ~PIBT();
    };
}