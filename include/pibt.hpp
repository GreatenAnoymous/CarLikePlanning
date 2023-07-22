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
        std::vector<int> undecided;

        std::vector<State> currentStates;
        std::vector<State> nextPlan;
        std::vector<Action> lastActions;

        std::function<bool(int,int)> compareAgent;
        // std::priority_queue<int,std::vector<int>,sc
    
        int time_steps;
        int max_timesteps=300;
        void update_priorities();

        bool arrivedGoals();
        /* data */
    public:
        PIBT(Environment &environment):m_env(environment){

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
            //to do: initialize undecided and sort the agents
            nextPlan.clear();
            while (undecided.empty()==false)
            {
                int agent=undecided.back();
                PIBT_func(agent);
            }
            
        }
        
        bool PIBT_func(int agent){
          
            
            undecided.erase(std::remove(undecided.begin(),undecided.end(),agent));
            std::vector<Neighbor<State, Action, double>> neighbors
            m_env.getNeighbors(currentStates[agent],lastActions[agent],neighbors);
            

            // to do: sort neighbors

            for(auto nbr:neighbors){
                auto next=nbr.state;
                if(m_env.stateValid(next)==false) continue;
                for(auto ak :undecided){
                    auto sk=currentStates[ak];
                    if(sk.agentCollision(next)){
                        if(PIBT_func((ak))==true){
                            nextPlan[ak]=
                        }
                    }
                }
            }

        }


        ~PIBT();
    };
}