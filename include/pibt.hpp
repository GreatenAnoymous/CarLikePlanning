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
    template <typename State, typename Action, typename Cost, typename Conflict,
              typename Constraints, typename Environment>

    class PIBT
    {
    private:
        Environment &m_env;
        std::vector<PlanResult<State, Action, Cost>> solution;
        std::set<State> occupied;

        std::function<bool(int,int)> compareAgent;
        std::priority_queue<int,std::vector<int>,sc

        /* data */
    public:
        PIBT(Environment &environment);

        void PUBT_main();
        void PIBT_func(int agent);
        



        ~PIBT();
    };
}