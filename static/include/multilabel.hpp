#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <unordered_set>

#define USE_FIBONACCI_HEAP
#include <boost/heap/d_ary_heap.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/program_options.hpp>

#include "neighbor.hpp"
#include "planresult.hpp"

namespace libMultiRobotPlanning
{
    template <typename State, typename Action, typename Cost, typename Environment,
              typename StateHasher = std::hash<State>>

    class MultiLabelHybridAstar
    {
    public:
        MultiLabelHybridAstar(Environment &env, double w): m_env(env),m_w(w){}
        ~MultiLabelHybridAstar() {}

        // bool search(const State &startState, PlanResult<State, Action, Cost> &solution)
        // {
        //     solution.states.clear();
        //     solution.actions.clear();
        //     solution.cost = 0;
        //     openSet_t openSet;
        //     focalSet_t focalSet;
        //     std::unordered_map<uint64_t, fibHeapHandle_t, std::hash<uint64_t>> stateToHeap;
        //     std::unordered_set<uint64_t, std::hash<uint64_t>> closedSet;
        //     std::unordered_map<State, std::tuple<State, Action, Cost, Cost>,StateHasher>cameFrom;
        //     auto handle = openSet.push(Node(startState, Action(),
        //                                     m_env.admissibleHeuristic(startState), 0,0));

        //     stateToHeap.insert(std::make_pair<>(m_env.calcIndex(startState), handle));
        //     (*handle).handle = handle;
        //     focalSet.push(handle);
        //     std::vector<Neighbor<State, Action, Cost>> neighbors;
        //     neighbors.reserve(10);
        //     Cost bestFScore = (*handle).fScore;
        //     while()
        // }


  

    private:
        Environment &m_env;
        double m_w;
    };

}