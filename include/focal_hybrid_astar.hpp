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

    const double sub_optimality = 1.5;

    template <typename State, typename Action, typename Cost, typename Environment,
              typename StateHasher = std::hash<State>>

    class FocalHybridAStar
    {
    public:
        FocalHybridAStar(Environment &environment, float w = 1.5) : m_nev(environment), m_w(w) {}
        ~FocalHybridAStar() {}

        bool search(const State &startState, PlanResult<State, Action, Cost> &solution)
        {
            solution.states.clear();
            solution.actions.clear();
            solution.cost = 0;
            openSet_t openSet;
            focalSet_t focalSet;
            std::unordered_map<uint64_t, heapHandle_t, std::hash<uint64_t>> stateToHeap;
            std::unordered_set<uint64_t, std::hash<uint64_t>> closedSet;
            std::unordered_map<State, std::tuple<State, Action, Cost, Cost>,
                               StateHasher>
                cameFrom;

            auto handle = openSet.push(Node(startState, Action(),
                                            m_env.admissibleHeuristic(startState), initialCost));

            stateToHeap.insert(std::make_pair<>(startState, handle));
            (*handle).handle = handle;
            focalSet.push(handle);
            std::vector<Neighbor<State, Action, Cost>> neighbors;
            neighbors.reserve(10);
            Cost bestFScore = (*handle).fScore;
            while (!openSet.empty())
            {
                Cost oldBestFScore = bestFScore;
                bestFScore = openSet.top().fScore;
                // std::cout << "bestFScore: " << bestFScore << std::endl;
                if (bestFScore > oldBestFScore)
                {
                    // std::cout << "oldBestFScore: " << oldBestFScore << " newBestFScore:
                    // " << bestFScore << std::endl;
                    auto iter = openSet.ordered_begin();
                    auto iterEnd = openSet.ordered_end();
                    for (; iter != iterEnd; ++iter)
                    {
                        Cost val = iter->fScore;
                        if (val > oldBestFScore * m_w && val <= bestFScore * m_w)
                        {
                            const Node &n = *iter;
                            focalSet.push(n.handle);
                        }
                        if (val > bestFScore * m_w)
                        {
                            break;
                        }
                    }
                }

                auto currentHandle = focalSet.top();
                Node current = *currentHandle;
                m_env.onExpandNode(current.state, current.fScore, current.gScore);

                // check if it is the solution
                if (m_env.isSolution(current.state, current.gScore, cameFrom))
                {
                    solution.states.clear();
                    solution.actions.clear();
                    auto iter = cameFrom.find(m_env.getGoal());
                    solution.cost = std::get<3>(iter->second);
                    solution.fmin =
                        std::get<3>(iter->second) +
                        m_env.admissibleHeuristic(iter->first); // current.fScore;
                    while (iter != cameFrom.end())
                    {
                        // std::cout << " From " << std::get<0>(iter->second)
                        //           << " to Node:" << iter->first
                        //           << " with ACTION: " << std::get<1>(iter->second) << "
                        //           cost "
                        //           << std::get<2>(iter->second) << " g_score "
                        //           << std::get<3>(iter->second) << std::endl;
                        solution.states.push_back(
                            std::make_pair<>(iter->first, std::get<3>(iter->second)));
                        solution.actions.push_back(std::make_pair<>(
                            std::get<1>(iter->second), std::get<2>(iter->second)));
                        iter = cameFrom.find(std::get<0>(iter->second));
                    }
                    solution.states.push_back(std::make_pair<>(startState, initialCost));
                    std::reverse(solution.states.begin(), solution.states.end());
                    std::reverse(solution.actions.begin(), solution.actions.end());

                    openSet.clear();
                    return true;
                }

                focalSet.pop();
                openSet.erase(currentHandle);
                stateToHeap.erase(m_env.calcIndex(current.state));
                closedSet.insert(m_env.calcIndex(current.state));

                // traverse neighbors
                neighbors.clear();
                m_env.getNeighbors(current.state, neighbors);


                //
                for (const Neighbor<State, Action, Cost> &neighbor : neighbors){
                    
                }
        }

    private:
        struct FocalNode
        {
            friend std::ostream &operator<<(std::ostream &os, const Node &node)
            {
                os << "state: " << node.state << " fScore: " << node.fScore
                   << " gScore: " << node.gScore;
                return os;
            }

            State state;
            Action action;
            Cost fScore;
            Cost gScore;
        }

#ifdef USE_FIBONACCI_HEAP
        typename boost::heap::fibonacci_heap<Node>::handle_type handle;
#else
        typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
                                         boost::heap::mutable_<true>>::handle_type
            handle;
#endif

#ifdef USE_FIBONACCI_HEAP
        typedef typename boost::heap::fibonacci_heap<Node> openSet_t;
        typedef typename openSet_t::handle_type heapHandle_t;
#else
        typedef typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
                                                 boost::heap::mutable_<true>>
            openSet_t;
        typedef typename openSet_t::handle_type heapHandle_t;
#endif

#ifdef USE_FIBONACCI_HEAP
        // typedef typename boost::heap::fibonacci_heap<Node> openSet_t;
        // typedef typename openSet_t::handle_type fibHeapHandle_t;
        typedef typename boost::heap::fibonacci_heap<
            fibHeapHandle_t, boost::heap::compare<compareFocalHeuristic>>
            focalSet_t;
#else
        // typedef typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
        // boost::heap::mutable_<true> > openSet_t;
        // typedef typename openSet_t::handle_type fibHeapHandle_t;
        typedef typename boost::heap::d_ary_heap<
            fibHeapHandle_t, boost::heap::arity<2>, boost::heap::mutable_<true>,
            boost::heap::compare<compareFocalHeuristic>>
            focalSet_t;
#endif

    private:
        Environment &m_nev;
        double m_w;
    };

}