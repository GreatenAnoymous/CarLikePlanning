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

    class FocalHybridAStar
    {
    public:
        FocalHybridAStar(Environment &environment, float w = 1.5) : m_env(environment), m_w(w) {}
        ~FocalHybridAStar() {}

        bool search(const State &startState, PlanResult<State, Action, Cost> &solution)
        {
            solution.states.clear();
            solution.actions.clear();
            solution.cost = 0;
            openSet_t openSet;
            focalSet_t focalSet;
            std::unordered_map<uint64_t, fibHeapHandle_t, std::hash<uint64_t>> stateToHeap;
            std::unordered_set<uint64_t, std::hash<uint64_t>> closedSet;
            std::unordered_map<State, std::tuple<State, Action, Cost, Cost>,
                               StateHasher>
                cameFrom;

            auto handle = openSet.push(Node(startState, Action(),
                                            m_env.admissibleHeuristic(startState), 0,0));

            stateToHeap.insert(std::make_pair<>(m_env.calcIndex(startState), handle));
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
                    solution.states.push_back(std::make_pair<>(startState, 0));
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
                m_env.getNeighbors(current.state, current.action, neighbors);

                //
                for (const Neighbor<State, Action, Cost> &neighbor : neighbors)
                {
                    if (closedSet.find(m_env.calcIndex(neighbor.state)) ==
                        closedSet.end())
                    { // not in closed set
                        Cost tentative_gScore = current.gScore + neighbor.cost;
                        auto iter = stateToHeap.find(m_env.calcIndex(neighbor.state));
                        if (iter == stateToHeap.end())
                        { // Discover a new node
                            Cost fScore =
                                tentative_gScore + m_env.admissibleHeuristic(neighbor.state);

                            Cost focalHeuristic =
                                current.focalHeuristic +
                                m_env.focalStateHeuristic(neighbor.state, tentative_gScore) +
                                m_env.focalTransitionHeuristic(current.state, neighbor.state,
                                                               current.gScore,
                                                               tentative_gScore);
                            auto handle = openSet.push(Node(neighbor.state, neighbor.action,
                                                                 fScore, tentative_gScore, focalHeuristic));
                            (*handle).handle = handle;
                            if (fScore <= bestFScore * m_w)
                            {
                                // std::cout << "focalAdd: " << *handle << std::endl;
                                focalSet.push(handle);
                            }
                            stateToHeap.insert(

                                std::make_pair<>(m_env.calcIndex(neighbor.state), handle));
                            m_env.onDiscover(neighbor.state, fScore, tentative_gScore);
                            // std::cout << "  this is a new node " << fScore << "," <<
                            // tentative_gScore << std::endl;
                        }
                        else
                        {
                            auto handle = iter->second;
                            // std::cout << "  this is an old node: " << tentative_gScore << ","
                            // << (*handle).gScore << std::endl;
                            // We found this node before with a better path
                            if (tentative_gScore >= (*handle).gScore)
                            {
                                continue;
                            }

                            // update f and gScore
                            // Cost last_gScore = (*handle).gScore;
                            Cost last_fScore = (*handle).fScore;
                            Cost delta = (*handle).gScore - tentative_gScore;
                            (*handle).gScore = tentative_gScore;
                            (*handle).fScore -= delta;
                            (*handle).state = neighbor.state;
                            openSet.increase(handle);
                            m_env.onDiscover(neighbor.state, (*handle).fScore,
                                             (*handle).gScore);
                            if ((*handle).fScore <= bestFScore * m_w &&
                                last_fScore > bestFScore * m_w)
                            {
                                // std::cout << "focalAdd: " << *handle << std::endl;
                                focalSet.push(handle);
                            }
                        }

                        // Best path for this node so far
                        // TODO: this is not the best way to update "cameFrom", but otherwise
                        // default c'tors of State and Action are required
                        cameFrom.erase(neighbor.state);
                        cameFrom.insert(std::make_pair<>(
                            neighbor.state,
                            std::make_tuple<>(current.state, neighbor.action, neighbor.cost,
                                              tentative_gScore)));
                    }
                }
            }
            openSet.clear();
            return false;
        }

    private:
        struct Node;

#ifdef USE_FIBONACCI_HEAP
        typedef typename boost::heap::fibonacci_heap<Node> openSet_t;
        typedef typename openSet_t::handle_type fibHeapHandle_t;
// typedef typename boost::heap::fibonacci_heap<fibHeapHandle_t,
// boost::heap::compare<compareFocalHeuristic> > focalSet_t;
#else
        typedef typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
                                                 boost::heap::mutable_<true>>
            openSet_t;
        typedef typename openSet_t::handle_type fibHeapHandle_t;
// typedef typename boost::heap::d_ary_heap<fibHeapHandle_t,
// boost::heap::arity<2>, boost::heap::mutable_<true>,
// boost::heap::compare<compareFocalHeuristic> > focalSet_t;
#endif
        struct Node
        {
           
            Node(const State &state, Action action, Cost fScore, Cost gScore, Cost focalHeuristic) : state(state), action(action),
                                                                                           fScore(fScore),
                                                                                           gScore(gScore),
                                                                                           focalHeuristic(focalHeuristic) {}

            friend std::ostream &operator<<(std::ostream &os, const Node &node)
            {
                os << "state: " << node.state << " fScore: " << node.fScore
                   << " gScore: " << node.gScore << " focal: " << node.focalHeuristic;
                return os;
            }

            State state;
            Action action;
            Cost fScore;
            Cost gScore;
            Cost focalHeuristic;

            bool operator<(const Node &other) const
            {
                // Sort order
                // 1. lowest fScore
                // 2. highest gScore

                // Our heap is a maximum heap, so we invert the comperator function here
                if (fScore != other.fScore)
                {
                    return fScore > other.fScore;
                }
                else
                {
                    return gScore < other.gScore;
                }
            }

            fibHeapHandle_t handle;
        };

        struct compareFocalHeuristic
        {
            bool operator()(const fibHeapHandle_t &h1,
                            const fibHeapHandle_t &h2) const
            {
                // Sort order (see "Improved Solvers for Bounded-Suboptimal Multi-Agent
                // Path Finding" by Cohen et. al.)
                // 1. lowest focalHeuristic
                // 2. lowest fScore
                // 3. highest gScore

                // Our heap is a maximum heap, so we invert the comperator function here
                if ((*h1).focalHeuristic != (*h2).focalHeuristic)
                {
                    return (*h1).focalHeuristic > (*h2).focalHeuristic;
                    // } else if ((*h1).fScore != (*h2).fScore) {
                    //   return (*h1).fScore > (*h2).fScore;
                }
                else if ((*h1).fScore != (*h2).fScore)
                {
                    return (*h1).fScore > (*h2).fScore;
                }
                else
                {
                    return (*h1).gScore < (*h2).gScore;
                }
            }
        };

#ifdef USE_FIBONACCI_HEAP
        typename boost::heap::fibonacci_heap<Node>::handle_type handle;
#else
        typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
                                         boost::heap::mutable_<true>>::handle_type
            handle;
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
        Environment &m_env;
        double m_w;
    };
};
