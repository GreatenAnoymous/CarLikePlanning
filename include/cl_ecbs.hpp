#pragma once

#include <map>

#include "focal_hybrid_astar.hpp"

namespace libMultiRobotPlanning
{
    template <typename State, typename Action, typename Cost, typename Conflict,
              typename Constraints, typename Environment>
    class ECBS
    {
    public:
        ECBS(Environment &environment, double w = 1.5) : m_env(environment), m_w(w) {}

        bool search(const std::vector<State> &initialStates,
                    std::vector<PlanResult<State, Action, Cost>> &solution)
        {
            HighLevelNode start;
            start.solution.resize(initialStates.size());
            start.constraints.resize(initialStates.size());
            start.cost = 0;
            start.LB = 0;
            start.id = 0;

            for (size_t i = 0; i < initialStates.size(); ++i)
            {
                std::cout << "searching for the initial solution for agent i" << i << std::endl;
                if (i < solution.size() && solution[i].states.size() > 1)
                {
                    std::cout << initialStates[i] << " " << solution[i].states.front().first
                              << std::endl;
                    assert(initialStates[i] == solution[i].states.front().first);
                    start.solution[i] = solution[i];
                    std::cout << "use existing solution for agent: " << i << std::endl;
                }
                else
                {
                    LowLevelEnvironment llenv(m_env, i, start.constraints[i],
                                              start.solution);
                    LowLevelSearch_t lowLevel(llenv, m_w);
                    bool success = lowLevel.search(initialStates[i], start.solution[i]);
                    if (!success)
                    {
                        return false;
                    }
                }
                start.cost += start.solution[i].cost;
                start.LB += start.solution[i].fmin;
            }
            start.focalHeuristic = m_env.focalHeuristic(start.solution);

            // std::priority_queue<HighLevelNode> open;
            openSet_t open;
            focalSet_t focal;

            handle_t handle = open.push(start);
            (*handle).handle = handle;
            focal.push(handle);

            Cost bestCost = (*handle).cost;

            solution.clear();
            int id = 1;
            while (!open.empty())
            {
// update focal list
#ifdef REBUILT_FOCAL_LIST
                focal.clear();
                Cost LB = open.top().LB;

                auto iter = open.ordered_begin();
                auto iterEnd = open.ordered_end();
                for (; iter != iterEnd; ++iter)
                {
                    float val = iter->cost;
                    // std::cout << "  cost: " << val << std::endl;
                    if (val <= LB * m_w)
                    {
                        const HighLevelNode &node = *iter;
                        focal.push(node.handle);
                    }
                    else
                    {
                        break;
                    }
                }
#else
                {
                    Cost oldBestCost = bestCost;
                    bestCost = open.top().cost;
                    // std::cout << "bestFScore: " << bestFScore << std::endl;
                    if (bestCost > oldBestCost)
                    {
                        // std::cout << "oldBestCost: " << oldBestCost << " bestCost: " <<
                        // bestCost << std::endl;
                        auto iter = open.ordered_begin();
                        auto iterEnd = open.ordered_end();
                        for (; iter != iterEnd; ++iter)
                        {
                            Cost val = iter->cost;
                            if (val > oldBestCost * m_w && val <= bestCost * m_w)
                            {
                                const HighLevelNode &n = *iter;
                                focal.push(n.handle);
                            }
                            if (val > bestCost * m_w)
                            {
                                break;
                            }
                        }
                    }
                }
#endif
// check focal list/open list consistency
#ifdef CHECK_FOCAL_LIST
                {
                    // focalSet_t focalSetGolden;
                    bool mismatch = false;
                    const auto &top = open.top();
                    Cost bestCost = top.cost;
                    auto iter = open.ordered_begin();
                    auto iterEnd = open.ordered_end();
                    for (; iter != iterEnd; ++iter)
                    {
                        const auto &s = *iter;
                        Cost val = s.cost;
                        if (val <= bestCost * m_w)
                        {
                            // std::cout << "should: " << s << std::endl;
                            // focalSetGolden.push(s.handle);
                            if (std::find(focal.begin(), focal.end(), s.handle) ==
                                focal.end())
                            {
                                std::cout << "focal misses: " << s << std::endl;
                                mismatch = true;
                            }
                        }
                        else
                        {
                            if (std::find(focal.begin(), focal.end(), s.handle) !=
                                focal.end())
                            {
                                std::cout << "focalSet shouldn't have: " << s << std::endl;
                                mismatch = true;
                            }
                            // break;
                        }
                    }
                    assert(!mismatch);
                    // assert(focalSet == focalSetGolden);
                }
#endif

                auto h = focal.top();
                HighLevelNode P = *h;
                m_env.onExpandHighLevelNode(P.cost);
                // std::cout << "expand: " << P << std::endl;

                focal.pop();
                open.erase(h);

                Conflict conflict;
                if (!m_env.getFirstConflict(P.solution, conflict))
                {
                    std::cout << "done; cost: " << P.cost << std::endl;
                    solution = P.solution;
                    return true;
                }

                // create additional nodes to resolve conflict
                std::cout << "Found conflict: " << conflict << std::endl;
                // std::cout << "Found conflict at t=" << conflict.time << " type: " <<
                // conflict.type << std::endl;

                std::map<size_t, Constraints> constraints;
                m_env.createConstraintsFromConflict(conflict, constraints);
                for (const auto &c : constraints)
                {
                    // std::cout << "Add HL node for " << c.first << std::endl;
                    size_t i = c.first;
                    std::cout << "create child with id " << id << std::endl;
                    HighLevelNode newNode = P;
                    newNode.id = id;
                    // (optional) check that this constraint was not included already
                    // std::cout << newNode.constraints[i] << std::endl;
                    // std::cout << c.second << std::endl;
                    assert(!newNode.constraints[i].overlap(c.second));

                    newNode.constraints[i].add(c.second);

                    newNode.cost -= newNode.solution[i].cost;
                    newNode.LB -= newNode.solution[i].fmin;

                    LowLevelEnvironment llenv(m_env, i, newNode.constraints[i],
                                              newNode.solution);
                    LowLevelSearch_t lowLevel(llenv, m_w);
                    bool success = lowLevel.search(initialStates[i], newNode.solution[i]);

                    if (success)
                    {
                        newNode.cost += newNode.solution[i].cost;
                        newNode.LB += newNode.solution[i].fmin;
                        std::cout << "  success. cost: " << newNode.cost << std::endl;

                        newNode.focalHeuristic = m_env.focalHeuristic(newNode.solution);

                        auto handle = open.push(newNode);
                        (*handle).handle = handle;
                        if (newNode.cost <= bestCost * m_w)
                        {
                            focal.push(handle);
                        }
                    }

                    ++id;
                }
            }

            return false;
        }

    private:
        struct HighLevelNode;

#ifdef USE_FIBONACCI_HEAP
        typedef typename boost::heap::fibonacci_heap<HighLevelNode> openSet_t;
        typedef typename openSet_t::handle_type handle_t;
// typedef typename boost::heap::fibonacci_heap<fibHeapHandle_t,
// boost::heap::compare<compareFocalHeuristic> > focalSet_t;
#else
        typedef typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                                 boost::heap::mutable_<true>>
            openSet_t;
        typedef typename openSet_t::handle_type handle_t;
// typedef typename boost::heap::d_ary_heap<fibHeapHandle_t,
// boost::heap::arity<2>, boost::heap::mutable_<true>,
// boost::heap::compare<compareFocalHeuristic> > focalSet_t;
#endif

        struct HighLevelNode
        {
            std::vector<PlanResult<State, Action, Cost>> solution;
            std::vector<Constraints> constraints;

            Cost cost;
            Cost LB; // sum of fmin of solution

            Cost focalHeuristic;

            int id;

            handle_t handle;

            bool operator<(const HighLevelNode &n) const
            {
                // if (cost != n.cost)
                return cost > n.cost;
                // return id > n.id;
            }

            friend std::ostream &operator<<(std::ostream &os, const HighLevelNode &c)
            {
                os << "id: " << c.id << " cost: " << c.cost << " LB: " << c.LB
                   << " focal: " << c.focalHeuristic << std::endl;
                for (size_t i = 0; i < c.solution.size(); ++i)
                {
                    os << "Agent: " << i << std::endl;
                    os << " States:" << std::endl;
                    for (size_t t = 0; t < c.solution[i].states.size(); ++t)
                    {
                        os << "  " << c.solution[i].states[t].first << std::endl;
                    }
                    os << " Constraints:" << std::endl;
                    os << c.constraints[i];
                    os << " cost: " << c.solution[i].cost << std::endl;
                }
                return os;
            }
        };

        struct compareFocalHeuristic
        {
            bool operator()(const handle_t &h1, const handle_t &h2) const
            {
                // Our heap is a maximum heap, so we invert the comperator function here
                if ((*h1).focalHeuristic != (*h2).focalHeuristic)
                {
                    return (*h1).focalHeuristic > (*h2).focalHeuristic;
                }
                return (*h1).cost > (*h2).cost;
            }
        };

#ifdef USE_FIBONACCI_HEAP
        typedef typename boost::heap::fibonacci_heap<
            handle_t, boost::heap::compare<compareFocalHeuristic>>
            focalSet_t;
#else
        typedef typename boost::heap::d_ary_heap<
            handle_t, boost::heap::arity<2>, boost::heap::mutable_<true>,
            boost::heap::compare<compareFocalHeuristic>>
            focalSet_t;
#endif

        struct LowLevelEnvironment
        {
            LowLevelEnvironment(
                Environment &env, size_t agentIdx, const Constraints &constraints,
                const std::vector<PlanResult<State, Action, Cost>> &solution)
                : m_env(env)
                  // , m_agentIdx(agentIdx)
                  // , m_constraints(constraints)
                  ,
                  m_solution(solution)
            {
                m_env.setLowLevelContext(agentIdx, &constraints);
            }

            Cost admissibleHeuristic(const State &s)
            {
                return m_env.admissibleHeuristic(s);
            }

            Cost focalStateHeuristic(const State &s, Cost gScore)
            {
                return m_env.focalStateHeuristic(s, gScore, m_solution);
            }

            State getGoal() { return m_env.getGoal(); }

            Cost focalTransitionHeuristic(const State &s1, const State &s2,
                                          Cost gScoreS1, Cost gScoreS2)
            {
                return m_env.focalTransitionHeuristic(s1, s2, gScoreS1, gScoreS2,
                                                      m_solution);
            }

            bool isSolution(
                const State &s, Cost g,
                std::unordered_map<State, std::tuple<State, Action, Cost, Cost>,
                                   std::hash<State>> &camefrom)
            {
                return m_env.isSolution(s, g, camefrom);
            }

            void getNeighbors(const State &s, Action act,
                              std::vector<Neighbor<State, Action, Cost>> &neighbors)
            {
                m_env.getNeighbors(s, act, neighbors);
            }

            void onExpandNode(const State &s, Cost fScore, Cost gScore)
            {
                // std::cout << "LL expand: " << s << " fScore: " << fScore << " gScore: "
                // << gScore << std::endl;
                // m_env.onExpandLowLevelNode(s, fScore, gScore, m_agentIdx,
                // m_constraints);
                m_env.onExpandLowLevelNode(s, fScore, gScore);
            }

            int calcIndex(const State &s) { return m_env.calcIndex(s); }

            void onDiscover(const State & /*s*/, Cost /*fScore*/, Cost /*gScore*/)
            {
                // std::cout << "LL discover: " << s << std::endl;
                // m_env.onDiscoverLowLevel(s, m_agentIdx, m_constraints);
            }

        private:
            Environment &m_env;
            // size_t m_agentIdx;
            // const Constraints& m_constraints;
            const std::vector<PlanResult<State, Action, Cost>> &m_solution;
        };

    private:
        Environment &m_env;
        float m_w;
        typedef FocalHybridAStar<State, Action, Cost, LowLevelEnvironment>
            LowLevelSearch_t;
    };

} // namespace libMultiRobotPlanning