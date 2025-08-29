#pragma once
// DYNOPLAN
#include "dynoplan/tdbastar/planresult.hpp"
#include "dynoplan/tdbastar/tdbastar.hpp"
// DYNOBENCH
#include "dynobench/general_utils.hpp"
#include "dynobench/robot_models_base.hpp"
// other
#include "dbpibt_options.hpp"

// struct compareNode
// {
//   bool operator()(const std::shared_ptr<dynoplan::AStarNode> a,
//                   const std::shared_ptr<dynoplan::AStarNode> b) const;
// };
// typedef typename boost::heap::d_ary_heap<
//     std::shared_ptr<dynoplan::AStarNode>, boost::heap::arity<2>,
//     boost::heap::compare<compareNode>, boost::heap::mutable_<true>>
//     open_set;

void est_guided(dynobench::Problem &problem,
                Planner_options planner_options,
                size_t &robot_id,
                ompl::NearestNeighbors<std::shared_ptr<dynoplan::AStarNode>> **heuristic_result);