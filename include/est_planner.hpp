#pragma once

#include <optional>
#include <functional>
// DYNOPLAN
#include "dynoplan/tdbastar/planresult.hpp"
#include "dynoplan/tdbastar/tdbastar.hpp"
// DYNOBENCH
#include "dynobench/general_utils.hpp"
#include "dynobench/robot_models_base.hpp"
// other
#include "dbpibt_options.hpp"

void est(const Eigen::VectorXd &state,
         const dynobench::Problem &problem,
         Planner_options planner_options,
         std::shared_ptr<dynobench::Model_robot> robot,
         size_t &robot_id,
         double &h_value,
         ompl::NearestNeighbors<std::shared_ptr<dynoplan::AStarNode>> *heuristic_nn,
         ompl::NearestNeighbors<std::shared_ptr<dynoplan::AStarNode>> **heuristic_rev,
         //  ompl::NearestNeighbors<std::shared_ptr<dynoplan::AStarNode>> &heuristic_result,
         std::optional<std::reference_wrapper<
             ompl::NearestNeighbors<std::shared_ptr<dynoplan::AStarNode>>>>
             heuristic_result,
         bool reverse_search);