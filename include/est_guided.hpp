#pragma once
// DYNOPLAN
#include "dynoplan/tdbastar/planresult.hpp"
#include "dynoplan/tdbastar/tdbastar.hpp"
// DYNOBENCH
#include "dynobench/general_utils.hpp"
#include "dynobench/robot_models_base.hpp"
// other
#include "dbpibt_options.hpp"

double est(const Eigen::VectorXd &state,
           const dynobench::Problem &problem,
           Planner_options planner_options,
           std::shared_ptr<dynobench::Model_robot> robot,
           size_t &robot_id,
           ompl::NearestNeighbors<std::shared_ptr<dynoplan::AStarNode>> *heuristic_nn,
           ompl::NearestNeighbors<std::shared_ptr<dynoplan::AStarNode>> &heuristic_result);

void est_unguided(dynobench::Problem &problem,
                  Planner_options planner_options,
                  size_t &robot_id,
                  ompl::NearestNeighbors<std::shared_ptr<dynoplan::AStarNode>> **heuristic_rev);