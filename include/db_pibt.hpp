#pragma once
#include "Eigen/Core"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>
#include <yaml-cpp/yaml.h>
// OMPL
#include "ompl/base/ScopedState.h"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/datastructures/NearestNeighbors.h>
// fcl
#include <fcl/fcl.h>
// custom -> dynoplan
#include "dynoplan/nigh_custom_spaces.hpp"
#include "dynoplan/tdbastar/tdbastar.hpp"
#include "dynoplan/dbastar/heuristics.hpp"
// dynobench
#include "dynobench/robot_models_base.hpp"
#include "dynobench/dyno_macros.hpp"
#include "dynobench/motions.hpp"
// custom
#include "map.hpp"
#include "utils.hpp"

using namespace dynoplan;

struct db_PIBT
{
  std::vector<std::shared_ptr<dynobench::Model_robot>> robots;
  const int N;            // number of robots
  double width = 11;      // comes from problem.yaml
  double height = 11;     // comes from problem.yaml
  double grid_size = 1.0; // my motion primitives length / 2, since a=0.5
  OccupancyMap occupied_nxt;
  OccupancyMap occupied_now;

  // Constructor with robots argument
  db_PIBT(std::vector<std::shared_ptr<dynobench::Model_robot>> _robots)
      : robots(std::move(_robots)),
        occupied_nxt(width, height, grid_size),
        occupied_now(width, height, grid_size),
        N(robots.size())
  {
  }

  // Default constructor
  db_PIBT() = default;

  bool set_new_config(std::vector<Eigen::VectorXd> Q_from,
                      std::vector<Eigen::VectorXd> &Q_to,
                      std::vector<std::shared_ptr<AStarNode>> &dbN_to,
                      std::vector<dynobench::Trajectory> &M_to,
                      const std::vector<int> &order,
                      std::map<size_t, RobotData> robot_data_rolled);

  bool funcPIBT(size_t robot_id,
                std::vector<Eigen::VectorXd> Q_from,
                std::vector<Eigen::VectorXd> &Q_to,
                std::vector<std::shared_ptr<AStarNode>> &dbN_to,
                std::vector<dynobench::Trajectory> &M_to,
                std::map<size_t, RobotData> robot_data_rolled,
                bool pi = false);

  std::pair<int, int> world_to_grid(double x_m, double y_m) const
  {
    int x_idx = static_cast<int>(std::round(x_m / grid_size));
    int y_idx = static_cast<int>(std::round(y_m / grid_size));
    return {x_idx, y_idx};
  }
};