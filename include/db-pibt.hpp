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
// dynobench
#include "dynobench/robot_models_base.hpp"
#include "dynobench/dyno_macros.hpp"
#include "dynobench/motions.hpp"
// custom
#include "nigh_custom_spaces.hpp"
#include "tdbastar.hpp"
#include "map.hpp"

bool pibt(std::vector<dynobench::TrajWrapper> traj_wrappers,
          dynobench::Model_robot &robot, dynobench::Trajectory &traj_out,
          Eigen::Ref<Eigen::VectorXd> x0,
          std::vector<dynobench::Trajectory> constrained_trajs);

struct PIBT
{
  Expander &expander;                                // homogeneous case
  std::vector<std::shared_ptr<Heu_fun>> h_functions; // can be hetero
  std::vector<std::shared_ptr<dynobench::Model_robot>> robots;
  std::vector<fcl::CollisionObjectd *> robot_objs;
  dynobench::TrajWrapper traj_wrapper; // can be hetero
  std::vector<dynobench::TrajWrapper> traj_wrappers;
  std::vector<LazyTraj> lazy_trajs;
  // std::vector<size_t> priorities;
  double delta = 0.5;
  // my map - grid style for now
  double width = 5.0;     // comes from problem.yaml
  double height = 5.0;    // comes from problem.yaml
  double grid_size = 0.5; // my motion primitives have this length
  OccupancyMap occupied_nxt;
  OccupancyMap occupied_now;
  std::map<size_t, std::vector<Eigen::VectorXd>> neighbors; // neighbor foru cells per robot, only for the current state

  PIBT(Expander &expander, std::vector<std::shared_ptr<Heu_fun>> h_functions, dynobench::TrajWrapper traj_wrapper, std::vector<std::shared_ptr<dynobench::Model_robot>> robots) : expander(expander), h_functions(h_functions), traj_wrapper(traj_wrapper), robots(robots), occupied_nxt(width, height, grid_size), occupied_now(width, height, grid_size)
  {
    // get robot objs for collision checking. Might need later, not now
    for (const auto &robot : robots)
    {
      auto robot_obj = new fcl::CollisionObject(robot->collision_geometries.at(0)); // homogeneous case
      robot_objs.push_back(robot_obj);
    }
  }
  PIBT() = default;
  bool funcDBPIBT(std::vector<std::shared_ptr<AStarNode>> from_nodes, // current node with state, gscore, hscpre
                  std::vector<std::shared_ptr<AStarNode>> &to_nodes,
                  std::vector<dynobench::Trajectory> &to_motions,
                  std::shared_ptr<dynobench::Model_robot>
                      robot, // current robot
                  size_t robot_id,
                  bool pi = false);

  void step(std::vector<std::shared_ptr<AStarNode>> from_nodes,
            std::vector<std::shared_ptr<AStarNode>> &to_nodes,
            std::vector<dynobench::Trajectory> &to_motions,
            std::vector<std::shared_ptr<dynobench::Model_robot>> robots,
            std::vector<size_t> priorities,
            bool &step_success);

  std::function<bool(Eigen::Ref<Eigen::VectorXd>)>
  make_validity_checker(std::shared_ptr<dynobench::Model_robot> robot)
  {
    return [robot](Eigen::Ref<Eigen::VectorXd> state)
    {
      return robot->is_state_valid(state);
    };
  }
  std::pair<int, int> world_to_grid(double x_m, double y_m) const
  {
    int x_idx = static_cast<int>(std::round(x_m / grid_size));
    int y_idx = static_cast<int>(std::round(y_m / grid_size));
    return {x_idx, y_idx};
  }
  // assumes motion primitives are of length 10, and hard-coded for single integrator dynamics
  void get_4neighbors(const Eigen::VectorXd &pos, size_t robot_id)
  {
    std::vector<Eigen::VectorXd> tmp_neighbors;

    tmp_neighbors.push_back(pos);

    Eigen::VectorXd up = pos;
    up(1) += 1;
    tmp_neighbors.push_back(up);

    Eigen::VectorXd down = pos;
    down(1) -= 1;
    tmp_neighbors.push_back(down);

    Eigen::VectorXd right = pos;
    right(0) += 1;
    tmp_neighbors.push_back(right);

    Eigen::VectorXd left = pos;
    left(0) -= 1;
    tmp_neighbors.push_back(left);

    neighbors[robot_id] = tmp_neighbors;
  }
};
