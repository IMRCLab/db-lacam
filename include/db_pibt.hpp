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
#include "dynobench/multirobot_trajectory.hpp"
// custom
#include "map.hpp"
#include "utils.hpp"

using namespace dynoplan;

struct db_PIBT
{
  std::vector<std::shared_ptr<dynobench::Model_robot>> robots;
  std::vector<fcl::CollisionObjectd *> robot_objs;
  std::vector<fcl::CollisionObjectd *> dyn_objs;
  std::vector<std::shared_ptr<dynobench::Model_robot>> dyn_robots;
  const int N; // number of robots
  Time_planner &time_planner;
  MultiRobotTrajectory dyn_obstacles;

  // Constructor with robots argument
  db_PIBT(std::vector<std::shared_ptr<dynobench::Model_robot>> _robots, Time_planner &_time_planner, MultiRobotTrajectory _dyn_obstacles)
      : robots(std::move(_robots)),
        time_planner(_time_planner),
        dyn_obstacles(_dyn_obstacles),
        N(robots.size())
  {
    for (const auto &robot : robots)
    {
      auto robot_obj = new fcl::CollisionObject(robot->collision_geometries.at(0)); // homogeneous case
      robot_objs.push_back(robot_obj);
    }
    if (!dyn_obstacles.is_empty())
    {
      for (size_t i = 0; i < dyn_obstacles.trajectories.size(); ++i)
      {
        dyn_robots.push_back(robots[0]);
      }
      for (const auto &robot : dyn_robots)
      {
        auto robot_obj = new fcl::CollisionObject(robot->collision_geometries.at(0)); // homogeneous case
        dyn_objs.push_back(robot_obj);
      }
    }
  }

  // Default constructor
  db_PIBT() = default;

  bool set_new_config(std::vector<Eigen::VectorXd> Q_from,
                      std::vector<Eigen::VectorXd> &Q_to,
                      std::vector<std::shared_ptr<AStarNode>> dbN_from,
                      std::vector<std::shared_ptr<AStarNode>> &dbN_to,
                      std::vector<dynobench::Trajectory> &M_to,
                      const std::vector<int> &order,
                      std::map<size_t, RobotData> robot_data_rolled);

  bool funcPIBT(size_t robot_id,
                std::vector<Eigen::VectorXd> Q_from,
                std::vector<Eigen::VectorXd> &Q_to,
                std::vector<std::shared_ptr<AStarNode>> dbN_from,
                std::vector<std::shared_ptr<AStarNode>> &dbN_to,
                std::vector<dynobench::Trajectory> &M_to,
                std::map<size_t, RobotData> robot_data_rolled,
                bool pi = false);

  bool has_inter_robot_collision(dynobench::Trajectory robot_traj, size_t robot_id,
                                 dynobench::Trajectory other_robot_traj, size_t other_robot_id,
                                 bool lazy_check,
                                 bool dynamic_obs);

  void update_obj(size_t id, const Eigen::VectorXd state, bool dynamic_obstacles);

  bool is_motion_valid(size_t robot_id,
                       const dynobench::Trajectory &traj,
                       const std::vector<dynobench::Trajectory> &M_to);

  bool is_motion_valid_env_collision(size_t robot_id,
                                     dynobench::Trajectory &traj);

  bool is_motion_valid_dyn_obstacles(size_t robot_id, int start_index,
                                     dynobench::Trajectory &traj);
};