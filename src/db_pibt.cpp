#include <boost/graph/graphviz.hpp>
#include <boost/heap/d_ary_heap.hpp>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
// OMPL headers
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include "ompl/base/Path.h"
#include "ompl/base/ScopedState.h"
#include <ompl/base/spaces/SE2StateSpace.h>
// dynobench
#include "dynobench/motions.hpp"
#include "dynobench/robot_models.hpp"
#include "dynobench/general_utils.hpp"
#include "dynobench/robot_models_base.hpp"
// boost stuff for the graph
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/undirected_graph.hpp>
#include <boost/property_map/property_map.hpp>
// custom
#include "db_pibt.hpp"
#include "map.hpp"

using dynobench::FMT;
using dynobench::Trajectory;

bool db_PIBT::set_new_config(std::vector<Eigen::VectorXd> Q_from,
                             std::vector<Eigen::VectorXd> &Q_to,
                             std::vector<std::shared_ptr<AStarNode>> dbN_from,
                             std::vector<std::shared_ptr<AStarNode>> &dbN_to,
                             std::vector<dynobench::Trajectory> &M_to,
                             const std::vector<int> &order,
                             std::map<size_t, RobotData> robot_data_rolled)
{
  bool success = true;
  for (auto i = 0; i < N; i++)
  {
    // ONLY with planned robots and env!
    if (!M_to[i].is_empty())
    {
      bool valid = false;
      time_planner.time_collision_with_env += timed_fun_void([&]
                                                             { valid = is_motion_valid_env_collision(i, M_to[i]); });
      if (valid)
      {
        time_planner.time_collision_with_planned += timed_fun_void([&]
                                                                   { valid = is_motion_valid(i, M_to[i], M_to); });
      }
      if (!valid)
      {
        success = false;
        break;
      }
    }
  }
  if (success)
  {
    for (auto p : order)
    {
      if (M_to[p].is_empty() && !funcPIBT(p, Q_from, Q_to, dbN_from, dbN_to, M_to, robot_data_rolled))
      {
        success = false;
        break;
      }
    }
  }

  return success;
}

bool db_PIBT::funcPIBT(size_t robot_id,
                       std::vector<Eigen::VectorXd> Q_from,
                       std::vector<Eigen::VectorXd> &Q_to,
                       std::vector<std::shared_ptr<AStarNode>> dbN_from,
                       std::vector<std::shared_ptr<AStarNode>> &dbN_to,
                       std::vector<dynobench::Trajectory> &M_to,
                       std::map<size_t, RobotData> robot_data_rolled,
                       bool pi)
{
  std::cout << "Calling dbPIBT for robot " << robot_id << ", PI: " << pi << std::endl;
  Eigen::VectorXd now_state = Q_from[robot_id];
  // std::cout << "robot " << robot_id << " start state: " << now_state.format(dynobench::FMT) << std::endl;
  RobotData robot_data = robot_data_rolled[robot_id];
  // std::cout << "rolled trajs size: " << robot_data.trajectories.size() << std::endl;
  bool success;
  for (size_t i = 0; i < robot_data.trajectories.size(); i++)
  {
    // std::cout << "robot " << robot_id << " motion: " << i << std::endl;
    dynobench::Trajectory traj = robot_data.trajectories[i];

    // check for collision with env and planned robots
    Eigen::VectorXd next_state = traj.states.back();
    bool valid = false;
    time_planner.time_collision_with_env += timed_fun_void([&]
                                                           { valid = is_motion_valid_env_collision(robot_id, traj); });
    if (valid)
    {
      time_planner.time_collision_with_planned += timed_fun_void([&]
                                                                 { valid = is_motion_valid(robot_id, traj, M_to); });
    }
    if (!valid)
      continue;
    // reserve the motion
    Q_to[robot_id] = next_state;
    dbN_to[robot_id]->state_eig = next_state;
    dbN_to[robot_id]->gScore = robot_data.last_state_g[i];
    dbN_to[robot_id]->hScore = robot_data.last_state_h[i];
    M_to[robot_id] = traj;
    success = true;
    // check for collision with unplanned robots (state-by-state)
    for (const auto &[key, rolled_trajs] : robot_data_rolled)
    {
      if (key == robot_id || !M_to[key].is_empty())
        continue;
      for (const auto &other_robot_trajs : rolled_trajs.trajectories) // potential motion for the neighbor
      {
        if (M_to[key].is_empty())
        {
          bool collides = false;
          time_planner.time_collision_with_unplanned += timed_fun_void([&]
                                                                       { collides = has_inter_robot_collision(traj, robot_id, other_robot_trajs, key); });
          if (collides)
          {
            int j = key;
            std::cout << "robot " << j << " hasn't planned, getting PI" << std::endl;
            success = funcPIBT(j, Q_from, Q_to, dbN_from, dbN_to, M_to, robot_data_rolled, /*pi*/ true);
            break;
          }
        }
      }
      if (!success)
        break;
    }
    if (!success)
      continue;
    // std::cout << "robot " << robot_id << " end state: " << next_state.format(dynobench::FMT) << std::endl;
    return true;
  }
  // if no motion was applicable, then the robot does not move
  std::cout << "robot " << robot_id << " did not find a motion primitive staying where it is" << std::endl;
  Q_to[robot_id] = Q_from[robot_id];
  dbN_to[robot_id]->state_eig = Q_from[robot_id];
  dbN_to[robot_id]->gScore = dbN_from[robot_id]->gScore;
  dbN_to[robot_id]->hScore = dbN_from[robot_id]->hScore;
  // keep it open, so the robot has not labeled as if it has planned
  M_to[robot_id].states.clear();
  M_to[robot_id].actions.clear();
  return false;
}

void db_PIBT::update_obj(size_t id, const Eigen::VectorXd state)
{
  std::vector<fcl::Transform3d> tmp_ts(1);
  robots[id]->transformation_collision_geometries(state, tmp_ts);
  fcl::Transform3d &tf = tmp_ts[0];
  robot_objs[id]->setTranslation(tf.translation());
  robot_objs[id]->setRotation(tf.rotation());
  robot_objs[id]->computeAABB();
};

// pair-wise collision checking for traj-to-traj, assumes equal length motions
bool db_PIBT::has_inter_robot_collision(dynobench::Trajectory robot_traj, size_t robot_id, dynobench::Trajectory other_robot_traj, size_t other_robot_id)
{
  const auto &ego_traj = robot_traj.states;
  const auto &other_traj = other_robot_traj.states;

  for (size_t s = 0; s < ego_traj.size(); ++s)
  {
    fcl::CollisionRequestd request;
    fcl::CollisionResultd result;
    time_planner.time_collisions += timed_fun_void([&]
                                                   {
    const Eigen::VectorXd &state1 = ego_traj[s];
    const Eigen::VectorXd &state2 = other_traj[s];
    update_obj(robot_id, state1);
    update_obj(other_robot_id, state2);

    fcl::collide(robot_objs[robot_id], robot_objs[other_robot_id], request, result); });

    if (result.isCollision())
    {
      std::cout << "collision between robots, skip the motion!" << std::endl;
      return true; // collision detected
    }
  }
  return false;
}
// collision with planned robot trajectories
bool db_PIBT::is_motion_valid(size_t robot_id,
                              const dynobench::Trajectory &traj,
                              const std::vector<dynobench::Trajectory> &M_to)
{
  for (size_t other_robot_id = 0; other_robot_id < M_to.size(); ++other_robot_id)
  {
    if (other_robot_id == robot_id || M_to[other_robot_id].is_empty())
      continue;
    // collision with other robot
    if (has_inter_robot_collision(traj, robot_id, M_to[other_robot_id], other_robot_id))
    {
      std::cout << "invalid motion, skip" << std::endl;
      return false;
    }
  }
  return true;
}
// collision with the env
bool db_PIBT::is_motion_valid_env_collision(size_t /*robot_id*/ i,
                                            dynobench::Trajectory &traj)
{
  Motion motion;
  time_planner.time_traj_to_motion += timed_fun_void([&]
                                                     { traj_to_motion(traj, *(robots[i]), motion, /*compute collision*/ true); });
  fcl::DefaultCollisionData<double> collision_data;
  time_planner.time_collisions += timed_fun_void([&]
                                                 {
    assert(motion.collision_manager);
    assert(robots[i]->env.get());
    motion.collision_manager->collide(robots[i]->env.get(), &collision_data,
                                      fcl::DefaultCollisionFunction<double>); });
  if (collision_data.result.isCollision())
  {
    std::cout << "collision with the env" << std::endl;
    return false;
  }
  return true;
}