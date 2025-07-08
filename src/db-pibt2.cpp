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
#include "nigh_custom_spaces.hpp"
#include "db-pibt.hpp"
#include "map.hpp"

using dynobench::FMT;
using dynobench::Trajectory;

void PIBT::step(std::vector<std::shared_ptr<AStarNode>> from_nodes, // current robot positions
                std::vector<std::shared_ptr<AStarNode>> &to_nodes,  // final state of the used motion
                std::vector<dynobench::Trajectory> &to_motions,
                std::vector<std::shared_ptr<dynobench::Model_robot>> robots,
                std::vector<size_t> priorities,
                bool &success)
{
  // 1. check occupied_now - state or up/down/right/left motions for the current state
  for (size_t i = 0; i < from_nodes.size(); i++)
  {
    Eigen::VectorXd state = from_nodes.at(i)->state_eig;
    get_4neighbors(state, i);
    auto [x_idx, y_idx] = world_to_grid(state(0), state(1)); // x,y of the position
    occupied_now.set_occupied(x_idx, y_idx, i);
  }
  // sanity checking
  // for (size_t j = 0; j < from_nodes.size(); ++j)
  // {
  //   if (neighbors.find(j) == neighbors.end())
  //   {
  //     std::cout << "Key " << j << " is missing from the map.\n";
  //   }
  //   if (neighbors.find(j) != neighbors.end() && neighbors[j].empty())
  //   {
  //     std::cout << "Key " << j << " has an empty vector.\n";
  //   }
  // }
  // 2. iteratively call pibt for each robot individually
  for (size_t p : priorities) // 2, 1, 3 can be
  {
    if (to_motions.at(p).is_empty())
      funcDBPIBT(from_nodes, to_nodes, to_motions, robots.at(p), p);
  }
  occupied_nxt.reset();
  occupied_now.reset();
  neighbors.clear();
  success = true;
}

bool PIBT::funcDBPIBT(std::vector<std::shared_ptr<AStarNode>> from_nodes, // node is state, gscore, hscore
                      std::vector<std::shared_ptr<AStarNode>> &to_nodes,  // final state of the used motion
                      std::vector<dynobench::Trajectory> &to_motions,
                      std::shared_ptr<dynobench::Model_robot> robot, // robot we consider
                      size_t /*robot id*/ i,
                      bool pi)
{
  std::cout << "calling pibt for robot " << i << ", PI: " << pi << std::endl;
  // 1. get applicable motions for robot_id
  auto tmp_node = std::make_shared<AStarNode>();
  lazy_trajs.clear();
  traj_wrappers.clear();
  Eigen::VectorXd now_state = from_nodes.at(i)->state_eig;
  expander.expand_lazy(now_state, lazy_trajs);
  auto ff = make_validity_checker(robot);
  bool collision = false;
  bool invalid_motion = false;
  double gScore, hScore;
  int num_valid_states = -1;
  int j = -1; // other robot that might need PI
  for (size_t j = 0; j < lazy_trajs.size(); j++)
  {
    auto &lazy_traj = lazy_trajs[j];
    traj_wrapper.set_size(lazy_traj.motion->traj.states.size());
    num_valid_states = -1;

    lazy_traj.compute(traj_wrapper, /*forward*/ true, /*check_state*/ &ff,
                      &num_valid_states);
    if (num_valid_states && num_valid_states < 1)
    {
      std::cout << "num_valid_states failed" << std::endl;
      continue;
    }
    if (num_valid_states < lazy_traj.motion->traj.states.size())
    {
      continue;
    }
    tmp_node->state_eig = traj_wrapper.get_state(traj_wrapper.get_size() - 1);
    hScore = h_functions.at(i)->h(tmp_node->state_eig); // for the last state of the motion
    double cost_motion = (traj_wrapper.get_size() - 1) * robot->ref_dt;
    gScore = from_nodes.at(i)->gScore + cost_motion;
    traj_wrapper.last_state_f = gScore + hScore;
    traj_wrappers.push_back(traj_wrapper);
  }
  // sort applicable motions based on f-value of the last state
  dynobench::TrajWrapper::SortByLastStateF(traj_wrappers);
  // 2. loop over sorted motions, and recursively call pibt if needed
  for (size_t k = 0; k < traj_wrappers.size(); k++)
  {
    j = -1;
    auto &traj_wrap = traj_wrappers[k];
    std::vector<Eigen::VectorXd> us = traj_wrap.get_actions();
    std::vector<Eigen::VectorXd> xs(us.size() + 1,
                                    Eigen::VectorXd::Zero(robot->nx));
    num_valid_states = -1;
    robot->rollout(now_state, us, xs, &ff,
                   &num_valid_states);
    if (num_valid_states && num_valid_states < xs.size())
    {
      std::cout << "rollout, state violations" << std::endl;
      continue;
    }
    // check for collision with env.
    Motion motion;
    dynobench::Trajectory traj;
    traj.start = now_state;
    traj.states = xs;
    traj.actions = us;
    traj.goal = traj.states.back();
    traj_to_motion(traj, *robot, motion, /*compute collision*/ true);
    assert(motion.collision_manager);
    assert(robot->env.get());
    fcl::DefaultCollisionData<double> collision_data;
    motion.collision_manager->collide(robot->env.get(), &collision_data,
                                      fcl::DefaultCollisionFunction<double>);
    if (collision_data.result.isCollision())
      continue;
    Eigen::VectorXd next_state = xs.back();
    // get grid where the robot is going to go next
    auto [x_idx, y_idx] = world_to_grid(next_state(0), next_state(1));
    // check for vertex collision
    if (occupied_nxt.get_cell(x_idx, y_idx) != -1) // already reserved by higher prioritized robot
      continue;
    // loop over neighbors of all robots, since those are states related to the current state of the robot and might affect/cause future collision
    // for (const auto &[key, vecs] : neighbors)
    // {
    //   if (key == i)
    //     continue;
    //   for (const auto &vec : vecs) // each neighbor of the state, including the state itself (5 in general)
    //   {
    //     if (vec.isApprox(next_state, 1e-4))
    //     {
    //       j = key;
    //       std::cout << "robot " << j << " might have some collison!" << std::endl;
    //       break;
    //     }
    //   }
    // }

    int j = occupied_now.get_cell(x_idx, y_idx);
    // check for swap collision if the other robot already has planned next move
    if (j != -1 && !to_motions.at(j).is_empty() && to_nodes.at(j)->state_eig.isApprox(now_state, 1e-4))
      continue;
    // reserve the motion
    to_nodes.at(i)->state_eig = xs.back();
    to_nodes.at(i)->gScore = gScore;
    to_nodes.at(i)->hScore = hScore;
    occupied_nxt.set_occupied(x_idx, y_idx, i);
    to_motions.at(i) = traj;
    // sanity check
    if (j != -1 && to_motions.at(j).is_empty())
    {
      std::cout << "robot " << j << " hasn't planned, getting PI" << std::endl;
    }
    // if the other robot hasn't planned yet, call pibt
    if (j != -1 && to_motions.at(j).is_empty() && !funcDBPIBT(from_nodes, to_nodes, to_motions, robots.at(j), j, /*pi*/ true))
    {
      std::cout << "recursive pibt failed" << std::endl;
      continue;
    }
    return true;
  }
  // if no motion was applicable, then the robot does not move - stay motion primitive is added
  return false;
}