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
                             std::vector<std::shared_ptr<AStarNode>> &dbN_to,
                             std::vector<dynobench::Trajectory> &M_to,
                             const std::vector<int> &order,
                             std::map<size_t, RobotData> robot_data_rolled)
{
  bool success = true;
  for (auto i = 0; i < N; i++)
  {
    // set occupied now
    Eigen::VectorXd now_state = Q_from[i];
    auto [x_idx, y_idx] = world_to_grid(now_state(0), now_state(1)); // x,y of the position
    occupied_now.set_occupied(x_idx, y_idx, i);
    // set occupied next
    if (!M_to[i].is_empty())
    {
      // vertex collision
      Eigen::VectorXd next_state = Q_to[i];
      auto [x_idx, y_idx] = world_to_grid(next_state(0), next_state(1));
      if (occupied_nxt.get_cell(x_idx, y_idx) != -1)
      {
        success = false;
        break;
      }
      // check for swap collision
      int j = occupied_now.get_cell(x_idx, y_idx);
      if (j != -1 && !M_to.at(j).is_empty() && Q_to[j].isApprox(now_state, 1e-4))
      {
        success = false;
        break;
      }
      occupied_nxt.set_occupied(x_idx, y_idx, i);
    }
  }
  if (success)
  {
    for (auto p : order)
    {
      if (M_to[p].is_empty() && !funcPIBT(p, Q_from, Q_to, dbN_to, M_to, robot_data_rolled))
      {
        success = false;
        break;
      }
    }
  }
  // cleanup
  occupied_nxt.reset();
  occupied_now.reset();

  return success;
}

bool db_PIBT::funcPIBT(size_t robot_id,
                       std::vector<Eigen::VectorXd> Q_from,
                       std::vector<Eigen::VectorXd> &Q_to,
                       std::vector<std::shared_ptr<AStarNode>> &dbN_to,
                       std::vector<dynobench::Trajectory> &M_to,
                       std::map<size_t, RobotData> robot_data_rolled,
                       bool pi)
{
  std::cout << "calling pibt for robot " << robot_id << ", PI: " << pi << std::endl;
  RobotData robot_data = robot_data_rolled[robot_id];
  for (size_t r = 0; r < robot_data.trajectories.size(); r++)
  {
    dynobench::Trajectory traj = robot_data.trajectories[r];
    Eigen::VectorXd last_state = traj.states.back();
    // std::cout << "last state: " << last_state.format(dynobench::FMT) << std::endl;
    // std::cout << "gScore: " << robot_data.last_state_g[r] << ", hScore: " << robot_data.last_state_h[r] << std::endl;
  }
  for (size_t i = 0; i < robot_data.trajectories.size(); i++)
  {
    dynobench::Trajectory traj = robot_data.trajectories[i];
    Eigen::VectorXd next_state = traj.states.back();
    auto [x_idx, y_idx] = world_to_grid(next_state(0), next_state(1));
    // check for vertex collision
    if (occupied_nxt.get_cell(x_idx, y_idx) != -1)
      continue;
    // check for swap collision
    int j = occupied_now.get_cell(x_idx, y_idx);
    Eigen::VectorXd now_state = Q_from[robot_id];
    std::cout << "robot " << robot_id << " now state: " << std::endl;
    std::cout << now_state.format(dynobench::FMT) << std::endl;
    if (j != -1 && !M_to[j].is_empty() && Q_to[j].isApprox(now_state, 1e-4))
      continue;
    // reserve the motion
    Q_to[robot_id] = next_state;
    // double next_state_g = robot_data.last_state_g[j];
    // double next_state_h = robot_data.last_state_h[j];
    auto next_dbN = std::make_shared<AStarNode>();
    next_dbN->state_eig = next_state;
    next_dbN->gScore = robot_data.last_state_g[j];
    next_dbN->hScore = robot_data.last_state_h[j];
    dbN_to[robot_id] = next_dbN;
    occupied_nxt.set_occupied(x_idx, y_idx, robot_id);
    M_to[robot_id] = traj;
    if (j != -1 && M_to[j].is_empty())
    {
      std::cout << "robot " << j << " hasn't planned, getting PI" << std::endl;
    }
    if (j != -1 && M_to[j].is_empty() && !funcPIBT(j, Q_from, Q_to, dbN_to, M_to, robot_data_rolled, /*pi*/ true))
    {
      std::cout << "recursive pibt failed, continue to the next motion" << std::endl;
      continue;
    }
    std::cout << "robot " << robot_id << " end state: " << std::endl;
    std::cout << next_state.format(dynobench::FMT) << std::endl;
    return true;
  }
  // if no motion was applicable, then the robot does not move - stay motion primitive is added
  return false;
}