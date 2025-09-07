#include <iostream>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <iterator>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <bits/stdc++.h>
// fcl
#include "fcl/broadphase/broadphase_collision_manager.h"
#include <fcl/fcl.h>
// BOOST
#include <boost/program_options.hpp>
#include <boost/program_options.hpp>
#include <boost/heap/d_ary_heap.hpp>
// DYNOPLAN
#include "dynoplan/nigh_custom_spaces.hpp"
#include "dynoplan/ompl/robots.h"
// others
#include "est_planner.hpp"

namespace fs = std::filesystem;
using namespace dynoplan;
#define DYNOBENCH_BASE "../dynoplan/dynobench/"

void est(const Eigen::VectorXd &state,
         const dynobench::Problem &problem,
         Planner_options planner_options,
         std::shared_ptr<dynobench::Model_robot> robot,
         size_t &robot_id,
         double &h_value,
         ompl::NearestNeighbors<std::shared_ptr<AStarNode>> *heuristic_nn,   // forward search uses it
         ompl::NearestNeighbors<std::shared_ptr<AStarNode>> **heuristic_rev, // reverse search fills it
                                                                             //  ompl::NearestNeighbors<std::shared_ptr<AStarNode>> &heuristic_result, // forward search fills it
         std::optional<std::reference_wrapper<
             ompl::NearestNeighbors<std::shared_ptr<dynoplan::AStarNode>>>>
             heuristic_result,
         bool reverse_search = false)
{
  int expansions = 0;
  bool success = false;
  std::vector<Eigen::VectorXd> expanded_nodes;
  if (!reverse_search)
    planner_options.max_expands = 1000; // limit the expansion for the forward search

  std::vector<Motion> &motions = *planner_options.motions_ptr;
  auto check_motions = [&]
  {
    for (size_t idx = 0; idx < motions.size(); ++idx)
    {
      if (motions[idx].idx != idx)
      {
        return false;
      }
    }
    return true;
  };
  assert(check_motions());
  // kd-tree related
  ompl::NearestNeighbors<Motion *> *T_m = nullptr;
  T_m = nigh_factory_t<Motion *>(problem.robotTypes[robot_id], robot, /*reverse_search*/ true); // reverse search?
  for (size_t i = 0; i < std::min(motions.size(), planner_options.max_motions); ++i)
  {
    T_m->add(&motions.at(i));
  }
  ompl::NearestNeighbors<std::shared_ptr<AStarNode>> *T_n = nullptr;
  T_n = nigh_factory2<std::shared_ptr<AStarNode>>(problem.robotTypes[robot_id], robot);
  if (reverse_search)
    *heuristic_rev = T_n;
  // motion primitives expander
  Expander expander(robot.get(), T_m,
                    planner_options.alpha * planner_options.delta, /*add static motion*/ true);
  srand(time(0));

  std::shared_ptr<Heu_fun> h_fun = nullptr;
  if (reverse_search)
    h_fun = std::make_shared<Heu_euclidean>(robot, problem.goals[robot_id]);
  else
    h_fun = std::make_shared<
        Heu_roadmap_bwd2<std::shared_ptr<AStarNode>, AStarNode>>(
        robot, heuristic_nn, problem.goals[robot_id], /*use_nn*/ false);

  std::vector<std::shared_ptr<AStarNode>> all_nodes;
  all_nodes.push_back(std::make_shared<AStarNode>());
  // start node
  auto start_node = all_nodes.at(0);
  start_node->gScore = 0;
  start_node->state_eig = state;
  start_node->hScore =
      h_fun->h(state);
  start_node->fScore = start_node->gScore + start_node->hScore;
  start_node->came_from = nullptr;
  start_node->is_in_open = true;
  start_node->reaches_goal =
      (robot->distance(state, problem.goals[robot_id]) <= planner_options.goal_delta);
  DYNO_CHECK_GEQ(start_node->hScore, 0, "hScore should be positive");
  DYNO_CHECK_LEQ(start_node->hScore, 1e5, "hScore should be bounded");
  T_n->add(start_node);
  // goal node
  auto goal_node = std::make_shared<AStarNode>();
  goal_node->state_eig = problem.goals[robot_id];
  // open set
  open_t open;
  start_node->handle = open.push(start_node);
  double best_distance_to_goal =
      robot->distance(start_node->state_eig, problem.goals[robot_id]);
  auto tmp_node = std::make_shared<AStarNode>();
  tmp_node->state_eig = Eigen::VectorXd::Zero(robot->nx);
  const size_t print_every = 100;
  double last_f_score = start_node->fScore;
  auto print_search_status = [&]
  {
    std::cout << "expands: " << expansions << " open: " << open.size()
              << " best distance: " << best_distance_to_goal
              << " fscore: " << last_f_score << std::endl;
  };
  Terminate_status status = Terminate_status::UNKNOWN;
  auto stop_search = [&]
  {
    if (expansions >=
        planner_options.max_expands)
    {
      status = Terminate_status::MAX_EXPANDS;
      std::cout << "BREAK search:"
                << "MAX_EXPANDS" << std::endl;
      return true;
    }
    if (open.empty())
    {
      status = Terminate_status::EMPTY_QUEUE;
      std::cout << "BREAK search:"
                << "EMPTY_QUEUE" << std::endl;
      return true;
    }

    return false;
  };
  std::shared_ptr<AStarNode> best_node;
  std::vector<std::shared_ptr<AStarNode>> neighbors_n;
  const size_t num_check_goal = 0;
  std::function<bool(Eigen::Ref<Eigen::VectorXd>)> ff =
      [&](Eigen::Ref<Eigen::VectorXd> state)
  {
    return robot->is_state_valid(state);
  };
  // allocate a trajectory for the largest motion primitive
  dynobench::TrajWrapper traj_wrapper;
  {
    std::vector<Motion *> motions;
    T_m->list(motions);
    size_t max_traj_size = (*std::max_element(motions.begin(), motions.end(),
                                              [](Motion *a, Motion *b)
                                              {
                                                return a->traj.states.size() <
                                                       b->traj.states.size();
                                              }))
                               ->traj.states.size();

    traj_wrapper.allocate_size(max_traj_size, robot->nx, robot->nu);
  }
  while (!stop_search())
  {
    expansions++;
    best_node = open.top();
    best_node->out_degree++;
    open.pop();
    last_f_score = best_node->fScore;
    best_node->is_in_open = false;

    if (expansions % print_every == 0)
    {
      print_search_status();
    }
    double distance_to_goal =
        robot->distance(best_node->state_eig, problem.goals[robot_id]);
    if (distance_to_goal < best_distance_to_goal)
    {
      best_distance_to_goal = distance_to_goal;
    }
    if (distance_to_goal < planner_options.goal_delta)
    {
      std::cout << "State reached the goal" << std::endl;
      status = Terminate_status::SOLVED;
      success = true;
      if (reverse_search)
      {
        break;
        // return;
      }
      // put path nodes to heuristic
      std::shared_ptr<AStarNode> n = best_node;
      while (n != nullptr)
      {
        heuristic_result->get().add(n);
        n = n->came_from;
      }
      h_value = best_node->gScore;
      break;
    }
    if (success)
      return;
    std::vector<LazyTraj> lazy_trajs;
    expander.expand_lazy(best_node->state_eig, lazy_trajs);
    std::vector<std::vector<Eigen::VectorXd>> all_actions;
    all_actions.resize(lazy_trajs.size());

    std::transform(lazy_trajs.begin(), lazy_trajs.end(), all_actions.begin(),
                   [](const LazyTraj &traj)
                   {
                     return traj.motion->traj.actions;
                   });
    int chosen_index = -1;
    // apply actions and expand the state
    Eigen::VectorXd x0 = best_node->state_eig;
    for (size_t j = 0; j < all_actions.size(); j++)
    {
      // i. rollout and keep the valid
      std::vector<Eigen::VectorXd> us = all_actions[j];
      std::vector<Eigen::VectorXd>
          xs(us.size() + 1,
             Eigen::VectorXd::Zero(robot->nx));
      int num_valid_states = -1;
      robot->rollout(x0, us, xs, &ff,
                     &num_valid_states);
      if (num_valid_states && num_valid_states < xs.size())
      {
        continue;
      }

      // ii. check for collision with the env.
      dynobench::Trajectory traj;
      traj.states.clear();
      traj.actions.clear();
      traj.start = x0;
      traj.states = xs;
      traj.actions = us;
      traj.goal = traj.states.back();

      Motion motion;
      traj_to_motion(traj, *(robot), motion, /*compute collision*/ true, /*merged_aabb*/ false);
      fcl::DefaultCollisionData<double> collision_data;
      assert(motion.collision_manager);
      assert(robot->env.get());
      motion.collision_manager->collide(robot->env.get(), &collision_data,
                                        fcl::DefaultCollisionFunction<double>);
      if (collision_data.result.isCollision())
        continue;
      // check if the mid state close to goal
      size_t mid_index = traj.states.size() / 2;
      Eigen::VectorXd tmp_mid_state = traj.states.at(mid_index); // middle of the motion
      if (robot->distance(tmp_mid_state, problem.goals[robot_id]) < planner_options.goal_delta)
      {
        status = Terminate_status::SOLVED;
        std::cout << "MID state reached the goal" << std::endl;
        // put path nodes to heuristic
        auto mid_node = std::make_shared<AStarNode>();
        mid_node->state_eig = tmp_mid_state;
        mid_node->gScore = best_node->gScore + ((mid_index - 1) * robot->ref_dt);
        mid_node->hScore = h_fun->h(tmp_mid_state);
        mid_node->fScore = mid_node->gScore + mid_node->hScore;
        mid_node->came_from = best_node;
        success = true;
        if (reverse_search)
        {
          T_n->add(mid_node);
          break;
        }
        std::shared_ptr<AStarNode> n = mid_node;
        while (n != nullptr)
        {
          heuristic_result->get().add(n);
          n = n->came_from;
        }
        h_value = best_node->gScore + ((mid_index - 1) * robot->ref_dt);
        break;
      }
      if (success)
        return;
      // ii. valid, add it to open set
      tmp_node->state_eig = xs.back();
      // expanded_nodes.push_back(tmp_node->state_eig);
      double hScore = h_fun->h(tmp_node->state_eig);
      double cost_motion = us.size() * robot->ref_dt;
      double gScore = best_node->gScore + cost_motion;
      T_n->nearestR(tmp_node, (1. - planner_options.alpha) * planner_options.delta, neighbors_n); // R can be customized
      if (!neighbors_n.size())
      {
        // STATE is NOVEL, we add the node
        all_nodes.push_back(std::make_shared<AStarNode>());
        auto __node = all_nodes.back();
        __node->state_eig = tmp_node->state_eig;
        __node->gScore = gScore;
        __node->hScore = hScore;
        double cost = gScore + hScore;
        __node->fScore = cost; // SHOULD BE WEIGHT
        __node->came_from = best_node;
        __node->is_in_open = true;
        __node->handle = open.push(__node);
        T_n->add(__node);
        // expanded_nodes.push_back(__node->state_eig);
      }
      else
      {

        for (auto &n : neighbors_n)
        {
          // STATE is not novel, we udpate
          if (float tentative_g =
                  gScore + planner_options.cost_delta_factor *
                               robot->lower_bound_time(tmp_node->state_eig,
                                                       n->state_eig);
              tentative_g < n->gScore)
          {
            n->gScore = tentative_g;
            double cost = tentative_g + n->hScore; // fScore
            n->fScore = cost;                      // SHOULD BE WEIGHT
            n->came_from = best_node;
            if (n->is_in_open)
            {
              open.increase(n->handle);
            }
            else
            {
              n->is_in_open = true;
              n->handle = open.push(n);
            }
          }
        }
      }
    }
  }
  // std::string filename = "est_expansion_" + std::to_string(robot_id) + ".yaml";
  // std::ofstream out(filename);
  // auto space6 = std::string(6, ' ');
  // out << "states:" << std::endl;
  // for (auto &state : expanded_nodes)
  // {
  //   out << space6 << "  - " << state.format(dynobench::FMT) << std::endl;
  // }
  if (!success)
    h_value = -1.0;
}
