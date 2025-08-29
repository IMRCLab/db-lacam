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
#include "est_guided.hpp"

namespace fs = std::filesystem;
using namespace dynoplan;
#define DYNOBENCH_BASE "../dynoplan/dynobench/"

void est_guided(dynobench::Problem &problem,
                Planner_options planner_options,
                size_t &robot_id,
                ompl::NearestNeighbors<std::shared_ptr<AStarNode>> **heuristic_result)
{
  // custom tmp params
  int expansions = 0;
  int cost_delta_factor = 1;
  double weight = 0;
  int alpha = 1;
  int betta = 2;
  int gamma = 3;
  // std::vector<Eigen::VectorXd> expanded_nodes;
  // create the robot
  std::shared_ptr<dynobench::Model_robot>
      robot = dynobench::robot_factory(
          (problem.models_base_path + problem.robotTypes[robot_id] + ".yaml")
              .c_str(),
          problem.p_lb, problem.p_ub);
  // load the env - obstacles
  load_env(*robot, problem);
  // load motions
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
  T_m = nigh_factory_t<Motion *>(problem.robotTypes[robot_id], robot, /*reverse_search*/ true); // NOT CLEAR YET
  for (size_t i = 0; i < std::min(motions.size(), planner_options.max_motions); ++i)
  {
    T_m->add(&motions.at(i));
  }
  ompl::NearestNeighbors<std::shared_ptr<AStarNode>> *T_n = nullptr;
  T_n = nigh_factory2<std::shared_ptr<AStarNode>>(problem.robotTypes[robot_id], robot);
  *heuristic_result = T_n;
  // motion primitives expander
  Expander expander(robot.get(), T_m,
                    planner_options.alpha * planner_options.delta, /*add static motion*/ true);
  srand(time(0));

  std::shared_ptr<Heu_fun> h_fun = nullptr;
  // h_fun = std::make_shared<Heu_blind>(); // NOT CLEAR
  h_fun = std::make_shared<Heu_euclidean>(robot, problem.goals[robot_id]); //  USE EUCLIDEAN
  std::vector<std::shared_ptr<AStarNode>> all_nodes;
  all_nodes.push_back(std::make_shared<AStarNode>());
  // start node
  auto start_node = all_nodes.at(0);
  start_node->gScore = 0;
  start_node->state_eig = problem.starts[robot_id];
  start_node->hScore =
      h_fun->h(problem.starts[robot_id]);
  start_node->fScore = start_node->gScore + start_node->hScore;
  start_node->is_in_open = true;
  start_node->reaches_goal =
      (robot->distance(problem.starts[robot_id], problem.goals[robot_id]) <=
       planner_options.delta);
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
      std::cout << "CLOSE to GOAL: " << distance_to_goal << std::endl;
      status = Terminate_status::SOLVED;
      break;
    }
    size_t num_expansion_best_node = 0;
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
        // std::cout << "rollout, state violations" << std::endl;
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
      // DEBUG
      // ii. valid, add it to open set
      tmp_node->state_eig = xs.back();
      // expanded_nodes.push_back(tmp_node->state_eig);
      tmp_node->out_degree = 0;
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
        double neighbors = 0;
        double out_degree = tmp_node->out_degree;
        double cost = gScore + hScore;
        double order = 1; // CHECK, UPDATE
        int neigh_density = 0;
        weight = 1 / (neigh_density + std::pow(out_degree, betta) + std::pow(cost, gamma));
        __node->fScore = cost; // SHOULD BE WEIGHT
        __node->is_in_open = true;
        __node->handle = open.push(__node);
        T_n->add(__node);
      }
      else
      {

        for (auto &n : neighbors_n)
        {
          // STATE is not novel, we udpate
          if (float tentative_g =
                  gScore + cost_delta_factor *
                               robot->lower_bound_time(tmp_node->state_eig,
                                                       n->state_eig);
              tentative_g < n->gScore)
          {
            n->gScore = tentative_g;
            double neighbors = neighbors_n.size();
            double out_degree = tmp_node->out_degree;
            double cost = tentative_g + n->hScore; // fScore
            double order = 1;                      // CHECK, UPDATE
            int neigh_density = neighbors_n.size();
            weight = 1 / (+std::pow(out_degree, betta) + std::pow(cost, gamma));
            n->fScore = cost; // SHOULD BE WEIGHT
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
  // DEBUG for viz., export expanded trajs
  // std::string filename = "est_expansion_cost.yaml";
  // std::ofstream out(filename);
  // auto space6 = std::string(6, ' ');
  // out << "states:" << std::endl;
  // for (auto &state : expanded_nodes)
  // {
  //   out << space6 << "  - " << state.format(dynobench::FMT) << std::endl;
  // }
}