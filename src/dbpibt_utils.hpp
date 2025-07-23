#pragma once
#include "Eigen/Core"
// dynoplan
#include "dynoplan/nigh_custom_spaces.hpp"
#include "dynoplan/tdbastar/tdbastar.hpp"
#include "dynoplan/dbastar/heuristics.hpp"
// dynobench
#include "dynobench/robot_models_base.hpp"
#include "dynobench/dyno_macros.hpp"
#include "dynobench/motions.hpp"
#include "dynobench/multirobot_trajectory.hpp"

using namespace dynoplan;

std::function<bool(Eigen::Ref<Eigen::VectorXd>)>
validity_checker(std::shared_ptr<dynobench::Model_robot> robot)
{
  return [robot](Eigen::Ref<Eigen::VectorXd> state)
  {
    return robot->is_state_valid(state);
  };
}

void get_applicable_trajs(Expander &expander,
                          std::vector<std::shared_ptr<dynoplan::Heu_fun>> h_funs,
                          std::vector<std::shared_ptr<dynobench::Model_robot>> robots,
                          dynobench::TrajWrapper tmp_traj_wrapper,
                          std::shared_ptr<AStarNode> db_node,
                          RobotData &robot_data, size_t robot_id)
{
  std::vector<dynoplan::LazyTraj> tmp_lazy_trajs;
  std::vector<dynobench::TrajWrapper> tmp_traj_wrappers;
  robot_data.clear();
  // i. expand applicable motions
  expander.expand_lazy(db_node->state_eig, tmp_lazy_trajs);
  auto ff = validity_checker(robots[robot_id]);
  int num_valid_states = -1;
  double min_f = std::numeric_limits<double>::max();
  double max_f = std::numeric_limits<double>::lowest();
  double gScore = 0;
  double hScore;
  for (size_t j = 0; j < tmp_lazy_trajs.size(); j++)
  {
    auto &lazy_traj = tmp_lazy_trajs[j];
    tmp_traj_wrapper.set_size(lazy_traj.motion->traj.states.size());
    num_valid_states = -1;
    lazy_traj.compute(tmp_traj_wrapper, /*forward*/ true, /*check_state*/ &ff,
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
    Eigen::VectorXd tmp_state = tmp_traj_wrapper.get_state(tmp_traj_wrapper.get_size() - 1);
    // hScore = robots[robot_id]->distance(tmp_state, problem.goals[robot_id]); // for single integrator, if the env is small
    hScore = h_funs[robot_id]->h(tmp_state); // for the last state of the motion
    double cost_motion = (tmp_traj_wrapper.get_size() - 1) * robots[robot_id]->ref_dt;
    gScore = db_node->gScore + cost_motion;
    tmp_traj_wrapper.last_state_g = gScore;
    tmp_traj_wrapper.last_state_h = hScore;
    tmp_traj_wrapper.last_state_f = gScore + hScore;
    if (tmp_traj_wrapper.last_state_f < min_f)
      min_f = tmp_traj_wrapper.last_state_f;
    if (tmp_traj_wrapper.last_state_f > max_f)
      max_f = tmp_traj_wrapper.last_state_f;
    tmp_traj_wrappers.push_back(tmp_traj_wrapper);
  }
  // ii. sort/cluster based on f-value
  dynobench::TrajWrapper wr;
  std::vector<dynobench::TrajWrapper> sorted_traj_wrappers = wr.GetTopNPerClusterByLastStateF(tmp_traj_wrappers, /*range*/ 0.02, min_f, max_f, /*N*/ 8); // 0.02
  // ii. rollout trajs - env collision free
  Eigen::VectorXd x0 = db_node->state_eig;
  for (size_t k = 0; k < sorted_traj_wrappers.size(); k++)
  {
    auto &traj_wrap = sorted_traj_wrappers[k];
    std::vector<Eigen::VectorXd> us = traj_wrap.get_actions();
    std::vector<Eigen::VectorXd> xs(us.size() + 1,
                                    Eigen::VectorXd::Zero(robots[robot_id]->nx));
    int num_valid_states = -1;
    robots[robot_id]->rollout(x0, us, xs, &ff,
                              &num_valid_states);
    if (num_valid_states && num_valid_states < xs.size())
    {
      std::cout << "rollout, state violations" << std::endl;
      continue;
    }
    dynobench::Trajectory traj;
    traj.states.clear();
    traj.actions.clear();
    traj.start = x0;
    traj.states = xs;
    traj.actions = us;
    traj.goal = traj.states.back();
    // check for collision with the env
    Motion motion;
    traj_to_motion(traj, *(robots[robot_id]), motion, /*compute collision*/ true);
    assert(motion.collision_manager);
    assert(robots[robot_id]->env.get());
    fcl::DefaultCollisionData<double> collision_data;
    motion.collision_manager->collide(robots[robot_id]->env.get(), &collision_data,
                                      fcl::DefaultCollisionFunction<double>);
    if (collision_data.result.isCollision())
      continue;

    robot_data.trajectories.push_back(traj);
    // need for the Node update
    robot_data.last_state_g.push_back(traj_wrap.last_state_g);
    robot_data.last_state_h.push_back(traj_wrap.last_state_h);
  }
}