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
// h-based clustering
RobotData GetTopNPerClusterByH(const RobotData &input, double range, double min_h, double max_h, size_t N, bool shuffle = false)
{
  if (input.trajectories.empty())
    return {};

  double threshold = range * (max_h - min_h);
  // Combine into a sortable struct
  struct IndexedData
  {
    size_t index;
    double h;
    double g;
    dynobench::Trajectory traj;
  };

  std::vector<IndexedData> data;
  for (size_t i = 0; i < input.trajectories.size(); ++i)
  {
    data.push_back({i, input.last_state_h[i], input.last_state_g[i], input.trajectories[i]});
  }

  // Sort by h (then optionally g or something else)
  std::sort(data.begin(), data.end(), [](const IndexedData &a, const IndexedData &b)
            {
              if (a.h != b.h)
                return a.h < b.h;
              return a.g < b.g; // tie-breaker
            });

  // Clustering
  RobotData result;
  std::vector<IndexedData> current_cluster;
  double cluster_start_value = data[0].h;

  for (const auto &d : data)
  {
    if (std::fabs(d.h - cluster_start_value) > threshold)
    {
      // Cluster ended, pick top N
      size_t take = std::min(N, current_cluster.size());
      for (size_t i = 0; i < take; ++i)
      {
        result.trajectories.push_back(current_cluster[i].traj);
        result.last_state_h.push_back(current_cluster[i].h);
        result.last_state_g.push_back(current_cluster[i].g);
      }

      current_cluster.clear();
      cluster_start_value = d.h;
    }

    current_cluster.push_back(d);
  }

  // Handle final cluster
  if (!current_cluster.empty())
  {
    size_t take = std::min(N, current_cluster.size());
    for (size_t i = 0; i < take; ++i)
    {
      result.trajectories.push_back(current_cluster[i].traj);
      result.last_state_h.push_back(current_cluster[i].h);
      result.last_state_g.push_back(current_cluster[i].g);
    }
  }
  if (shuffle)
    result.shuffle();
  return result;
}
// distance based filtering
RobotData GetFilteredUniqueTopByH(const RobotData &input, double min_distance, std::vector<std::shared_ptr<dynobench::Model_robot>> robots, size_t robot_id)
{
  if (input.trajectories.empty())
    return {};

  struct IndexedData
  {
    size_t index;
    double h;
    double g;
    dynobench::Trajectory traj;
  };

  std::vector<IndexedData> data;
  for (size_t i = 0; i < input.trajectories.size(); ++i)
  {
    data.push_back({i, input.last_state_h[i], input.last_state_g[i], input.trajectories[i]});
  }

  // Sort by h (lowest first)
  std::sort(data.begin(), data.end(), [](const IndexedData &a, const IndexedData &b)
            {
              if (a.h != b.h)
                return a.h < b.h;
              return a.g < b.g; });

  RobotData result;

  for (const auto &d : data)
  {
    const auto &new_final_state = d.traj.states.back();

    bool too_close = false;
    for (const auto &existing_traj : result.trajectories)
    {
      const auto &existing_final_state = existing_traj.states.back(); // (new_final_state - existing_final_state).norm() < min_distance
      if (robots[robot_id]->distance(new_final_state, existing_final_state) < min_distance)
      {
        too_close = true;
        break;
      }
    }

    if (!too_close)
    {
      result.trajectories.push_back(d.traj);
      result.last_state_h.push_back(d.h);
      result.last_state_g.push_back(d.g);
    }
  }

  return result;
}

void get_applicable_trajs_precise(Expander &expander,
                                  std::vector<std::shared_ptr<dynoplan::Heu_fun>> h_funs,
                                  std::vector<std::shared_ptr<dynobench::Model_robot>> robots,
                                  dynobench::TrajWrapper tmp_traj_wrapper,
                                  std::shared_ptr<AStarNode> db_node,
                                  RobotData &robot_data, size_t robot_id)
{
  // clear
  std::vector<dynoplan::LazyTraj> tmp_lazy_trajs;
  std::vector<dynobench::TrajWrapper> tmp_traj_wrappers;
  robot_data.clear();
  // i. expand applicable motions
  expander.expand_lazy(db_node->state_eig, tmp_lazy_trajs);
  auto ff = validity_checker(robots[robot_id]);
  int num_valid_states = -1;
  double gScore = 0;
  double min_h = std::numeric_limits<double>::max();
  double max_h = std::numeric_limits<double>::lowest();

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
    double cost_motion = (tmp_traj_wrapper.get_size() - 1) * robots[robot_id]->ref_dt;
    gScore = db_node->gScore + cost_motion;
    tmp_traj_wrapper.last_state_g = gScore;
    tmp_traj_wrappers.push_back(tmp_traj_wrapper);
  }
  // ii. rollout trajs - env collision free
  RobotData tmp_data;
  double last_state_h = 0;
  Eigen::VectorXd x0 = db_node->state_eig;
  for (size_t k = 0; k < tmp_traj_wrappers.size(); k++)
  {
    auto &traj_wrap = tmp_traj_wrappers[k];
    std::vector<Eigen::VectorXd> us = traj_wrap.get_actions();
    std::vector<Eigen::VectorXd> xs(us.size() + 1,
                                    Eigen::VectorXd::Zero(robots[robot_id]->nx));
    int num_valid_states = -1;
    robots[robot_id]->rollout(x0, us, xs, &ff,
                              &num_valid_states);
    if (num_valid_states && num_valid_states < xs.size())
    {
      // std::cout << "rollout, state violations" << std::endl;
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
    tmp_data.trajectories.push_back(traj);
    tmp_data.last_state_g.push_back(traj_wrap.last_state_g);
    last_state_h = h_funs[robot_id]->h(traj.goal);
    tmp_data.last_state_h.push_back(last_state_h);
    if (last_state_h < min_h)
      min_h = last_state_h;
    if (last_state_h > max_h)
      max_h = last_state_h;
  }
  // h_value-based clustering, needs finetuning, that's why don't like it
  robot_data = GetTopNPerClusterByH(tmp_data, /*range*/ 0.1, min_h, max_h, 1, /*shuffle*/ false); // range 0.1-0.5 for sparseness, N=1 for alcove, atgoal, circle_uni. 0.05 for forest, wall
  // based on distance between last states of rolled-out-trajs, min_distance is the threshould for filtering, distance is computed with robot->distance function
  // robot_data = GetFilteredUniqueTopByH(tmp_data, /*min_distance*/ 0.5, robots, robot_id);
}
