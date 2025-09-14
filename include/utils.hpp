/*
 * utility functions
 */
#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <climits>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <list>
#include <map>
#include <numeric>
#include <queue>
#include <random>
#include <regex>
#include <set>
#include <stack>
#include <string>
#include <unordered_map>
#include <vector>
#include "Eigen/Core"
#include <vector>
#include <Eigen/Dense>
#include <limits>
// dynobench
// dynobench
#include "dynobench/robot_models_base.hpp"
#include "dynobench/motions.hpp"

using Time = std::chrono::steady_clock;

// time manager
struct Deadline
{
  const Time::time_point t_s;
  const double time_limit_ms;

  Deadline(double _time_limit_ms = 0);
  double elapsed_ms() const;
  double elapsed_ns() const;
};

double elapsed_ms(const Deadline *deadline);
double elapsed_ns(const Deadline *deadline);
bool is_expired(const Deadline *deadline);
bool is_expired(const Deadline &deadline);

float get_random_float(std::mt19937 &MT, float from = 0, float to = 1);
float get_random_float(std::mt19937 *MT, float from = 0, float to = 1);
int get_random_int(std::mt19937 &MT, int from = 0, int to = 1);
int get_random_int(std::mt19937 *MT, int from = 0, int to = 1);

template <typename Head, typename... Tail>
void info(const int level, const int verbose, Head &&head, Tail &&...tail);

void info(const int level, const int verbose);

template <typename Head, typename... Tail>
void info(const int level, const int verbose, Head &&head, Tail &&...tail)
{
  if (verbose < level)
    return;
  std::cout << head;
  info(level, verbose, std::forward<Tail>(tail)...);
}

template <typename... Body>
void info(const int level, const int verbose, const Deadline *deadline,
          Body &&...body)
{
  if (verbose < level)
    return;
  std::cout << "elapsed:" << std::setw(6) << elapsed_ms(deadline) << "ms  ";
  info(level, verbose, (body)...);
}

template <typename... Body>
void warn(Body &&...body)
{
  info(0, 0, "[warning] ", (body)...);
}

template <typename T>
std::ostream &operator<<(std::ostream &os, const std::vector<T> &arr)
{
  for (auto ele : arr)
    os << ele << ",";
  return os;
}

template <typename T>
std::ostream &operator<<(std::ostream &os, const std::list<int> &arr)
{
  for (auto ele : arr)
    os << ele << ",";
  return os;
}

template <typename T>
std::ostream &operator<<(std::ostream &os, const std::set<int> &arr)
{
  for (auto ele : arr)
    os << ele << ",";
  return os;
}
// bool is_close_config(std::vector<Eigen::VectorXd> Q1, std::vector<Eigen::VectorXd> Q2,
//  std::shared_ptr<dynobench::Model_robot> robot, double threshold);

struct Node
{
  Eigen::VectorXd state_eig;

  double gScore;
  double hScore;
  double fScore;
  bool is_in_open = false;
  bool valid = true;
  bool reaches_goal;
  Node();
  Node(Eigen::VectorXd _state_eig, double _gScore, double _hScore);
  ~Node();
  double get_cost() const { return gScore; }
  const Eigen::VectorXd &getStateEig() { return state_eig; }
};

struct RobotData
{
  std::vector<dynobench::Trajectory> trajectories;
  std::vector<double> last_state_g;
  std::vector<double> last_state_h;
  void clear()
  {
    trajectories.clear();
    last_state_g.clear();
    last_state_h.clear();
  }
  void shuffle()
  {
    size_t N = trajectories.size();
    if (N != last_state_g.size() || N != last_state_h.size())
    {
      throw std::runtime_error("Inconsistent vector sizes in RobotData::shuffle");
    }

    // Create index vector
    std::vector<size_t> indices(N);
    for (size_t i = 0; i < N; ++i)
      indices[i] = i;

    // Shuffle indices
    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(indices.begin(), indices.end(), gen);

    // Create shuffled copies
    std::vector<dynobench::Trajectory> shuffled_traj(N);
    std::vector<double> shuffled_g(N);
    std::vector<double> shuffled_h(N);

    for (size_t i = 0; i < N; ++i)
    {
      shuffled_traj[i] = trajectories[indices[i]];
      shuffled_g[i] = last_state_g[indices[i]];
      shuffled_h[i] = last_state_h[indices[i]];
    }

    // Assign back
    trajectories = std::move(shuffled_traj);
    last_state_g = std::move(shuffled_g);
    last_state_h = std::move(shuffled_h);
  }
  void sort_by_h()
  {
    size_t N = last_state_h.size();
    if (N != trajectories.size() || N != last_state_g.size())
    {
      throw std::runtime_error("Inconsistent vector sizes in RobotData::sort_by_h");
    }
    // Create index vector
    std::vector<size_t> indices(N);
    for (size_t i = 0; i < N; ++i)
      indices[i] = i;

    // Sort indices based on h values
    std::sort(indices.begin(), indices.end(), [&](size_t a, size_t b)
              { return last_state_h[a] < last_state_h[b]; });

    // Create sorted copies
    std::vector<dynobench::Trajectory> sorted_traj(N);
    std::vector<double> sorted_g(N);
    std::vector<double> sorted_h(N);

    for (size_t i = 0; i < N; ++i)
    {
      sorted_traj[i] = trajectories[indices[i]];
      sorted_g[i] = last_state_g[indices[i]];
      sorted_h[i] = last_state_h[indices[i]];
    }

    // Assign back
    trajectories = std::move(sorted_traj);
    last_state_g = std::move(sorted_g);
    last_state_h = std::move(sorted_h);
  }
};

struct ConfigHasher
{
  std::size_t operator()(const std::vector<Eigen::VectorXd> &config) const;
};

struct ConfigEqual
{
  bool operator()(const std::vector<Eigen::VectorXd> &a,
                  const std::vector<Eigen::VectorXd> &b) const;
};

struct Time_planner
{

  double extra_time = 0.;
  double reverse_search = 0.;        // reverse search
  double time_nearestMotion = 0.0;   // Tm-related operations
  double time_sort_order = 0.;       // sort robot orders
  double time_get_trajs = 0.;        // get applicable motions (rollout)
  double time_grow_search_tree = 0.; // constraint robots
  double time_lazy_expand = 0.0;     // motions expansion from the current state
  double time_lazy_sort = 0.;        // sort, leave only applicable ones
  double time_rollout = 0.;
  double time_collisions = 0.0;              // collision checking w.r.t env, other robots
  double time_clustering = 0.0;              // clustering of motions based on h-value
  double time_hfun = 0.0;                    // get h-value from the reverse search
  double time_collision_with_unplanned = 0.; // collision with motions of others (potentail)
  double time_collision_with_planned = 0;    // with motions of already-reserved-robots
  int expands = 0;                           // node expansion
  double time_traj_to_motion = 0.;           // convert traj to motion
  double time_get_actions = 0.;              // get actions from traj_wrap
  double time_collision_with_env = 0.;       // collision with the env

  double check_bounds = 0.0;
  double build_heuristic = 0.0;
  double time_transform_primitive = 0.0;
  double time_queue = 0.0;
  double time_nearestNode = 0.0;
  double time_nearestNode_add = 0.0;
  double time_nearestNode_search = 0.0;
  double prepare_time = 0.0;
  double total_time = 0.;
  int num_nn_motions = 0;
  int num_nn_states = 0;
  int num_col_motions = 0;
  int motions_tree_size = 0;
  int states_tree_size = 0;
  double time_search = 0;

  void print() const
  {
    std::cout << "*** Time stats for the planner ***" << std::endl;
    std::cout << " reverse_search: " << reverse_search << "\n";
    std::cout << " time_nearestMotion: " << time_nearestMotion << "\n";
    std::cout << " time_sort_order: " << time_sort_order << "\n";
    std::cout << " time_grow_search_tree: " << time_grow_search_tree << "\n";
    std::cout << " time_get_trajs: " << time_get_trajs << "\n";
    // std::cout << " time_lazy_expand: " << time_lazy_expand << "\n";
    // std::cout << " time_lazy_sort: " << time_lazy_sort << "\n";
    // std::cout << " time_get_actions: " << time_get_actions << "\n";
    std::cout << " time_traj_to_motion: " << time_traj_to_motion << "\n";
    std::cout << " time_rollout: " << time_rollout << "\n";
    std::cout << " time_collisions (mixed): " << time_collisions << "\n";
    // std::cout << " time_collision_with_env: " << time_collision_with_env << "\n";

    // std::cout << " time_clustering: " << time_clustering << "\n";
    std::cout << " time_hfun: " << time_hfun << "\n";
    std::cout << " time_collision_with_unplanned: " << time_collision_with_unplanned << "\n";
    std::cout << " time_collision_with_planned: " << time_collision_with_planned << "\n";
    // std::cout << " expands: " << expands << "\n";
    std::cout << "***" << std::endl;
  }
};

double action_seq_distance(const std::vector<Eigen::VectorXd> &A, const std::vector<Eigen::VectorXd> &B);
std::vector<std::vector<Eigen::VectorXd>> filter_diverse(const std::vector<std::vector<Eigen::VectorXd>> &candidates,
                                                         double eps);
