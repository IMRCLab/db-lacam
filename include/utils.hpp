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
bool is_close_config(std::vector<Eigen::VectorXd> Q1, std::vector<Eigen::VectorXd> Q2,
                     std::shared_ptr<dynobench::Model_robot> robot, double threshold);

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
