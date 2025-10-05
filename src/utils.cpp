#include "utils.hpp"

void info(const int level, const int verbose) { std::cout << std::endl; }

Deadline::Deadline(double _time_limit_ms)
    : t_s(Time::now()), time_limit_ms(_time_limit_ms)
{
}

double Deadline::elapsed_ms() const
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(Time::now() -
                                                               t_s)
      .count();
}

double Deadline::elapsed_ns() const
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Time::now() - t_s)
      .count();
}

double elapsed_ms(const Deadline *deadline)
{
  if (deadline == nullptr)
    return 0;
  return deadline->elapsed_ms();
}

double elapsed_ns(const Deadline *deadline)
{
  if (deadline == nullptr)
    return 0;
  return deadline->elapsed_ns();
}

bool is_expired(const Deadline *deadline)
{
  if (deadline == nullptr)
    return false;
  return deadline->elapsed_ms() > deadline->time_limit_ms;
}

bool is_expired(const Deadline &deadline) { return is_expired(&deadline); }

float get_random_float(std::mt19937 &MT, float from, float to)
{
  return std::uniform_real_distribution<float>(from, to)(MT);
}

float get_random_float(std::mt19937 *MT, float from, float to)
{
  return get_random_float(*MT, from, to);
}

int get_random_int(std::mt19937 &MT, int from, int to)
{
  return std::uniform_int_distribution<int>(from, to)(MT);
}

int get_random_int(std::mt19937 *MT, int from, int to)
{
  return get_random_int(*MT, from, to);
}
// allow threshold for reaching the goal for now. Homogeneous robots
bool is_close_config(std::vector<Eigen::VectorXd> Q1, std::vector<Eigen::VectorXd> Q2, std::shared_ptr<dynobench::Model_robot> robot, double threshold)
{
  assert(Q1.size() == Q2.size());
  int cnt = 0;
  for (size_t i = 0; i < Q1.size(); i++)
  {
    if (robot->distance(Q1.at(i), Q2.at(i)) <= threshold)
    {
      std::cout << "close to final: " << robot->distance(Q1.at(i), Q2.at(i)) << std::endl;
      cnt++;
    }
  }
  return (cnt == Q1.size()) ? true : false;
}

Node::Node(Eigen::VectorXd _state_eig, double _gScore, double _hScore) : state_eig(_state_eig),
                                                                         gScore(_gScore),
                                                                         hScore(_hScore),
                                                                         fScore(gScore + hScore) {}

std::size_t ConfigHasher::operator()(const std::vector<Eigen::VectorXd> &config) const
{
  std::size_t hash = config.size();
  for (const auto &vec : config)
  {
    for (int i = 0; i < vec.size(); ++i)
    {
      std::size_t h = std::hash<double>{}(vec[i]);
      hash ^= h + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    }
  }
  return hash;
}

bool ConfigEqual::operator()(const std::vector<Eigen::VectorXd> &a,
                             const std::vector<Eigen::VectorXd> &b) const
{
  if (a.size() != b.size())
    return false;
  for (size_t i = 0; i < a.size(); ++i)
  {
    if (a[i].size() != b[i].size())
      return false;
    if (!a[i].isApprox(b[i]))
      return false;
  }
  return true;
}

double action_seq_distance(const std::vector<Eigen::VectorXd> &A, const std::vector<Eigen::VectorXd> &B)
{
  size_t T = A.size();
  double sum = 0.0;
  for (size_t t = 0; t < T; t++)
  {
    sum += (A[t] - B[t]).norm();
  }
  return sum / static_cast<double>(T);
}

std::vector<std::vector<Eigen::VectorXd>> filter_diverse(const std::vector<std::vector<Eigen::VectorXd>> &candidates,
                                                         double eps)
{
  std::vector<std::vector<Eigen::VectorXd>> diverse;
  for (const auto &cand : candidates)
  {
    bool is_unique = true;
    for (const auto &kept : diverse)
    {
      if (action_seq_distance(cand, kept) < eps)
      {
        is_unique = false;
        break;
      }
    }
    if (is_unique)
    {
      diverse.push_back(cand);
    }
  }
  return diverse;
}
