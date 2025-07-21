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
// custom
#include "db_pibt.hpp"
#include "utils.hpp"

using namespace dynoplan;

// low-level search node
struct LNode
{
  std::vector<int> who;
  std::vector<dynobench::Trajectory> where;
  std::vector<Eigen::VectorXd> where_state;
  std::vector<std::shared_ptr<AStarNode>> where_dbN;
  const uint depth;
  LNode();
  LNode(LNode *parent, int i, dynobench::Trajectory v, Eigen::VectorXd v_s, std::shared_ptr<AStarNode> v_dbN); // who, where (motion), where (state)
  ~LNode();
};

struct HNode;
struct CompareHNodePointers
{ // for determinism
  bool operator()(const HNode *lhs, const HNode *rhs) const;
};

// high-level search node
struct HNode
{
  const std::vector<Eigen::VectorXd> Q;
  const std::vector<std::shared_ptr<AStarNode>> dbN;
  HNode *parent;
  std::vector<dynobench::Trajectory> M_to;
  // cost
  // int g;
  // int h;
  // int f;
  int depth;

  std::vector<float> priorities;
  std::vector<int> order;
  std::queue<LNode *> search_tree;

  HNode(std::vector<Eigen::VectorXd> _C, std::vector<std::shared_ptr<AStarNode>> _dbNodes, std::vector<int> _order, HNode *_parent = nullptr);
  ~HNode();
};
using HNodes = std::vector<HNode *>;

struct LaCAM
{
  const dynobench::Problem problem;
  std::vector<std::shared_ptr<AStarNode>> dbNodes;
  const Deadline *timelimit;
  const int verbose;
  // search utils
  Expander &expander;
  std::vector<std::shared_ptr<dynoplan::Heu_fun>> h_funs;
  std::vector<std::shared_ptr<dynobench::Model_robot>> robots;
  std::map<size_t, RobotData> rolled_robot_data; // sorted, rolled robot trajs
  // tmp params
  std::vector<dynoplan::LazyTraj> tmp_lazy_trajs;
  dynobench::TrajWrapper tmp_traj_wrapper;
  std::vector<dynobench::TrajWrapper> tmp_traj_wrappers;
  MultiRobotTrajectory solution;
  // solver utils
  db_PIBT db_pibt;
  HNode *H_goal;
  std::deque<HNode *> OPEN;
  int loop_cnt;
  std::vector<int> order;

  LaCAM(const dynobench::Problem _problem,
        std::vector<std::shared_ptr<AStarNode>> _dbNodes,
        Expander &_expander,
        std::vector<std::shared_ptr<dynoplan::Heu_fun>> _h_funs,
        std::vector<std::shared_ptr<dynobench::Model_robot>> _robots,
        int _verbose = 0,
        const Deadline *_timelimit = nullptr);
  ~LaCAM();
  MultiRobotTrajectory solve();
  bool set_new_config(HNode *S, LNode *M, std::vector<Eigen::VectorXd> &Q_to,
                      std::vector<std::shared_ptr<AStarNode>> &dbN_to,
                      std::vector<dynobench::Trajectory> &M_to,
                      // std::map<size_t, std::vector<dynobench::Trajectory>> valid_trajs
                      std::map<size_t, RobotData> valid_trajs);

  void get_applicable_trajs(std::shared_ptr<AStarNode> db_node, RobotData &robot_data, size_t robot_id);

  std::vector<int> get_sorted_order(
      std::vector<std::shared_ptr<dynobench::Model_robot>> &robots,
      const std::vector<Eigen::VectorXd> states,
      const std::vector<Eigen::VectorXd> goal_states);

  std::function<bool(Eigen::Ref<Eigen::VectorXd>)>
  validity_checker(std::shared_ptr<dynobench::Model_robot> robot)
  {
    return [robot](Eigen::Ref<Eigen::VectorXd> state)
    {
      return robot->is_state_valid(state);
    };
  }
  // utilities
  template <typename... Body>
  void solver_info(const int level, Body &&...body)
  {
    if (verbose < level)
      return;
    std::cout << "elapsed:" << std::setw(6) << elapsed_ms(timelimit) << "ms"
              << "  loop_cnt:" << std::setw(8) << loop_cnt << "\t";
    info(level, verbose, (body)...);
  }
};