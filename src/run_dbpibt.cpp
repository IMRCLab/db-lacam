#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <yaml-cpp/yaml.h>
// BOOST
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/undirected_graph.hpp>
#include <boost/heap/d_ary_heap.hpp>
#include <boost/program_options.hpp>
#include <boost/property_map/property_map.hpp>
// OMPL headers
#include "ompl/base/Path.h"
#include "ompl/base/ScopedState.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
// custom->dynoplan
#include "dynoplan/nigh_custom_spaces.hpp"
#include "dynoplan/ompl/robots.h"
#include "dynoplan/tdbastar/tdbastar.hpp"
#include "dynoplan/tdbastar/options.hpp"
#include "dynoplan/tdbastar/planresult.hpp"
// DYNOBENCH
#include "dynobench/general_utils.hpp"
#include "dynobench/motions.hpp"
#include "dynobench/robot_models.hpp"
#include "dynobench/robot_models_base.hpp"
#include "dynobench/multirobot_trajectory.hpp"
// custom
#include "db_pibt.hpp"
#include "db_lacam.hpp"
#include "utils.hpp"
#include "dbpibt_utils.hpp"

namespace fs = std::filesystem;
#define DYNOBENCH_BASE "../dynoplan/dynobench/"
using duration = std::chrono::duration<double>;
using namespace dynoplan;

int main(int argc, char *argv[])
{
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  std::string inputFile;
  std::string outputFile;
  std::string cfgFile;
  double timeLimit;

  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&inputFile)->required(),
      "input file (yaml)")("output,o",
                           po::value<std::string>(&outputFile)->required(),
                           "output file (yaml)")(
      "cfg,c", po::value<std::string>(&cfgFile)->required(),
      "configuration file (yaml)")("time_limit,t",
                                   po::value<double>(&timeLimit)->required(),
                                   "time limit for search");

  try
  {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u)
    {
      std::cout << desc << "\n";
      return 0;
    }
  }
  catch (po::error &e)
  {
    std::cerr << e.what() << std::endl
              << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }
  auto start_time = std::chrono::steady_clock::now();
  YAML::Node cfg = YAML::LoadFile(cfgFile);
  // cfg = cfg["db-pibt"]["default"];
  Options_tdbastar planner_options; // fine to use tdbastar options
  planner_options.outFile = outputFile;
  planner_options.search_timelimit = timeLimit;
  planner_options.cost_delta_factor = 1;
  planner_options.fix_seed = 1;
  planner_options.delta = cfg["delta_0"].as<float>();
  planner_options.goal_delta = cfg["goal_delta"].as<float>();
  planner_options.max_motions = cfg["num_primitives_0"].as<size_t>();
  std::cout << "*** options for dbpibt search ***" << std::endl;
  planner_options.print(std::cout);
  std::cout << "***" << std::endl;
  dynobench::Problem problem(inputFile);
  std::string models_base_path = DYNOBENCH_BASE + std::string("models/");
  problem.models_base_path = models_base_path;
  Out_info_tdb out_pibt;
  YAML::Node env = YAML::LoadFile(inputFile);
  // create robots
  std::vector<std::shared_ptr<dynobench::Model_robot>> robots;
  for (size_t k = 0; k < problem.robotTypes.size(); k++)
  {
    std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
        (problem.models_base_path + problem.robotTypes.at(k) + ".yaml").c_str(), problem.p_lb,
        problem.p_ub);
    robots.push_back(robot);
    load_env(*(robots.at(k)), problem); // env enable, smarter needed
  }
  // read motions
  std::string motionsFile;
  if (problem.robotTypes[0] == "unicycle1_v0")
  {
    motionsFile = "../new_format_motions/unicycle1_v0/unit_length/unicycle1_v0.bin.im.bin.sp.bin";
  }
  else if (problem.robotTypes[0] == "integrator1_2d_v0")
  {
    motionsFile = "../new_format_motions/integrator1_2d_v0/unit_length2/integrator1_2d_v0.bin.im.bin.sp.bin";
  }
  else
  {
    throw std::runtime_error("Unknown motion filename for this robottype!");
  }
  std::vector<Motion> motions;
  planner_options.motionsFile = motionsFile;
  // read and filter duplicates
  load_motion_primitives_new(planner_options.motionsFile, *(robots[0]), motions,
                             planner_options.max_motions, planner_options.cut_actions,
                             /*shuffle*/ false, planner_options.check_cols);

  disable_motions(robots[0], problem.robotTypes[0], planner_options.delta, /*filter duplicates*/ true, /*alpha*/ 0.5,
                  planner_options.max_motions, motions);

  planner_options.motions_ptr = &motions;
  std::vector<ompl::NearestNeighbors<std::shared_ptr<AStarNode>> *> heuristics(
      robots.size(), nullptr);
  if (cfg["heuristic1"].as<std::string>() == "reverse-search")
  {
    auto start_rev = std::chrono::steady_clock::now();
    dynobench::Problem problem_original(inputFile);
    planner_options.delta = cfg["heuristic1_delta"].as<float>();
    Out_info_tdb out_pibt;
    size_t robot_id = 0;
    for (const auto &robot : robots)
    {
      problem.starts[robot_id]
          .head(robot->translation_invariance)
          .setConstant(std::sqrt(std::numeric_limits<double>::max()));
      Eigen::VectorXd tmp_state = problem.starts[robot_id];
      problem.starts[robot_id] = problem.goals[robot_id];
      problem.goals[robot_id] = tmp_state;
      LowLevelPlan<dynobench::Trajectory> tmp_solution;
      tdbastar(problem, planner_options, tmp_solution.trajectory,
               /*constraints*/ {}, out_pibt, robot_id, /*reverse_search*/ true,
               nullptr, &heuristics[robot_id]);
      std::cout << "computed heuristic with " << heuristics[robot_id]->size()
                << " entries." << std::endl;
      robot_id++;
    }
    auto end_rev = std::chrono::steady_clock::now();
    duration duration_rev = end_rev - start_rev;
    std::cout << "Time taken reverse search: " << duration_rev.count() << " seconds" << std::endl;

    // put back settings
    problem.starts = problem_original.starts;
    problem.goals = problem_original.goals;
    planner_options.delta = cfg["delta_0"].as<float>();
  }
  std::vector<std::shared_ptr<Heu_fun>> robot_hfuns;
  // check motions
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
  ompl::NearestNeighbors<Motion *> *T_m = nullptr;
  T_m = nigh_factory_t<Motion *>(problem.robotTypes[0], robots[0], // homogeneous case
                                 /*reverse_search*/ false);
  // add all motions to Tm
  for (size_t i = 0; i < std::min(motions.size(), planner_options.max_motions);
       ++i)
  {
    T_m->add(&motions[i]);
  }
  // add the expander, homogeneous case
  Expander expander(robots[0].get(), T_m, planner_options.alpha * planner_options.delta, /*add static motion*/ false);
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
    traj_wrapper.allocate_size(max_traj_size, robots[0]->nx, robots[0]->nu);
  }
  Terminate_status status = Terminate_status::UNKNOWN;
  // manage nodes
  std::vector<open_t> opens(robots.size());
  std::unordered_map<int, std::vector<std::shared_ptr<AStarNode>>> robot_nodes;
  std::vector<dynobench::Trajectory> M_to;
  // for now each robot has its own Open set
  for (size_t i = 0; i < robots.size(); i++)
  {
    std::shared_ptr<Heu_fun> h_fun = nullptr;
    h_fun =
        std::make_shared<Heu_roadmap_bwd2<std::shared_ptr<AStarNode>, AStarNode>>(
            robots[i], heuristics[i], problem.goals[i]);
    robot_hfuns.push_back(h_fun);
    robot_nodes[i].push_back(std::make_shared<AStarNode>());
    auto start_node = robot_nodes[i].at(0);
    start_node->gScore = 0;
    start_node->state_eig = problem.starts[i];
    start_node->hScore = robot_hfuns[i]->h(problem.starts[i]);
    start_node->fScore = start_node->gScore + start_node->hScore;
    start_node->is_in_open = true;
    start_node->reaches_goal =
        (robots[i]->distance(problem.starts[i], problem.goals[i]) <=
         planner_options.delta);
    DYNO_CHECK_GEQ(start_node->hScore, 0, "hScore should be positive");
    DYNO_CHECK_LEQ(start_node->hScore, 1e5, "hScore should be bounded");
    start_node->handle = opens[i].push(start_node);
    dynobench::Trajectory traj;
    M_to.push_back(traj);
  }
  Time_benchmark time_bench;
  Stopwatch watch;
  auto stop_search = [&]
  {
    if (static_cast<size_t>(time_bench.expands) >= planner_options.max_expands)
    {
      status = Terminate_status::MAX_EXPANDS;
      std::cout << "BREAK search:"
                << "MAX_EXPANDS" << std::endl;
      return true;
    }
    if (watch.elapsed_ms() > planner_options.search_timelimit)
    {
      status = Terminate_status::MAX_TIME;
      std::cout << "BREAK search:"
                << "MAX_TIME" << std::endl;
      return true;
    }
    if (std::any_of(opens.begin(), opens.end(), [](const auto &elem)
                    { return elem.empty(); }))
    {
      status = Terminate_status::EMPTY_QUEUE;
      std::cout << "BREAK search:"
                << "EMPTY_QUEUE" << std::endl;
      return true;
    }
    return false;
  };
  db_PIBT dbpibt(robots);
  std::shared_ptr<AStarNode> best_node;
  // store the output
  MultiRobotTrajectory solution;
  solution.trajectories.resize(robots.size());
  // for the search
  std::map<size_t, RobotData> rolled_robot_data;
  std::vector<Eigen::VectorXd> Q_from;
  Q_from.resize(robots.size());
  std::vector<int> order(robots.size());
  std::iota(order.begin(), order.end(), 0);
  int reached_goal;
  int loop_cnt = 0;
  bool success = false;
  while (!stop_search())
  {
    success = false;
    Q_from.clear();
    reached_goal = 0;
    for (size_t i = 0; i < robots.size(); i++)
    {
      best_node = opens[i].top(); // open set for each robot
      opens[i].pop();
      best_node->is_in_open = false;
      double distance_to_goal =
          robots[i]->distance(best_node->state_eig, problem.goals[i]);
      if (distance_to_goal <= planner_options.goal_delta)
      {
        reached_goal++;
      }
      Q_from.push_back(best_node->state_eig);
      M_to[i].states.clear();
      M_to[i].actions.clear();
      get_applicable_trajs(expander,
                           robot_hfuns, robots,
                           traj_wrapper,
                           best_node, rolled_robot_data[i], /*id*/ i);
    }

    if (reached_goal == robots.size())
    {
      auto end_time = std::chrono::steady_clock::now();
      duration duration_total = end_time - start_time;
      std::cout << "elapsed:" << std::setw(6) << duration_total.count() << "s"
                << "  loop_cnt:" << std::setw(8) << loop_cnt << std::endl;
      solution.to_yaml_format(outputFile.c_str());
      return 0;
    }
    std::sort(order.begin(), order.end(), [&](size_t i, size_t j)
              { return (robots[i]->distance(Q_from[i], problem.goals[i])) > (robots[j]->distance(Q_from[j], problem.goals[j])); });
    std::cout << "updated order: " << std::endl;
    loop_cnt++;
    // prepare _to
    std::vector<Eigen::VectorXd> Q_to;
    Q_to.resize(robots.size());
    std::vector<std::shared_ptr<AStarNode>> dbN_to;
    dbN_to.resize(robots.size());
    // call pibt
    success = dbpibt.set_new_config(Q_from, Q_to, dbN_to, M_to, order, rolled_robot_data);
    if (!success)
    {
      std::cout << "dbPIBT failed!" << std::endl;
      continue;
    }
    else
    {
      for (size_t j = 0; j < robots.size(); j++)
      {
        solution.trajectories[j].states.insert(solution.trajectories[j].states.end(),
                                               M_to[j].states.begin(), M_to[j].states.end());

        solution.trajectories[j].actions.insert(solution.trajectories[j].actions.end(),
                                                M_to[j].actions.begin(), M_to[j].actions.end());
        // update the open set for each robot
        robot_nodes[j].push_back(std::make_shared<AStarNode>());
        auto __node = robot_nodes[j].back();
        __node->state_eig = dbN_to[j]->state_eig;
        __node->gScore = dbN_to[j]->gScore;
        __node->hScore = dbN_to[j]->hScore;
        __node->fScore = dbN_to[j]->gScore + dbN_to[j]->hScore;
        __node->is_in_open = true;
        __node->reaches_goal = robots[j]->distance(dbN_to[j]->state_eig, problem.goals[j]) <=
                               planner_options.goal_delta;
        __node->handle = opens[j].push(__node);
      }
    }
  }
  return 0;
}