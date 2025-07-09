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
#include "db-pibt.hpp"
#include "map.hpp"

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
  cfg = cfg["pibt"]["default"];
  Options_tdbastar options_pibt; // fine to use tdbastar options
  options_pibt.outFile = outputFile;
  options_pibt.search_timelimit = timeLimit;
  options_pibt.cost_delta_factor = 0;
  options_pibt.delta = cfg["delta_0"].as<float>();
  options_pibt.fix_seed = 1;
  options_pibt.max_motions = cfg["num_primitives_0"].as<size_t>();
  std::cout << "*** options for pibt search ***" << std::endl;
  options_pibt.print(std::cout);
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
  // read motions - homogeneous for now
  std::string motionsFile;
  motionsFile = "../new_format_motions/integrator1_2d_v0/my_motions.bin"; // important to change map size, grid size = motion lenght / 2
  // motionsFile = "../new_format_motions/integrator1_2d_v0/handcrafted_motions.bin";
  std::vector<Motion> motions;
  options_pibt.motionsFile = motionsFile;
  // load motions for a single robot - homogeneous case
  load_motion_primitives_new(options_pibt.motionsFile, *(robots.at(0)), motions,
                             options_pibt.max_motions, options_pibt.cut_actions,
                             true, options_pibt.check_cols);
  options_pibt.motions_ptr = &motions;
  std::vector<ompl::NearestNeighbors<std::shared_ptr<AStarNode>> *> heuristics(
      robots.size(), nullptr);
  // support reverse search, pibt needs a look-up table for cost-to-go value
  if (cfg["heuristic1"].as<std::string>() == "reverse-search")
  {
    auto start_rev = std::chrono::steady_clock::now();
    dynobench::Problem problem_original(inputFile);
    options_pibt.delta = cfg["heuristic1_delta"].as<float>();
    Out_info_tdb out_pibt;
    size_t robot_id = 0;
    for (const auto &robot : robots)
    {
      // start to inf for the reverse search
      problem.starts[robot_id]
          .head(robot->translation_invariance)
          .setConstant(std::sqrt(std::numeric_limits<double>::max()));
      Eigen::VectorXd tmp_state = problem.starts[robot_id];
      problem.starts[robot_id] = problem.goals[robot_id];
      problem.goals[robot_id] = tmp_state;
      LowLevelPlan<dynobench::Trajectory> tmp_solution;
      tdbastar(problem, options_pibt, tmp_solution.trajectory,
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
    options_pibt.delta = cfg["delta_0"].as<float>();
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
  T_m = nigh_factory_t<Motion *>(problem.robotTypes[0], robots.at(0), // homogeneous case
                                 /*reverse_search*/ false);
  // add all motions to Tm
  for (size_t i = 0; i < std::min(motions.size(), options_pibt.max_motions);
       ++i)
  {
    T_m->add(&motions.at(i));
  }
  // add the expander, homogeneous case
  Expander expander(robots.at(0).get(), T_m, options_pibt.alpha * options_pibt.delta);
  // allocate trajectory for the longest motion primitive
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
    traj_wrapper.allocate_size(max_traj_size, robots.at(0)->nx, robots.at(0)->nu);
  }
  Terminate_status status = Terminate_status::UNKNOWN;
  // manage nodes
  std::vector<open_t> opens(robots.size());
  std::unordered_map<int, std::vector<std::shared_ptr<AStarNode>>> robot_nodes;
  std::vector<dynobench::Trajectory> tmp_output_trajs;
  // for now each robot has its own Open set
  for (size_t i = 0; i < robots.size(); i++)
  {
    std::shared_ptr<Heu_fun> h_fun = nullptr;
    h_fun =
        std::make_shared<Heu_roadmap_bwd<std::shared_ptr<AStarNode>, AStarNode>>(
            robots.at(i), heuristics[i], problem.goals[i]);
    robot_hfuns.push_back(h_fun);
    robot_nodes[i].push_back(std::make_shared<AStarNode>());
    auto start_node = robot_nodes[i].at(0);
    start_node->gScore = 0;
    start_node->state_eig = problem.starts[i];
    start_node->hScore = robot_hfuns.at(i)->h(problem.starts[i]);
    start_node->fScore = start_node->gScore + start_node->hScore;
    start_node->is_in_open = true;
    start_node->reaches_goal =
        (robots.at(i)->distance(problem.starts[i], problem.goals[i]) <=
         options_pibt.delta);
    DYNO_CHECK_GEQ(start_node->hScore, 0, "hScore should be positive");
    DYNO_CHECK_LEQ(start_node->hScore, 1e5, "hScore should be bounded");
    start_node->handle = opens.at(i).push(start_node);
    dynobench::Trajectory traj;
    tmp_output_trajs.push_back(traj);
  }
  Time_benchmark time_bench;
  Stopwatch watch;
  auto stop_search = [&]
  {
    if (static_cast<size_t>(time_bench.expands) >= options_pibt.max_expands)
    {
      status = Terminate_status::MAX_EXPANDS;
      std::cout << "BREAK search:"
                << "MAX_EXPANDS" << std::endl;
      return true;
    }
    if (watch.elapsed_ms() > options_pibt.search_timelimit)
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
  PIBT pibt(expander, robot_hfuns, traj_wrapper, robots);
  std::shared_ptr<AStarNode> best_node;
  // store the output
  MultiRobotTrajectory output_trajs;
  output_trajs.trajectories.resize(robots.size());
  bool step_success = false;
  std::vector<std::shared_ptr<AStarNode>> from_nodes;
  std::vector<std::shared_ptr<AStarNode>> to_nodes;
  std::vector<size_t> priorities(robots.size());
  std::iota(priorities.begin(), priorities.end(), 0);
  int reached_goal;
  int itr = 0;
  while (!stop_search())
  {
    step_success = false;
    from_nodes.clear();
    to_nodes.clear();
    reached_goal = 0;
    for (size_t i = 0; i < robots.size(); i++)
    {
      best_node = opens.at(i).top(); // open set for each robot
      opens.at(i).pop();
      best_node->is_in_open = false;
      double distance_to_goal =
          robots.at(i)->distance(best_node->state_eig, problem.goals[i]);
      if (distance_to_goal < options_pibt.delta_factor_goal *
                                 options_pibt.delta)
      {
        reached_goal++;
      }
      from_nodes.push_back(best_node);
      to_nodes.push_back(std::make_shared<AStarNode>());
      tmp_output_trajs.at(i).states.clear();
      tmp_output_trajs.at(i).actions.clear();
    }

    if (reached_goal == robots.size())
    {
      auto end_time = std::chrono::steady_clock::now();
      duration duration_total = end_time - start_time;
      std::cout << "Time taken total: " << duration_total.count() << " seconds" << std::endl;
      output_trajs.to_yaml_format(outputFile.c_str());
      return 0;
    }
    std::sort(priorities.begin(), priorities.end(), [&](size_t i, size_t j)
              { return (robots.at(i)->distance(from_nodes.at(i)->state_eig, problem.goals[i])) > (robots.at(j)->distance(from_nodes.at(j)->state_eig, problem.goals[j])); });
    std::cout << "updated priorities: " << std::endl;
    for (auto p : priorities)
    {
      std::cout << p << std::endl;
    }
    itr++;
    std::cout << "Itr: " << itr << std::endl;
    pibt.step(from_nodes, to_nodes, tmp_output_trajs, robots, priorities, step_success);
    if (!step_success)
    {
      std::cout << "step function failed to get solution for all robots!" << std::endl;
    }
    else // save the motion inside output traj
    {
      for (size_t j = 0; j < robots.size(); j++)
      {
        output_trajs.trajectories.at(j).states.insert(output_trajs.trajectories.at(j).states.end(),
                                                      tmp_output_trajs.at(j).states.begin(), tmp_output_trajs.at(j).states.end());

        output_trajs.trajectories.at(j).actions.insert(output_trajs.trajectories.at(j).actions.end(),
                                                       tmp_output_trajs.at(j).actions.begin(), tmp_output_trajs.at(j).actions.end());
        // update the open set for each robot
        robot_nodes[j].push_back(std::make_shared<AStarNode>());
        auto __node = robot_nodes[j].back();
        __node->state_eig = to_nodes.at(j)->state_eig;
        __node->gScore = to_nodes.at(j)->gScore;
        __node->hScore = to_nodes.at(j)->hScore;
        __node->fScore = to_nodes.at(j)->gScore + to_nodes.at(j)->hScore;
        __node->is_in_open = true;
        __node->reaches_goal = robots.at(j)->distance(to_nodes.at(j)->state_eig, problem.goals[j]) <=
                               options_pibt.delta;
        __node->handle = opens.at(j).push(__node);
      }
    }
  }
  return 0;
}