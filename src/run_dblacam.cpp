// problem, dbNodes (start conf), expander, h_funs, robots, traj_wrapper
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
#include "utils.hpp"
#include "db_lacam.hpp"

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
  double timelimit;

  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&inputFile)->required(),
      "input file (yaml)")("output,o",
                           po::value<std::string>(&outputFile)->required(),
                           "output file (yaml)")(
      "cfg,c", po::value<std::string>(&cfgFile)->required(),
      "configuration file (yaml)")("time_limit,t",
                                   po::value<double>(&timelimit)->required(),
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
  cfg = cfg["db-lacam"]["default"];
  Options_tdbastar planner_options;
  planner_options.outFile = outputFile;
  // planner_options.search_timelimit = timelimit;
  planner_options.cost_delta_factor = 0;
  planner_options.delta = cfg["delta_0"].as<float>();
  planner_options.fix_seed = 1;
  planner_options.max_motions = cfg["num_primitives_0"].as<size_t>();
  std::cout << "*** options for pibt search ***" << std::endl;
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
  // read motions - homogeneous for now
  std::string motionsFile;
  motionsFile = "../new_format_motions/integrator1_2d_v0/my_motions.bin";
  std::vector<Motion> motions;
  planner_options.motionsFile = motionsFile;
  // load motions for a single robot - homogeneous case
  load_motion_primitives_new(planner_options.motionsFile, *(robots.at(0)), motions,
                             planner_options.max_motions, planner_options.cut_actions,
                             true, planner_options.check_cols);
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
      // start to inf for the reverse search
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
  for (size_t i = 0; i < std::min(motions.size(), planner_options.max_motions);
       ++i)
  {
    T_m->add(&motions.at(i));
  }
  Expander expander(robots.at(0).get(), T_m, planner_options.alpha * planner_options.delta, /*add static motion*/ false);
  // for LaCam
  std::vector<std::shared_ptr<Heu_fun>> h_funs;
  std::vector<std::shared_ptr<AStarNode>> dbN_start;
  for (size_t i = 0; i < robots.size(); i++)
  {
    std::shared_ptr<Heu_fun> h_fun = nullptr;
    h_fun =
        std::make_shared<Heu_roadmap_bwd<std::shared_ptr<AStarNode>, AStarNode>>(
            robots[i], heuristics[i], problem.goals[i]);
    h_funs.push_back(h_fun);
    dbN_start.push_back(std::make_shared<AStarNode>());
    auto node = dbN_start.back();
    node->gScore = 0;
    node->state_eig = problem.starts[i];
    node->hScore = h_funs[i]->h(problem.starts[i]);
    node->fScore = node->gScore + node->hScore;
    node->is_in_open = true;
    node->reaches_goal =
        (robots[i]->distance(problem.starts[i], problem.goals[i]) <=
         planner_options.goal_delta);
    DYNO_CHECK_GEQ(node->hScore, 0, "hScore should be positive");
    DYNO_CHECK_LEQ(node->hScore, 1e5, "hScore should be bounded");
  }
  const auto deadline = Deadline(timelimit);
  LaCAM lacam(problem, dbN_start, expander, h_funs, robots, /*verbose*/ 1, &deadline);
  MultiRobotTrajectory solution = lacam.solve();
  if (solution.is_empty())
  {
    std::cout << "LaCAM failed!" << std::endl;
    return false;
  }
  solution.to_yaml_format(outputFile.c_str());
  return 0;
}