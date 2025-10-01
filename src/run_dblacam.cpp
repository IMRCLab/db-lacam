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
#include "dynoplan/tdbastar/compute_heu.hpp"
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
#include "dbpibt_options.hpp"
#include "est_planner.hpp"
#include "dbcbs_utils.hpp"

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
  std::string statsFile;
  std::string cfgFile;
  double timelimit;

  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&inputFile)->required(),
      "input file (yaml)")("output,o", po::value<std::string>(&outputFile)->required(),
                           "output file (yaml)")("stats,s", po::value<std::string>(&statsFile)->required(),
                                                 "stats file (yaml)")("cfg,c", po::value<std::string>(&cfgFile)->required(),
                                                                      "configuration file (yaml)")("time_limit,t", po::value<double>(&timelimit)->required(),
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
  create_dir_if_necessary(statsFile);
  std::ofstream stats(statsFile, std::ios::app);
  if (!stats)
  {
    std::cerr << "Failed to open stats.yaml file.\n";
    return 1;
  }
  auto start_time = std::chrono::steady_clock::now();
  YAML::Node cfg = YAML::LoadFile(cfgFile);
  // cfg = cfg["db-lacam"]["default"];
  // setup dblacam options
  Planner_options planner_options;
  planner_options.delta = cfg["delta_0"].as<float>();
  planner_options.max_motions = cfg["num_primitives_0"].as<size_t>();
  planner_options.alpha = cfg["alpha"].as<float>();
  planner_options.goal_delta = cfg["goal_delta"].as<float>();
  planner_options.cluster_range = cfg["cluster_range"].as<double>();
  planner_options.cluster_n = cfg["cluster_n"].as<size_t>();
  planner_options.cluster_shuffle = cfg["cluster_shuffle"].as<bool>();
  planner_options.cluster_distance = cfg["cluster_distance"].as<double>();
  planner_options.merged_aabb = cfg["merged_aabb"].as<bool>();
  planner_options.refine_solution = cfg["refine_solution"].as<bool>();
  planner_options.print();
  bool use_nn = false;
  Time_planner time_planner;
  // define the problem
  dynobench::Problem problem(inputFile);
  problem.models_base_path = DYNOBENCH_BASE + std::string("models/");
  YAML::Node env = YAML::LoadFile(inputFile);
  // create robots
  std::vector<std::shared_ptr<dynobench::Model_robot>> robots;
  for (size_t k = 0; k < problem.robotTypes.size(); k++)
  {
    std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
        (problem.models_base_path + problem.robotTypes[k] + ".yaml").c_str(), problem.p_lb,
        problem.p_ub);
    robots.push_back(robot);
    load_env(*(robots[k]), problem); // env enable, smarter needed
  }
  // read motions
  std::string motionsFile;
  if (problem.robotTypes[0] == "unicycle1_v0" || problem.robotTypes[0] == "unicycle1_sphere_v0")
  {
    motionsFile = "../new_format_motions/unicycle1_v0/spread/unicycle1_v0.bin.im.bin.sp.bin";
    // motionsFile = "../new_format_motions/unicycle1_v0/short/unicycle1_v0.bin.im.bin.sp.bin";
  }
  else if (problem.robotTypes[0] == "integrator1_2d_v0")
  {
    motionsFile = "../new_format_motions/integrator1_2d_v0/unit_length2/integrator1_2d_v0.bin.im.bin.sp.bin";
    // motionsFile = "../new_format_motions/integrator1_2d_v0/my_motions2.bin"; // handcrafted for debugging
  }
  else if (problem.robotTypes[0] == "integrator2_3d_v0")
  {
    // motionsFile = "../new_format_motions/integrator2_3d_v0/spread/integrator2_3d_v0.bin.im.bin.sp.bin";
    motionsFile = "../new_format_motions/integrator2_3d_v0/short/integrator2_3d_v0.bin.im.bin.sp.bin";
    use_nn = true;
  }
  else
  {
    throw std::runtime_error("Unknown motion filename for this robottype!");
  }
  std::vector<Motion> motions;
  planner_options.motionsFile = motionsFile;
  // read and filter duplicates
  load_motion_primitives_new(planner_options.motionsFile, *(robots[0]), motions,
                             planner_options.max_motions, /*cut_actions*/ false,
                             /*shuffle*/ false,
                             /*check_cols*/ true);

  disable_motions(robots[0], problem.robotTypes[0], planner_options.delta, /*filter duplicates*/ true, /*alpha*/ 0.5,
                  planner_options.max_motions, motions);

  planner_options.motions_ptr = &motions;
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
  std::vector<ompl::NearestNeighbors<std::shared_ptr<AStarNode>> *> heuristics_rev(
      robots.size(), nullptr);
  if (cfg["heuristic1"].as<std::string>() == "reverse-search")
  {
    double fake_h;
    dynobench::Problem problem_original(inputFile);
    Planner_options planner_options_rev = planner_options;
    planner_options_rev.delta = cfg["heuristic1_delta"].as<float>();
    planner_options_rev.max_motions = cfg["heuristic1_num_primitives_0"].as<size_t>();
    planner_options_rev.cost_delta_factor = 1;
    time_planner.reverse_search += timed_fun_void([&]
                                                  {
      auto start_rev = std::chrono::steady_clock::now();
      size_t robot_id = 0;
      for (const auto &robot : robots)
      {
        Eigen::VectorXd tmp_state = problem.starts[robot_id];
        problem.starts[robot_id] = problem.goals[robot_id];
        problem.goals[robot_id] = tmp_state;
        est(problem.starts[robot_id], problem, planner_options_rev, robots[robot_id], 
                              robot_id, fake_h, nullptr, &heuristics_rev[robot_id], std::nullopt, /*reverse search*/true);
        std::cout << "computed heuristic with " << heuristics_rev[robot_id]->size()
                  << " entries." << std::endl;
        robot_id++;
      }
      auto end_rev = std::chrono::steady_clock::now();
      duration duration_rev = end_rev - start_rev;
      std::cout << "Time taken reverse search: " << duration_rev.count() << " seconds" << std::endl; });
    // put back settings
    problem.starts = problem_original.starts;
    problem.goals = problem_original.goals;
  }
  ompl::NearestNeighbors<Motion *> *T_m = nullptr;
  T_m = nigh_factory_t<Motion *>(problem.robotTypes[0], robots[0], // homogeneous case
                                 /*reverse_search*/ false);
  // add all motions to Tm
  time_planner.time_nearestMotion += timed_fun_void([&]
                                                    {
  for (size_t i = 0; i < std::min(motions.size(), planner_options.max_motions);
       ++i)
  {
    T_m->add(&motions.at(i));
  } });
  Expander expander(robots[0].get(), T_m, planner_options.alpha * planner_options.delta, /*add static motion*/ true); // false for integrator1
  // for LaCam
  std::vector<std::shared_ptr<AStarNode>> dbN_start;
  std::vector<double> lower_bound_costs;
  std::vector<double> ratios;
  for (size_t i = 0; i < robots.size(); i++)
  {
    dbN_start.push_back(std::make_shared<AStarNode>());
    auto node = dbN_start.back();
    node->gScore = 0;
    node->state_eig = problem.starts[i];
    // node->hScore = h_funs[i]->h(problem.starts[i]);
    node->hScore = robots[i]->distance(problem.starts[i], problem.goals[i]);
    node->fScore = node->gScore + node->hScore;
    node->is_in_open = true;
    node->reaches_goal =
        (robots[i]->distance(problem.starts[i], problem.goals[i]) <=
         planner_options.goal_delta); // don't change distance weights!
    lower_bound_costs.push_back(node->hScore);
    DYNO_CHECK_GEQ(node->hScore, 0, "hScore should be positive");
    DYNO_CHECK_LEQ(node->hScore, 1e5, "hScore should be bounded");
  }
  const auto deadline = Deadline(timelimit);
  MultiRobotTrajectory dynamic_obstacles;
  LaCAM lacam(problem, dbN_start, expander, heuristics_rev, planner_options, robots, time_planner, dynamic_obstacles, /*verbose*/ 1, &deadline);
  MultiRobotTrajectory solution = lacam.solve();
  if (solution.is_empty())
  {
    std::cout << "LaCAM failed!" << std::endl;
    return false;
  }
  auto end_time = std::chrono::steady_clock::now();
  duration first_run_time = end_time - start_time;
  std::cout << "Time taken (total): " << first_run_time.count() * 1000 << " ms" << std::endl;
  double cost = solution.get_cost();
  time_planner.print();
  solution.to_yaml_format(outputFile.c_str()); // save just in case
  // save stats
  stats << "stats: " << "\n";
  stats << "  - t: " << first_run_time.count() << "\n";
  stats << "    cost: " << cost * 0.1 << "\n";
  stats.flush();

  if (first_run_time.count() * 1000 < timelimit && planner_options.refine_solution)
  {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::vector<int> all_ids(robots.size());
    auto start_time_refine = std::chrono::steady_clock::now();
    int k = 0;
    while (true)
    {
      std::cout << "New Refine Itr" << std::endl;
      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(first_run_time + (now - start_time_refine));
      // check if timelimit exceeded
      if (elapsed.count() >= timelimit)
        break;
      // create new problem for a run
      dynamic_obstacles.trajectories.clear();
      dynobench::Problem problem_tmp;
      problem_tmp.models_base_path = problem.models_base_path;
      problem_tmp.obstacles = problem.obstacles;
      problem_tmp.p_lb = problem.p_lb;
      problem_tmp.p_ub = problem.p_ub;
      std::vector<std::shared_ptr<AStarNode>> dbN_start_tmp;
      std::vector<ompl::NearestNeighbors<std::shared_ptr<AStarNode>> *> heuristics_rev_tmp;
      std::vector<std::shared_ptr<dynobench::Model_robot>> robots_tmp;
      Time_planner time_planner_tmp;
      // Option 1: randomly choose 2 distinct IDs
      std::iota(all_ids.begin(), all_ids.end(), 0);
      std::shuffle(all_ids.begin(), all_ids.end(), gen);
      std::vector<int> ids_tmp(all_ids.begin(), all_ids.begin() + 2);

      // Option 2: pick based on ratio = actual_cost / lower_bound_cost
      // for (size_t j = 0; j < robots.size(); j++)
      // {
      //   ratios.push_back(solution.trajectories[j].cost / lower_bound_costs[j]);
      // }
      // std::uniform_int_distribution<> dist(1, robots.size());
      // const auto num_refine_robots = std::max(1, std::min(dist(gen), int(robots.size() / 2)));
      // std::vector<int> ids_tmp = pick_subset_robots(ratios, /*N*/ num_refine_robots, gen);
      double old_cost = 0.;
      const auto deadline_tmp = Deadline(timelimit - elapsed.count());
      for (size_t i = 0; i < robots.size(); i++)
      {
        if (std::find(ids_tmp.begin(), ids_tmp.end(), i) != ids_tmp.end())
        {
          problem_tmp.starts.push_back(problem.starts[i]);
          problem_tmp.goals.push_back(problem.goals[i]);
          problem_tmp.robotTypes.push_back(problem.robotTypes[i]);
          dbN_start_tmp.push_back(dbN_start[i]);
          heuristics_rev_tmp.push_back(heuristics_rev[i]);
          robots_tmp.push_back(robots[i]);
          old_cost += solution.trajectories[i].cost;
          std::cout << "robot " << i << std::endl;
        }
        else
          dynamic_obstacles.trajectories.push_back(solution.trajectories[i]);
      }
      LaCAM lacam_tmp(problem_tmp, dbN_start_tmp, expander, heuristics_rev_tmp, planner_options, robots_tmp, time_planner_tmp, dynamic_obstacles, /*verbose*/ 1, &deadline_tmp);
      // save stats
      MultiRobotTrajectory tmp_solution = lacam_tmp.solve();
      if (!tmp_solution.is_empty())
      {
        double new_cost = tmp_solution.get_cost();
        std::cout << "old cost: " << old_cost << std::endl;
        std::cout << "new cost: " << new_cost << std::endl;
        if (new_cost < old_cost)
        {
          std::cout << "Refined returns a better solution" << std::endl;
          auto now_tmp = std::chrono::steady_clock::now();
          size_t i = 0;
          for (auto &traj : tmp_solution.trajectories)
          {
            solution.trajectories[ids_tmp[i]] = traj;
            i++;
          }
          duration ref_time = now_tmp - start_time;
          cost = solution.get_cost();
          stats << "  - t: " << ref_time.count() << "\n";
          stats << "    cost: " << cost * 0.1 << "\n";
          stats.flush();
          solution.to_yaml_format(outputFile.c_str()); // save just in case
        }
      }
    }
  }
  return 0;
}