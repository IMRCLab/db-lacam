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
#define BASE "../../" // mrmp benchmark dir
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
  std::optional<unsigned int> seed;


  desc.add_options()("help", "produce help message")
      ("input,i", po::value<std::string>(&inputFile)->required(),"input file (yaml)")
      ("output,o", po::value<std::string>(&outputFile)->required(),"output file (yaml)")
      ("stats", po::value<std::string>(&statsFile)->required(),"stats file (yaml)")
      ("cfg,c", po::value<std::string>(&cfgFile)->required(),"configuration file (yaml)")
      ("time_limit,t", po::value<double>(&timelimit)->required(),"time limit for search")
      ("seed", po::value<unsigned int>(), "random seed (default: deterministic)");
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  try
  {
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
    // set seeds
  seed = vm.count("seed")
         ? std::optional<unsigned int>(vm["seed"].as<unsigned int>())
         : std::optional<unsigned int>(42); // determenistic behavior
  if (seed.has_value()) {
    std::cout << "db-LaCAM uses seed " << seed.value() << std::endl;
  } else {
      std::cout << "db-LaCAM uses seed (default) " << std::endl;
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
  cfg = cfg["db-lacam"]["default"];
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
  planner_options.seed = seed;
  planner_options.print();
  bool use_nn = false;
  Time_planner time_planner;
  // define the problem
  dynobench::Problem problem(inputFile);
  problem.models_base_path = BASE + std::string("robot_types/");
  YAML::Node env = YAML::LoadFile(inputFile);
  // create robots
  std::vector<std::shared_ptr<dynobench::Model_robot>> robots;
  // read motions
  std::vector<std::string> all_motionsFile;
  std::string motionsFile;
  for (auto &robotType : problem.robotTypes)
  {
    std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
        (problem.models_base_path + robotType + ".yaml").c_str(), problem.p_lb, problem.p_ub);
    robots.push_back(robot);
    if (robotType == "unicycle_first_order" || robotType == "unicycle_sphere_first_order")
    {
      motionsFile = "db-lacam/new_format_motions/unicycle1_v0/spread/unicycle1_v0.bin.im.bin.sp.bin";
    }
    else if (robotType == "single_integrator")
    {
      motionsFile = "db-lacam/new_format_motions/integrator1_2d_v0/unit_length2/integrator1_2d_v0.bin.im.bin.sp.bin";
    }
    else if (robotType == "double_integrator_2d")
    {
      motionsFile = "db-lacam/new_format_motions/integrator2_2d_v0/integrator2_2d_v0.bin.im.bin.sp.bin.yaml";
    }
    else if (robotType == "double_integrator_3d")
    {
      motionsFile = "db-lacam/new_format_motions/integrator2_3d_v0/short/integrator2_3d_v0.bin.im.bin.sp.bin";
    }
    else
    {
      throw std::runtime_error("Unknown motion filename for this robottype!");
    }
    all_motionsFile.push_back(motionsFile);
  }
  std::map<std::string, std::vector<Motion>> robot_motions;
  planner_options.motions_ptrs.resize(robots.size());
  for (size_t i = 0; i < robots.size(); ++i)
  {
    auto &robot = robots[i];
    auto &type = problem.robotTypes[i];
    load_env(*robot, problem); 
    // Load motions only once per robot type
    if (robot_motions.find(type) == robot_motions.end())
    {
      planner_options.motionsFile = all_motionsFile[i];
      load_motion_primitives_new(planner_options.motionsFile, *robot, robot_motions[type],
                                 planner_options.max_motions, /*cut_actions*/ false,
                                 /*shuffle*/ true,
                                 /*check_cols*/ true, seed);
      disable_motions(robot, type, planner_options.delta, /*filter duplicates*/ true,
                      /*alpha*/0.5, planner_options.max_motions, robot_motions[type]);
    }
    planner_options.motions_ptrs[i] = &robot_motions[type];
  }
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
        planner_options_rev.motions_ptr = &robot_motions[problem.robotTypes[robot_id]];
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
  // for LaCam
  std::vector<std::shared_ptr<AStarNode>> dbN_start;
  std::vector<double> lower_bound_costs;
  std::vector<double> ratios;
  std::vector<Expander> expanders;
  for (size_t i = 0; i < robots.size(); i++)
  {
    // prepare motion expander for each robot
    ompl::NearestNeighbors<Motion *> *T_m = nullptr;
    T_m = nigh_factory_t<Motion *>(problem.robotTypes[i], robots[i],
                                   /*reverse_search*/ false);
    auto &motions = robot_motions[problem.robotTypes[i]];
    time_planner.time_nearestMotion += timed_fun_void([&]
                                                      {
        for (size_t j = 0; j < std::min(motions.size(), planner_options.max_motions);
            ++j)
        {
          T_m->add(&motions.at(j));
        } });
    expanders.emplace_back(robots[i].get(), T_m, planner_options.alpha * planner_options.delta, true);
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
         planner_options.goal_delta);
    lower_bound_costs.push_back(node->hScore);
    DYNO_CHECK_GEQ(node->hScore, 0, "hScore should be positive");
    DYNO_CHECK_LEQ(node->hScore, 1e5, "hScore should be bounded");
  }
  const auto deadline = Deadline(timelimit);
  MultiRobotTrajectory dynamic_obstacles;
  LaCAM lacam(problem, dbN_start, expanders, heuristics_rev, planner_options, robots, time_planner, dynamic_obstacles, /*verbose*/ 1, &deadline);
  MultiRobotTrajectory solution = lacam.solve();
  // extract the output/stats
  std::filesystem::path p(inputFile);
  std::string instanceName = p.filename().string(); 
  if (solution.is_empty())
  {
    std::cout << "LaCAM failed!" << std::endl;
    stats << "stats: " << "\n";
    stats << "  - instance: " << instanceName << "\n";
    stats << "    success: " << false << "\n";
    stats << "    elapsed_time_sec: " <<  ".nan\n";
    stats << "    makespan_sec: " <<  ".nan\n";
    stats << "    sum_of_costs_sec: " <<  ".nan\n";
    stats << "    total_control_effort: " <<  ".nan\n";
    stats.flush();
    return false;
  }
  auto end_time = std::chrono::steady_clock::now();
  duration first_run_time = end_time - start_time;
  std::cout << "Time taken (total): " << first_run_time.count() * 1000 << " ms" << std::endl;
  double cost = solution.get_cost();
  double makespan = solution.get_makespan_steps();
  double control_effort = solution.get_control_effort();
  time_planner.print();
  solution.to_yaml_format(outputFile.c_str());
  // save stats
  stats << "stats: " << "\n";
  stats << "  - instance: " << instanceName << "\n";
  stats << "    success: " << true << "\n";
  stats << "    elapsed_time_sec: " << first_run_time.count() << "\n";
  stats << "    makespan_sec: " << makespan << "\n";
  stats << "    sum_of_costs_sec: " << cost * 0.1 << "\n";
  stats << "    total_control_effort: " << control_effort << "\n";
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
      std::uniform_int_distribution<size_t> dist(1, all_ids.size() - 1);
      size_t rand_len = dist(gen);
      std::vector<int> ids_tmp(all_ids.begin(), all_ids.begin() + rand_len);

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
      LaCAM lacam_tmp(problem_tmp, dbN_start_tmp, expanders, heuristics_rev_tmp, planner_options, robots_tmp, time_planner_tmp, dynamic_obstacles, /*verbose*/ 1, &deadline_tmp);
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