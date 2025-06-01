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
// DYNOBENCH
#include "dynobench/general_utils.hpp"
#include "dynobench/motions.hpp"
#include "dynobench/robot_models.hpp"
#include "dynobench/robot_models_base.hpp"
#include "dynobench/multirobot_trajectory.hpp"

// OMPL headers
#include "ompl/base/Path.h"
#include "ompl/base/ScopedState.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
// custom
#include "db-pibt.hpp"
#include "nigh_custom_spaces.hpp"
#include "motion.hpp"
#include "db_options.hpp"
#include "tdbastar.hpp"

namespace fs = std::filesystem;
#define DYNOBENCH_BASE "../dynobench/"
using duration = std::chrono::duration<double>;

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
  Options_dbastar options_pibt; // fine to use tdbastar options
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
  Out_info out_pibt;
  YAML::Node env = YAML::LoadFile(inputFile);
  // create robots
  std::vector<std::shared_ptr<dynobench::Model_robot>> robots;
  for (size_t k = 0; k < problem.robotTypes.size(); k++)
  {
    std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
        (problem.models_base_path + problem.robotTypes.at(k) + ".yaml").c_str(), problem.p_lb,
        problem.p_ub);
    robots.push_back(robot);
  }
  std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
      (problem.models_base_path + problem.robotTypes.at(0) + ".yaml").c_str(), problem.p_lb,
      problem.p_ub);
  load_env(*robot, problem);
  // read motions - homogeneous for now
  std::string motionsFile;
  motionsFile = "../new_format_motions/unicycle1_v0/unit_length/unicycle1_v0.bin.im.bin.sp.bin";
  std::vector<Motion> motions;
  options_pibt.motionsFile = motionsFile;
  load_motion_primitives_new(options_pibt.motionsFile, *robot, motions,
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
    Out_info out_pibt;
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
  // heuristics - homogeneous case
  std::function<bool(Eigen::Ref<Eigen::VectorXd>)> ff =
      [&](Eigen::Ref<Eigen::VectorXd> state)
  {
    return robot->is_state_valid(state);
  };
  std::vector<std::shared_ptr<Heu_fun>> robot_hfun;
  // a. check motions
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
  // b. setup the Tm - for finding applicable motions
  ompl::NearestNeighbors<Motion *> *T_m = nullptr;
  if (options_pibt.use_nigh_nn)
  {
    T_m = nigh_factory_t<Motion *>(problem.robotTypes[0], robot, // homogeneous case
                                   /*reverse_search*/ false);
  }
  else
  {
    NOT_IMPLEMENTED;
  }
  // c. add all motions to Tm
  for (size_t i = 0; i < std::min(motions.size(), options_pibt.max_motions);
       ++i)
  {
    T_m->add(&motions.at(i));
  }
  // d. add the expander, homogeneous case
  Expander expander(robot.get(), T_m, options_pibt.alpha * options_pibt.delta);
  if (options_pibt.alpha <= 0 || options_pibt.alpha >= 1)
  {
    ERROR_WITH_INFO("Alpha needs to be between 0 and 1!");
  }
  //
  if (options_pibt.delta < 0)
  {
    NOT_IMPLEMENTED; // HERE i could compute delta based on desired branching
                     // factor!
  }
  // d. allocate trajectory for the longest motion primitive
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
    //
    traj_wrapper.allocate_size(max_traj_size, robot->nx, robot->nu);
  }
  // e. update the terminating condition accordingly
  Time_benchmark time_bench;
  Stopwatch watch;
  Terminate_status status = Terminate_status::UNKNOWN;
  // manage nodes
  std::vector<open_t> robot_opens(robots.size());
  std::unordered_map<int, std::vector<std::shared_ptr<AStarNode>>> robot_nodes;
  for (size_t i = 0; i < robots.size(); i++)
  {
    std::shared_ptr<Heu_fun> h_fun = nullptr;
    h_fun =
        std::make_shared<Heu_roadmap_bwd<std::shared_ptr<AStarNode>, AStarNode>>(
            robots.at(i), heuristics[i], problem.goals[i]);
    robot_hfun.push_back(h_fun);
    robot_nodes[i].push_back(std::make_shared<AStarNode>());
    auto start_node = robot_nodes[i].at(0);
    start_node->gScore = 0;
    start_node->state_eig = problem.starts[i];
    start_node->hScore = robot_hfun.at(i)->h(problem.starts[i]);
    start_node->fScore = start_node->gScore + start_node->hScore;
    start_node->is_in_open = true;
    start_node->reaches_goal =
        (robots.at(i)->distance(problem.starts[i], problem.goals[i]) <=
         options_pibt.delta);
    DYNO_CHECK_GEQ(start_node->hScore, 0, "hScore should be positive");
    DYNO_CHECK_LEQ(start_node->hScore, 1e5, "hScore should be bounded");
    // auto goal_node = std::make_shared<AStarNode>();
    // goal_node->state_eig = problem.goals[i];
    start_node->handle = robot_opens.at(i).push(start_node);
  }
  auto stop_search = [&]
  {
    if (static_cast<size_t>(time_bench.expands) >= options_pibt.max_expands)
    {
      status = Terminate_status::MAX_EXPANDS;
      std::cout << "BREAK search:"
                << "MAX_EXPANDS" << std::endl;
      return true;
    }
    //
    if (watch.elapsed_ms() > options_pibt.search_timelimit)
    {
      status = Terminate_status::MAX_TIME;
      std::cout << "BREAK search:"
                << "MAX_TIME" << std::endl;
      return true;
    }
    //
    if (std::any_of(robot_opens.begin(), robot_opens.end(), [](const auto &elem)
                    { return elem.empty(); }))
    {
      status = Terminate_status::EMPTY_QUEUE;
      std::cout << "BREAK search:"
                << "EMPTY_QUEUE" << std::endl;
      return true;
    }
    //
    return false;
  };
  // 2. run the pibt
  std::shared_ptr<AStarNode>
      best_node;
  auto tmp_node = std::make_shared<AStarNode>();
  double hScore, gScore, cost_motion;
  std::vector<LazyTraj> lazy_trajs;
  std::vector<dynobench::TrajWrapper> traj_wrappers;
  MultiRobotTrajectory output_trajs;
  output_trajs.trajectories.resize(robots.size());
  std::vector<dynobench::Trajectory> trajs_constrained;
  int reached_goal = 0;
  while (!stop_search())
  {
    trajs_constrained.clear(); // first robot has no constraints for each new itr.
    for (size_t i = 0; i < robots.size(); i++)
    {                                      // in a consecutive manner, no priority for now
      best_node = robot_opens.at(i).top(); // open set for each robot
      robot_opens.at(i).pop();
      best_node->is_in_open = false;
      double distance_to_goal =
          robots.at(i)->distance(best_node->state_eig, problem.goals[i]);
      std::cout << "distance to goal for a robot " << i << " is: " << distance_to_goal << std::endl;
      if (distance_to_goal < options_pibt.delta_factor_goal *
                                 options_pibt.delta)
      {
        std::cout << "FOUND SOLUTION FOR A ROBOT " << i << std::endl;
        reached_goal++;
        if (reached_goal == robots.size())
          break;
        continue;
      }
      lazy_trajs.clear();
      traj_wrappers.clear();
      time_bench.time_lazy_expand += timed_fun_void(
          [&]
          { expander.expand_lazy(best_node->state_eig, lazy_trajs); });
      assert(lazy_trajs.size());
      // separate for each robot
      for (size_t j = 0; j < lazy_trajs.size(); j++)
      {
        auto &lazy_traj = lazy_trajs[j];
        traj_wrapper.set_size(lazy_traj.motion->traj.states.size());
        int num_valid_states = -1;
        lazy_traj.compute(traj_wrapper, /*forward*/ true, /*check_state*/ &ff,
                          &num_valid_states);
        if (num_valid_states && num_valid_states < 1)
        {
          continue;
        }
        // I need to check for state violations !!!
        tmp_node->state_eig = traj_wrapper.get_state(traj_wrapper.get_size() - 1);
        hScore =
            robot_hfun.at(i)->h(tmp_node->state_eig); // for the last state of the motion
        cost_motion = (traj_wrapper.get_size() - 1) * robots.at(i)->ref_dt;
        gScore = best_node->gScore + cost_motion;
        traj_wrapper.last_state_f = gScore + hScore;
        traj_wrappers.push_back(traj_wrapper);
      } // lazy_trajs are read
      dynobench::TrajWrapper::SortByLastStateF(traj_wrappers);
      dynobench::Trajectory tmp_traj_output;
      auto start_pibt = std::chrono::steady_clock::now();
      if (!pibt(/*sorted, applicable motions*/ traj_wrappers, *robot, tmp_traj_output,
                best_node->state_eig, trajs_constrained))
      {
        std::cout << "pibt failed to use a motion for a robot " << i << std::endl;
        break;
      }
      auto end_pibt = std::chrono::steady_clock::now();
      duration duration_pibt = end_pibt - start_pibt;
      std::cout << "Time taken for a single run pibt: " << duration_pibt.count() * 1000 << " ml seconds" << std::endl;

      trajs_constrained.push_back(tmp_traj_output); // for neighbor robots as constraint
      // save it as a solution
      output_trajs.trajectories.at(i).states.insert(output_trajs.trajectories.at(i).states.end(),
                                                    tmp_traj_output.states.begin(), tmp_traj_output.states.end());

      output_trajs.trajectories.at(i).actions.insert(output_trajs.trajectories.at(i).actions.end(),
                                                     tmp_traj_output.actions.begin(), tmp_traj_output.actions.end());

      bool reachesGoal =
          robots.at(i)->distance(tmp_node->state_eig, problem.goals[i]) <=
          options_pibt.delta;
      robot_nodes[i].push_back(std::make_shared<AStarNode>());
      auto __node = robot_nodes[i].back();
      __node->state_eig = output_trajs.trajectories.at(i).states.back(); // last state
      __node->gScore = gScore;
      __node->hScore = hScore;
      __node->fScore = gScore + hScore;
      __node->is_in_open = true;
      __node->reaches_goal = reachesGoal;
      __node->handle = robot_opens.at(i).push(__node);
    }
  }
  auto end_time = std::chrono::steady_clock::now();
  duration duration_total = end_time - start_time;
  std::cout << "Time taken total: " << duration_total.count() << " seconds" << std::endl;
  output_trajs.to_yaml_format(outputFile.c_str());
  return 0;
}