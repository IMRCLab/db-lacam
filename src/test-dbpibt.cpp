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
  YAML::Node cfg = YAML::LoadFile(cfgFile);
  cfg = cfg["pibt"]["default"];
  // //  search options
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
  // consider a single robot case. If multi-robot, then loop over
  size_t robot_id = 0;
  const auto robot_type = problem.robotTypes[robot_id];
  // create the robot
  std::vector<std::shared_ptr<dynobench::Model_robot>> robots; // mrs case
  std::shared_ptr<dynobench::Model_robot> robot = dynobench::robot_factory(
      (problem.models_base_path + robot_type + ".yaml").c_str(), problem.p_lb,
      problem.p_ub);
  robots.push_back(robot);
  load_env(*robot, problem);
  // read motions
  std::string motionsFile;
  motionsFile = "../new_format_motions/unicycle1_v0/unicycle1_v0.msgpack";
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
    dynobench::Problem problem_original(inputFile);
    options_pibt.delta = cfg["heuristic1_delta"].as<float>();
    robot_id = 0; // mrs case. For a single robot we assume robot_id=0
    Out_info out_pibt;
    std::vector<dynobench::Trajectory> expanded_trajs_tmp; // not used
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
    // put back settings
    problem.starts = problem_original.starts;
    problem.goals = problem_original.goals;
    options_pibt.delta = cfg["delta_0"].as<float>();
    robot_id = 0; // single robot
  }
  // heuristics
  std::function<bool(Eigen::Ref<Eigen::VectorXd>)> ff =
      [&](Eigen::Ref<Eigen::VectorXd> state)
  {
    return robot->is_state_valid(state);
  };
  std::shared_ptr<Heu_fun> h_fun = nullptr;
  h_fun =
      std::make_shared<Heu_roadmap_bwd<std::shared_ptr<AStarNode>, AStarNode>>(
          robot, heuristics[robot_id], problem.goals[robot_id]);
  // 1. read and find applicable motions
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
    T_m = nigh_factory_t<Motion *>(problem.robotTypes[robot_id], robot,
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
  // d. add the expander
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
  std::vector<std::shared_ptr<AStarNode>> all_nodes;
  all_nodes.push_back(std::make_shared<AStarNode>());
  //
  auto start_node = all_nodes.at(0);
  start_node->gScore = 0;
  start_node->state_eig = problem.starts[robot_id];
  start_node->hScore = h_fun->h(problem.starts[robot_id]);
  start_node->fScore = start_node->gScore + start_node->hScore;
  start_node->is_in_open = true;
  start_node->reaches_goal =
      (robot->distance(problem.starts[robot_id], problem.goals[robot_id]) <=
       options_pibt.delta);
  //
  DYNO_CHECK_GEQ(start_node->hScore, 0, "hScore should be positive");
  DYNO_CHECK_LEQ(start_node->hScore, 1e5, "hScore should be bounded");
  //
  auto goal_node = std::make_shared<AStarNode>();
  goal_node->state_eig = problem.goals[robot_id];
  // open set from db-cbs
  open_t open;
  start_node->handle = open.push(start_node);
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
    if (open.empty())
    {
      status = Terminate_status::EMPTY_QUEUE;
      std::cout << "BREAK search:"
                << "EMPTY_QUEUE" << std::endl;
      return true;
    }
    //
    return false;
  };
  // 2. run the pibt with hard-coded constraints
  dynobench::Trajectory traj_output;
  std::shared_ptr<AStarNode> best_node;
  auto tmp_node = std::make_shared<AStarNode>();
  double hScore, gScore, cost_motion;
  while (!stop_search())
  {
    best_node = open.top();
    open.pop();
    best_node->is_in_open = false;
    double distance_to_goal =
        robot->distance(best_node->state_eig, problem.goals[robot_id]);
    std::cout << "distance to goal: " << distance_to_goal << std::endl;
    if (distance_to_goal < options_pibt.delta_factor_goal *
                               options_pibt.delta)
    { // !!! Assuming db here !!!
      std::cout << "FOUND SOLUTION" << std::endl;
      break;
    } // 2.a find applicable motions
    std::vector<LazyTraj> lazy_trajs;
    time_bench.time_lazy_expand += timed_fun_void(
        [&]
        { expander.expand_lazy(best_node->state_eig, lazy_trajs); });
    // 2.b pass it to pibt (the same set since it's homogeneous robots)
    assert(lazy_trajs.size());
    // 2.c filter lazy_trajs - convert to <TrajWrappers> and sort lazy_trajs
    // based on f=g+h value
    std::vector<dynobench::TrajWrapper> traj_wrappers;
    for (size_t i = 0; i < lazy_trajs.size(); i++)
    {
      auto &lazy_traj = lazy_trajs[i];
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
          h_fun->h(tmp_node->state_eig); // for the last state of the motion
      cost_motion = (traj_wrapper.get_size() - 1) * robot->ref_dt;
      gScore = best_node->gScore + cost_motion;
      traj_wrapper.last_state_f = gScore + hScore;
      traj_wrappers.push_back(traj_wrapper);
    }
    // sort the valid trajs based on f-value
    dynobench::TrajWrapper::SortByLastStateF(traj_wrappers);
    std::cout << "current state before: "
              << best_node->state_eig.format(dynobench::FMT) << std::endl;
    if (!pibt(traj_wrappers, *robot, traj_output, best_node->state_eig))
    {
      std::cout << "pibt failed to use a motion!" << std::endl;
      break;
    }
    // always add new node, not rewiring yet
    bool reachesGoal =
        robot->distance(tmp_node->state_eig, problem.goals[robot_id]) <=
        options_pibt.delta;
    all_nodes.push_back(std::make_shared<AStarNode>());
    auto __node = all_nodes.back();
    __node->state_eig = traj_output.states.back(); // last state
    __node->gScore = gScore;
    __node->hScore = hScore;
    __node->fScore = gScore + hScore;
    // if (chosen_index != -1)
    //   __node->intermediate_state = chosen_index;
    __node->is_in_open = true;
    __node->reaches_goal = reachesGoal;
    __node->handle = open.push(__node);
  }
  // safe the output
  std::ofstream out(outputFile);
  out << "result:" << std::endl;
  out << "-" << std::endl;
  traj_output.to_yaml_format(out, "    ");
  return 0;
}
