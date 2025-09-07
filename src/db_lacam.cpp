#include "utils.hpp"
#include <cassert>
#include <chrono>
#include "db_lacam.hpp"
#include "db_pibt.hpp"
#include "est_planner.hpp"

LNode::LNode() : who(), where(), where_state(), where_dbN(), depth(0) {}

LNode::LNode(LNode *parent, int i, dynobench::Trajectory v, Eigen::VectorXd v_s, std::shared_ptr<AStarNode> v_dbN)
    : who(parent->who), where(parent->where), where_state(parent->where_state), where_dbN(parent->where_dbN), depth(parent->depth + 1)
{
  who.push_back(i);
  where.push_back(v);
  where_state.push_back(v_s);
  where_dbN.push_back(v_dbN);
}

LNode::~LNode() {};

HNode::HNode(int _id, std::vector<Eigen::VectorXd> _Q, std::vector<std::shared_ptr<AStarNode>> _dbN, std::vector<int> _order, HNode *_parent)
    : Q(_Q),
      dbN(_dbN),
      M_to(Q.size()),
      parent(_parent),
      depth(parent == nullptr ? 0 : parent->depth + 1),
      order(_order),
      search_tree(),
      id(_id)
{
  search_tree.push(new LNode());
}

HNode::~HNode()
{
  while (!search_tree.empty())
  {
    delete search_tree.front();
    search_tree.pop();
  }
}

LaCAM::LaCAM(const dynobench::Problem _problem,
             std::vector<std::shared_ptr<AStarNode>> _dbNodes,
             Expander &_expander,
             std::vector<ompl::NearestNeighbors<std::shared_ptr<AStarNode>> *> &_heuristics_nn,
             Planner_options _planner_options,
             std::vector<std::shared_ptr<dynobench::Model_robot>> _robots,
             Time_planner &_time_planner,
             int _verbose,
             const Deadline *_timelimit)
    : problem(_problem),
      dbNodes(_dbNodes),
      expander(_expander),
      heuristics_nn(_heuristics_nn),
      planner_options(_planner_options),
      robots(_robots),
      timelimit(_timelimit),
      verbose(_verbose),
      m_time_planner(_time_planner),
      db_pibt(robots, _time_planner),
      H_goal(nullptr),
      OPEN(),
      order(robots.size(), 0),
      loop_cnt(0)

{
  tmp_traj_wrapper.allocate_size(/*max_traj_size*/ 100, robots.at(0)->nx, robots.at(0)->nu);
  heuristics.resize(robots.size());

  for (size_t i = 0; i < robots.size(); ++i)
  {
    heuristics[i] = nigh_factory2<std::shared_ptr<AStarNode>>(problem.robotTypes[i], robots[i]);
    auto h_fun = std::make_shared<HeuRoadmapBwdNearestR<std::shared_ptr<AStarNode>, AStarNode>>(
        robots[i], heuristics[i], problem.goals[i], /*use_nn*/ true);

    h_funs.push_back(h_fun);
  }
}

LaCAM::~LaCAM()
{
  for (auto nn_ptr : heuristics)
  {
    delete nn_ptr; // free memory
  }
  heuristics.clear();
}

MultiRobotTrajectory LaCAM::solve()
{
  solver_info(1, "LaCAM begins");
  auto start_time = std::chrono::steady_clock::now();
  // setup search
  int h_id = 1;
  std::unordered_map<std::vector<Eigen::VectorXd>, HNode *, ConfigHasher, ConfigEqual> EXPLORED;
  // insert initial node
  m_time_planner.time_sort_order += timed_fun_void([&]
                                                   { order = get_sorted_order(robots, problem.starts, problem.goals); });
  auto H_init = new HNode(h_id, problem.starts, dbNodes, order);
  OPEN.push_front(H_init);
  // DEBUG
  double best_distance_to_goal = 5000;
  Eigen::VectorXd best_distance_state;
  EXPLORED[H_init->Q] = H_init;
  bool invalid_node = false;
  // search loop
  solver_info(2, "search iteration begins");
  while (!OPEN.empty() && !is_expired(timelimit))
  {
    ++loop_cnt;
    if (loop_cnt > 1000)
    {
      // export_node_expansion();
      // auto space6 = std::string(6, ' ');
      // for (size_t i = 0; i < heuristics.size(); ++i)
      // {
      //   std::string filename = "heuristics_dblacam_" + std::to_string(i) + ".yaml";
      //   std::ofstream out(filename);
      //   out << "states:" << std::endl;
      //   auto *nn = heuristics[i];
      //   std::vector<std::shared_ptr<AStarNode>> nodes;
      //   nn->list(nodes);
      //   std::cout << "writing " << nodes.size() << " nodes for robot " << i << std::endl;
      //   for (auto &node : nodes)
      //   {
      //     const Eigen::VectorXd &state = node->state_eig;
      //     out << space6 << "  - " << state.format(dynobench::FMT) << std::endl;
      //   }
      // }
      return solution;
    }
    // do not pop here!
    auto H = OPEN.front(); // high-level node
    invalid_node = false;
    std::cout << "Loop count: " << loop_cnt << std::endl;
    std::cout << "HL Node ID: " << OPEN.front()->id << std::endl;
    m_time_planner.time_sort_order += timed_fun_void([&]
                                                     { H->order = get_sorted_order(robots, H->Q, problem.goals); });
    if (H->parent != nullptr)
      // check goal condition
      if (H_goal == nullptr && is_close_config(H->Q, problem.goals, robots.at(0), /*threshold*/ 0.75)) // planner_options.goal_delta
      {
        H_goal = H;
        solver_info(2, "found solution!");
        break;
      }
    // extract constraints
    if (H->search_tree.empty())
    {
      std::cout << "Popping up the HL Node ID: " << OPEN.front()->id << std::endl;
      OPEN.pop_front();
      continue;
    }
    auto L = H->search_tree.front();
    H->search_tree.pop();
    std::vector<std::shared_ptr<AStarNode>> dbN_to;
    for (size_t r = 0; r < robots.size(); r++)
    {
      m_time_planner.time_get_trajs += timed_fun_void([&]
                                                      { get_applicable_trajs_precise_exhaustive(H->dbN[r], rolled_robot_data[r], r); });
      // if no applicable motions for the current state, then remove the node
      if (!rolled_robot_data[r].trajectories.size())
      {
        std::cout << "Invalid Node: " << OPEN.front()->id << std::endl;
        OPEN.pop_front();
        invalid_node = true;
        break;
      }
      dbN_to.push_back(std::make_shared<AStarNode>());
      double distance_to_goal = robots[r]->distance(H->dbN[r]->state_eig, problem.goals[r]);
      std::cout << "robot " << r << " distance to goal: " << distance_to_goal << std::endl;
    }
    if (invalid_node)
      continue;
    // low level search
    if (L->depth < H->Q.size())
    {
      const auto i = H->order[L->depth];
      assert(!rolled_robot_data[i].trajectories.empty());
      m_time_planner.time_grow_search_tree += timed_fun_void([&]
                                                             { 
      for (size_t j = 0; j < rolled_robot_data[i].trajectories.size(); j++)
      {
        dynobench::Trajectory u_traj = rolled_robot_data[i].trajectories[j];
        Eigen::VectorXd u_state = rolled_robot_data[i].trajectories[j].goal; // last state of the motion
        if (!robots[i]->is_state_valid(u_state))
          continue;
        auto u_dbN = std::make_shared<AStarNode>();
        u_dbN->state_eig = u_state;
        u_dbN->gScore = rolled_robot_data[i].last_state_g[j];
        u_dbN->hScore = rolled_robot_data[i].last_state_h[j];
        H->search_tree.push(new LNode(L, i, u_traj, u_state, u_dbN));
      } });
    }
    // create successors at the high-level search
    std::vector<Eigen::VectorXd> Q_to;
    Q_to.resize(robots.size());
    std::vector<dynobench::Trajectory> M_to;
    M_to.resize(robots.size());
    bool res = set_new_config(H, L, Q_to, dbN_to, M_to, rolled_robot_data);
    std::cout << "set new config: " << res << std::endl;
    delete L;
    if (!res)
      continue;
    H->M_to = M_to;
    if (EXPLORED.find(Q_to) != EXPLORED.end())
    {
      std::cout << "Config is already explored, reinserting the node!" << std::endl;
      OPEN.push_front(H);
      continue;
    }
    // always add the node
    if (EXPLORED.find(Q_to) == EXPLORED.end())
    {
      order.clear();
      order = get_sorted_order(robots, Q_to, problem.goals);
      h_id++;
      // H->M_to = M_to;
      auto H_new = new HNode(h_id, Q_to, dbN_to, order, H);
      OPEN.push_front(H_new);
      EXPLORED[H_new->Q] = H_new;
    }
  }
  if (OPEN.empty())
  {
    std::cout << "open set is empty" << std::endl;
    return solution;
  }
  // if OPEN is empty
  // export_node_expansion();
  // auto space6 = std::string(6, ' ');
  // for (size_t i = 0; i < heuristics.size(); ++i)
  // {
  // std::string filename = "heuristics_dblacam_" + std::to_string(i) + ".yaml";
  // std::ofstream out(filename);
  // out << "states:" << std::endl;
  // auto *nn = heuristics[i];
  // std::vector<std::shared_ptr<AStarNode>> nodes;
  // nn->list(nodes);
  // std::cout << "writing " << nodes.size() << " nodes for robot " << i << std::endl;
  // for (auto &node : nodes)
  // {
  // const Eigen::VectorXd &state = node->state_eig;
  // out << space6 << "  - " << state.format(dynobench::FMT) << std::endl;
  // }
  // }
  // backtrack the solution
  {
    auto H = H_goal;
    std::vector<std::vector<dynobench::Trajectory>> segments;
    // Backtrace from H to root
    while (H->parent != nullptr)
    {
      segments.push_back(H->parent->M_to); // M_to belongs to the parent
      H = H->parent;
    }

    std::reverse(segments.begin(), segments.end());

    solution.trajectories.resize(robots.size());
    cost = 0;
    for (const auto &seg : segments)
    {
      for (size_t id = 0; id < seg.size(); ++id)
      {
        auto &traj_out = solution.trajectories[id];
        const auto &seg_traj = seg[id];
        traj_out.states.insert(traj_out.states.end(), seg_traj.states.begin(), seg_traj.states.end());
        traj_out.actions.insert(traj_out.actions.end(), seg_traj.actions.begin(), seg_traj.actions.end());
        cost += traj_out.actions.size();
      }
    }
  }
  auto end_time = std::chrono::steady_clock::now();
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
  std::cout << "elapsed:" << std::setw(6) << elapsed_ms << "ms"
            << "  loop_cnt:" << std::setw(8) << loop_cnt << std::endl;
  double avg_ms = static_cast<double>(elapsed_ms) / loop_cnt;

  std::cout << "per iteration: " << std::fixed << std::setprecision(3)
            << avg_ms << " ms\n";
  std::cout << "cost: " << std::fixed << cost * 0.1 << std::endl;
  return solution;
}
// without rollout. h-vlaue is for the "expected state", but can diverge hugely with unicycle dynamics
void LaCAM::get_applicable_trajs(std::shared_ptr<AStarNode> db_node, RobotData &robot_data, size_t robot_id)
{
  // clear
  tmp_lazy_trajs.clear();
  tmp_traj_wrappers.clear();
  robot_data.clear();
  // i. expand applicable motions
  expander.expand_lazy(db_node->state_eig, tmp_lazy_trajs);
  auto ff = validity_checker(robots[robot_id]);
  int num_valid_states = -1;
  double min_f = std::numeric_limits<double>::max();
  double max_f = std::numeric_limits<double>::lowest();
  double gScore = 0;
  double hScore;
  for (size_t j = 0; j < tmp_lazy_trajs.size(); j++)
  {
    auto &lazy_traj = tmp_lazy_trajs[j];
    tmp_traj_wrapper.set_size(lazy_traj.motion->traj.states.size());
    num_valid_states = -1;
    lazy_traj.compute(tmp_traj_wrapper, /*forward*/ true, /*check_state*/ &ff,
                      &num_valid_states);
    if (num_valid_states && num_valid_states < 1)
    {
      std::cout << "num_valid_states failed" << std::endl;
      continue;
    }
    if (num_valid_states < lazy_traj.motion->traj.states.size())
    {
      continue;
    }
    Eigen::VectorXd tmp_state = tmp_traj_wrapper.get_state(tmp_traj_wrapper.get_size() - 1);
    // hScore = robots[robot_id]->distance(tmp_state, problem.goals[robot_id]); // for single integrator, if the env is small
    hScore = h_funs[robot_id]->h(tmp_state); // for the last state of the motion
    double cost_motion = (tmp_traj_wrapper.get_size() - 1) * robots[robot_id]->ref_dt;
    gScore = db_node->gScore + cost_motion;
    tmp_traj_wrapper.last_state_g = gScore;
    tmp_traj_wrapper.last_state_h = hScore;
    tmp_traj_wrapper.last_state_f = gScore + hScore;
    if (tmp_traj_wrapper.last_state_f < min_f)
      min_f = tmp_traj_wrapper.last_state_f;
    if (tmp_traj_wrapper.last_state_f > max_f)
      max_f = tmp_traj_wrapper.last_state_f;
    tmp_traj_wrappers.push_back(tmp_traj_wrapper);
  }
  // ii. sort/cluster based on f-value
  dynobench::TrajWrapper wr;
  std::vector<dynobench::TrajWrapper> sorted_traj_wrappers = wr.GetTopNPerClusterByLastStateF(tmp_traj_wrappers, /*range*/ 0.3, min_f, max_f, /*N*/ 8);
  // ii. rollout trajs - env collision free
  Eigen::VectorXd x0 = db_node->state_eig;
  for (size_t k = 0; k < sorted_traj_wrappers.size(); k++)
  {
    auto &traj_wrap = sorted_traj_wrappers[k];
    std::vector<Eigen::VectorXd> us = traj_wrap.get_actions();
    std::vector<Eigen::VectorXd> xs(us.size() + 1,
                                    Eigen::VectorXd::Zero(robots[robot_id]->nx));
    int num_valid_states = -1;
    robots[robot_id]->rollout(x0, us, xs, &ff,
                              &num_valid_states);
    if (num_valid_states && num_valid_states < xs.size())
    {
      std::cout << "rollout, state violations" << std::endl;
      continue;
    }
    dynobench::Trajectory traj;
    traj.states.clear();
    traj.actions.clear();
    traj.start = x0;
    traj.states = xs;
    traj.actions = us;
    traj.goal = traj.states.back();
    // check for collision with the env
    Motion motion;
    traj_to_motion(traj, *(robots[robot_id]), motion, /*compute collision*/ true);
    assert(motion.collision_manager);
    assert(robots[robot_id]->env.get());
    fcl::DefaultCollisionData<double> collision_data;
    motion.collision_manager->collide(robots[robot_id]->env.get(), &collision_data,
                                      fcl::DefaultCollisionFunction<double>);
    if (collision_data.result.isCollision())
      continue;

    robot_data.trajectories.push_back(traj);
    // need for the Node update
    robot_data.last_state_g.push_back(traj_wrap.last_state_g);
    robot_data.last_state_h.push_back(traj_wrap.last_state_h);
  }
}
// OPTION 1: no clustering, sort based on h-value
void LaCAM::get_applicable_trajs_precise_no_clustering(std::shared_ptr<AStarNode> db_node, RobotData &robot_data, size_t robot_id)
{
  // clear
  tmp_lazy_trajs.clear();
  robot_data.clear();
  // i. expand applicable motions
  m_time_planner.time_lazy_expand += timed_fun_void(
      [&]
      { expander.expand_lazy(db_node->state_eig, tmp_lazy_trajs); });

  auto ff = validity_checker(robots[robot_id]);
  double gScore = 0;
  double hScore = 0;
  Eigen::VectorXd x0 = db_node->state_eig;
  h_values.clear();
  for (size_t j = 0; j < tmp_lazy_trajs.size(); j++)
  {
    // i. rollout and keep the valid
    auto &lazy_traj = tmp_lazy_trajs[j];
    std::vector<Eigen::VectorXd> us = lazy_traj.motion->traj.actions;
    std::vector<Eigen::VectorXd>
        xs(us.size() + 1,
           Eigen::VectorXd::Zero(robots[robot_id]->nx));
    int num_valid_states = -1;
    m_time_planner.time_rollout += timed_fun_void([&]
                                                  { robots[robot_id]->rollout(x0, us, xs, &ff,
                                                                              &num_valid_states); });
    if (num_valid_states && num_valid_states < xs.size())
    {
      std::cout << "rollout, state violations" << std::endl;
      continue;
    }
    // ii. check for similarity
    hScore = h_funs[robot_id]->h(xs.back());
    double cost_motion = us.size() * robots[robot_id]->ref_dt;
    gScore = db_node->gScore + cost_motion;

    if (!check_and_add(hScore))
      continue;
    // iii. check for collision with the env.
    dynobench::Trajectory traj;
    traj.states.clear();
    traj.actions.clear();
    traj.start = x0;
    traj.states = xs;
    traj.actions = us;
    traj.goal = traj.states.back();

    Motion motion;
    m_time_planner.time_traj_to_motion += timed_fun_void([&]
                                                         { traj_to_motion(traj, *(robots[robot_id]), motion, /*compute collision*/ true); });
    fcl::DefaultCollisionData<double> collision_data;
    m_time_planner.time_collisions += timed_fun_void([&]
                                                     {
    assert(motion.collision_manager);
    assert(robots[robot_id]->env.get());
    motion.collision_manager->collide(robots[robot_id]->env.get(), &collision_data,
                                      fcl::DefaultCollisionFunction<double>); });
    if (collision_data.result.isCollision())
      continue;
    // iv. save it into robot_data
    robot_data.trajectories.push_back(traj);
    robot_data.last_state_g.push_back(gScore);
    robot_data.last_state_h.push_back(hScore);
  }
  robot_data.sort_by_h();
}

// exhaustive, does check all motions for collision, clustering
void LaCAM::get_applicable_trajs_precise_exhaustive(std::shared_ptr<AStarNode> db_node, RobotData &robot_data, size_t robot_id)
{
  std::cout << "robot " << robot_id << " get applicable trajs" << std::endl;
  // clear
  tmp_lazy_trajs.clear();
  tmp_traj_wrappers.clear();
  robot_data.clear();
  // i. expand applicable motions
  expander.expand_lazy(db_node->state_eig, tmp_lazy_trajs);
  auto ff = validity_checker(robots[robot_id]);
  int num_valid_states = -1;
  double gScore = 0;
  double min_h = std::numeric_limits<double>::max();
  double max_h = std::numeric_limits<double>::lowest();
  for (size_t j = 0; j < tmp_lazy_trajs.size(); j++)
  {
    auto &lazy_traj = tmp_lazy_trajs[j];
    tmp_traj_wrapper.set_size(lazy_traj.motion->traj.states.size());
    num_valid_states = -1;
    lazy_traj.compute(tmp_traj_wrapper, /*forward*/ true, /*check_state*/ &ff,
                      &num_valid_states);
    if (num_valid_states && num_valid_states < 1)
    {
      std::cout << "num_valid_states failed" << std::endl;
      continue;
    }
    if (num_valid_states < lazy_traj.motion->traj.states.size())
    {
      continue;
    }
    double cost_motion = (tmp_traj_wrapper.get_size() - 1) * robots[robot_id]->ref_dt;
    gScore = db_node->gScore + cost_motion;
    tmp_traj_wrapper.last_state_g = gScore;
    tmp_traj_wrappers.push_back(tmp_traj_wrapper);
  }
  // ii. rollout trajs - env collision free
  RobotData tmp_data;
  double last_state_h = 0;
  Eigen::VectorXd x0 = db_node->state_eig;
  for (size_t k = 0; k < tmp_traj_wrappers.size(); k++)
  {
    auto &traj_wrap = tmp_traj_wrappers[k];
    std::vector<Eigen::VectorXd> us = traj_wrap.get_actions();
    std::vector<Eigen::VectorXd> xs(us.size() + 1,
                                    Eigen::VectorXd::Zero(robots[robot_id]->nx));
    int num_valid_states = -1;
    m_time_planner.time_rollout += timed_fun_void([&]
                                                  { robots[robot_id]->rollout(x0, us, xs, &ff,
                                                                              &num_valid_states); });
    if (num_valid_states && num_valid_states < xs.size())
    {
      std::cout << "rollout, state violations" << std::endl;
      continue;
    }
    dynobench::Trajectory traj;
    traj.states.clear();
    traj.actions.clear();
    traj.start = x0;
    traj.states = xs;
    traj.actions = us;
    traj.goal = traj.states.back();
    // check for collision with the env
    Motion motion;
    m_time_planner.time_traj_to_motion += timed_fun_void([&]
                                                         { traj_to_motion(traj, *(robots[robot_id]), motion, /*compute collision*/ true, planner_options.merged_aabb); });
    fcl::DefaultCollisionData<double> collision_data;
    m_time_planner.time_collisions += timed_fun_void([&]
                                                     {
    assert(motion.collision_manager);
    assert(robots[robot_id]->env.get());
    motion.collision_manager->collide(robots[robot_id]->env.get(), &collision_data,
                                      fcl::DefaultCollisionFunction<double>); });
    if (collision_data.result.isCollision())
      continue;
    tmp_data.trajectories.push_back(traj);
    tmp_data.last_state_g.push_back(traj_wrap.last_state_g);
    // m_time_planner.time_hfun += timed_fun_void([&]
    //                                            { last_state_h = h_funs[robot_id]->h(traj.goal); });
    last_state_h = h_funs[robot_id]->h(traj.goal);
    if (last_state_h == -1.0)
    {
      std::cout << "using EST for h-value" << std::endl;
      est(traj.goal, problem, planner_options, robots[robot_id], robot_id, last_state_h,
          heuristics_nn[robot_id], nullptr, *heuristics[robot_id], /*reverse search*/ false);
      if (last_state_h == -1.0)
      {
        std::cout << "EST failed to give h-value, skipping the motion" << std::endl;
        continue;
      }
    }
    tmp_data.last_state_h.push_back(last_state_h);
    if (last_state_h < min_h)
      min_h = last_state_h;
    if (last_state_h > max_h)
      max_h = last_state_h;
  }
  std::cout << "tmp data traj size: " << tmp_data.trajectories.size() << std::endl;
  robot_data = GetTopNPerClusterByH(tmp_data, /*range*/ planner_options.cluster_range, min_h, max_h, planner_options.cluster_n, /*shuffle*/ false);
}
// OPTION 3: sort actions based on epsilon
void LaCAM::get_applicable_trajs_precise_sort_actions(std::shared_ptr<AStarNode> db_node, RobotData &robot_data, size_t robot_id)
{
  // clear
  tmp_lazy_trajs.clear();
  robot_data.clear();
  // i. expand applicable motions
  m_time_planner.time_lazy_expand += timed_fun_void(
      [&]
      { expander.expand_lazy(db_node->state_eig, tmp_lazy_trajs); });

  std::vector<std::vector<Eigen::VectorXd>> all_actions;
  all_actions.resize(tmp_lazy_trajs.size());

  std::transform(tmp_lazy_trajs.begin(), tmp_lazy_trajs.end(), all_actions.begin(),
                 [](const LazyTraj &traj)
                 {
                   return traj.motion->traj.actions;
                 });
  double eps = 0.5; // FINE-TUNE
  auto diverse_actions = filter_diverse(all_actions, eps);

  auto ff = validity_checker(robots[robot_id]);
  double gScore = 0;
  double hScore = 0;
  Eigen::VectorXd x0 = db_node->state_eig;
  h_values.clear();
  for (size_t j = 0; j < all_actions.size(); j++)
  {
    // i. rollout and keep the valid
    std::vector<Eigen::VectorXd> us = all_actions[j];
    std::vector<Eigen::VectorXd>
        xs(us.size() + 1,
           Eigen::VectorXd::Zero(robots[robot_id]->nx));
    int num_valid_states = -1;
    m_time_planner.time_rollout += timed_fun_void([&]
                                                  { robots[robot_id]->rollout(x0, us, xs, &ff,
                                                                              &num_valid_states); });
    if (num_valid_states && num_valid_states < xs.size())
    {
      std::cout << "rollout, state violations" << std::endl;
      continue;
    }
    // ii. check for similarity
    hScore = h_funs[robot_id]->h(xs.back());
    double cost_motion = us.size() * robots[robot_id]->ref_dt;
    gScore = db_node->gScore + cost_motion;
    // iii. check for collision with the env.
    dynobench::Trajectory traj;
    traj.states.clear();
    traj.actions.clear();
    traj.start = x0;
    traj.states = xs;
    traj.actions = us;
    traj.goal = traj.states.back();

    Motion motion;
    m_time_planner.time_traj_to_motion += timed_fun_void([&]
                                                         { traj_to_motion(traj, *(robots[robot_id]), motion, /*compute collision*/ true); });
    fcl::DefaultCollisionData<double> collision_data;
    m_time_planner.time_collisions += timed_fun_void([&]
                                                     {
    assert(motion.collision_manager);
    assert(robots[robot_id]->env.get());
    motion.collision_manager->collide(robots[robot_id]->env.get(), &collision_data,
                                      fcl::DefaultCollisionFunction<double>); });
    if (collision_data.result.isCollision())
      continue;
    // iv. save it into robot_data
    robot_data.trajectories.push_back(traj);
    robot_data.last_state_g.push_back(gScore);
    robot_data.last_state_h.push_back(hScore);
  }
  robot_data.sort_by_h();
}

// simply check it some motion with similar h-value has been explored
bool LaCAM::check_and_add(const double h_value)
{
  for (double h : h_values)
  {
    if (std::fabs(h - h_value) < /*eps*/ 0.5)
    {
      return false; // too similar, skip
    }
  }
  h_values.push_back(h_value);
  return true;
}
RobotData LaCAM::GetFilteredUniqueTopByH(const RobotData &input, double min_distance, size_t robot_id)
{
  if (input.trajectories.empty())
    return {};

  struct IndexedData
  {
    size_t index;
    double h;
    double g;
    dynobench::Trajectory traj;
  };

  std::vector<IndexedData> data;
  for (size_t i = 0; i < input.trajectories.size(); ++i)
  {
    data.push_back({i, input.last_state_h[i], input.last_state_g[i], input.trajectories[i]});
  }

  // Sort by h (lowest first)
  std::sort(data.begin(), data.end(), [](const IndexedData &a, const IndexedData &b)
            {
              if (a.h != b.h)
                return a.h < b.h;
              return a.g < b.g; });

  RobotData result;

  for (const auto &d : data)
  {
    const auto &new_final_state = d.traj.states.back();

    bool too_close = false;
    for (const auto &existing_traj : result.trajectories)
    {
      const auto &existing_final_state = existing_traj.states.back(); // (new_final_state - existing_final_state).norm() < min_distance
      if (robots[robot_id]->distance(new_final_state, existing_final_state) < min_distance)
      {
        too_close = true;
        break;
      }
    }

    if (!too_close)
    {
      result.trajectories.push_back(d.traj);
      result.last_state_h.push_back(d.h);
      result.last_state_g.push_back(d.g);
    }
  }

  return result;
}

RobotData LaCAM::GetTopNPerClusterByH(const RobotData &input, double range, double min_h, double max_h, size_t N, bool shuffle = false)
{
  if (input.trajectories.empty())
    return {};

  double threshold = range * (max_h - min_h);
  // Combine into a sortable struct
  struct IndexedData
  {
    size_t index;
    double h;
    double g;
    dynobench::Trajectory traj;
  };

  std::vector<IndexedData> data;
  for (size_t i = 0; i < input.trajectories.size(); ++i)
  {
    data.push_back({i, input.last_state_h[i], input.last_state_g[i], input.trajectories[i]});
  }

  // Sort by h (then optionally g or something else)
  std::sort(data.begin(), data.end(), [](const IndexedData &a, const IndexedData &b)
            {
              if (a.h != b.h)
                return a.h < b.h;
              return a.g < b.g; // tie-breaker
            });

  // Clustering
  RobotData result;
  std::vector<IndexedData> current_cluster;
  double cluster_start_value = data[0].h;

  for (const auto &d : data)
  {
    if (std::fabs(d.h - cluster_start_value) > threshold)
    {
      // Cluster ended, pick top N
      size_t take = std::min(N, current_cluster.size());
      for (size_t i = 0; i < take; ++i)
      {
        result.trajectories.push_back(current_cluster[i].traj);
        result.last_state_h.push_back(current_cluster[i].h);
        result.last_state_g.push_back(current_cluster[i].g);
      }

      current_cluster.clear();
      cluster_start_value = d.h;
    }

    current_cluster.push_back(d);
  }

  // Handle final cluster
  if (!current_cluster.empty())
  {
    size_t take = std::min(N, current_cluster.size());
    for (size_t i = 0; i < take; ++i)
    {
      result.trajectories.push_back(current_cluster[i].traj);
      result.last_state_h.push_back(current_cluster[i].h);
      result.last_state_g.push_back(current_cluster[i].g);
    }
  }
  if (shuffle)
    result.shuffle();
  return result;
}

bool LaCAM::set_new_config(HNode *H, LNode *L, std::vector<Eigen::VectorXd> &Q_to,
                           std::vector<std::shared_ptr<AStarNode>> &dbN_to,
                           std::vector<dynobench::Trajectory> &M_to,
                           std::map<size_t, RobotData> robot_data_rolled)
{
  for (uint d = 0; d < L->depth; ++d)
  {
    Q_to[L->who[d]] = L->where_state[d];
    dbN_to[L->who[d]] = L->where_dbN[d];
    M_to[L->who[d]] = L->where[d];
    std::cout << "constraining the robot " << L->who[d] << std::endl;
    std::cout << Q_to[L->who[d]].format(dynobench::FMT) << std::endl;
  }
  return db_pibt.set_new_config(H->Q, Q_to, H->dbN, dbN_to, M_to, H->order, robot_data_rolled);
}

std::vector<int> LaCAM::get_sorted_order(
    std::vector<std::shared_ptr<dynobench::Model_robot>> &robots,
    const std::vector<Eigen::VectorXd> &states,
    const std::vector<Eigen::VectorXd> &goal_states)
{
  std::vector<int> orders(robots.size());
  std::iota(orders.begin(), orders.end(), 0);

  std::stable_sort(orders.begin(), orders.end(),
                   [&](size_t i, size_t j)
                   {
                     // First: sort by distance (larger first)
                     double di = robots[i]->distance(states[i], goal_states[i]);
                     double dj = robots[j]->distance(states[j], goal_states[j]);
                     if (di != dj)
                       return di > dj;

                     // Second: prioritize if trajectories == 0
                     bool ti = rolled_robot_data[i].trajectories.empty();
                     bool tj = rolled_robot_data[j].trajectories.empty();
                     if (ti != tj)
                       return ti; // true (0 trajs) comes before false

                     // Otherwise keep previous order (stable_sort guarantees this)
                     return false;
                   });

  return orders;
}

void LaCAM::export_node_expansion()
{
  for (const auto &kv : expanded_trajs)
  {
    size_t key = kv.first;
    const auto &traj_list = kv.second;

    // build filename: e.g. node_expansion_key_0.yaml
    std::string filename = "node_expansion_key_" + std::to_string(key) + ".yaml";
    std::ofstream out(filename);

    if (!out.is_open())
    {
      std::cerr << "Failed to open file: " << filename << std::endl;
      continue;
    }

    out << "trajs:" << std::endl;
    for (const auto &traj : traj_list)
    {
      out << "  - " << std::endl;
      traj.to_yaml_format(out, "    ");
    }

    std::cout << "Wrote " << traj_list.size()
              << " trajectories to " << filename << std::endl;
  }
}
