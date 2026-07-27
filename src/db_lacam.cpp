#include "utils.hpp"
#include <cassert>
#include <chrono>
#include "db_lacam.hpp"
#include "db_pibt.hpp"
#include "est_planner.hpp"

bool LaCAM::ANYTIME = false;
float LaCAM::RANDOM_INSERT_PROB1 = 1 * 0.001; // 4 within 1000 runs
float LaCAM::RANDOM_INSERT_PROB2 = 1 * 0.001; // 4 within 1000 runs
std::mt19937 MT(std::random_device{}());
std::uniform_real_distribution<double> rrd(0.0, 1.0);

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
  clearSearchTree();
}

LaCAM::LaCAM(const dynobench::Problem _problem,
             std::vector<std::shared_ptr<AStarNode>> _dbNodes,
             std::vector<Expander> &_expanders,
             std::vector<ompl::NearestNeighbors<std::shared_ptr<AStarNode>> *> &_heuristics_nn,
             Planner_options _planner_options,
             std::vector<std::shared_ptr<dynobench::Model_robot>> _robots,
             Time_planner &_time_planner,
             MultiRobotTrajectory _dynamic_obstacles,
             int _verbose,
             const Deadline *_timelimit)
    : problem(_problem),
      dbNodes(_dbNodes),
      expanders(_expanders),
      heuristics_nn(_heuristics_nn),
      planner_options(_planner_options),
      robots(_robots),
      timelimit(_timelimit),
      verbose(_verbose),
      m_time_planner(_time_planner),
      dynamic_obstacles(_dynamic_obstacles),
      db_pibt(robots, _time_planner, dynamic_obstacles),
      H_goal(nullptr),
      OPEN(),
      order(robots.size(), 0),
      loop_cnt(0)

{
  // tmp_traj_wrapper.allocate_size(/*max_traj_size*/ 100, robots.at(0)->nx, robots.at(0)->nu);
  heuristics.resize(robots.size());
  fake_traj_wrappers.resize(robots.size());

  for (size_t i = 0; i < robots.size(); ++i)
  {
    fake_traj_wrappers[i].allocate_size(/*max_traj_size*/ 100, robots.at(i)->nx, robots.at(i)->nu);
    heuristics[i] = nigh_factory2<std::shared_ptr<AStarNode>>(problem.robotTypes[i], robots[i]);
    auto h_fun = std::make_shared<HeuRoadmapBwdNearestR<std::shared_ptr<AStarNode>, AStarNode>>(
        robots[i], heuristics[i], problem.goals[i], /*use_nn*/ true);

    h_funs.push_back(h_fun);
  }
  if (!dynamic_obstacles.is_empty())
  {
    for (auto &traj : dynamic_obstacles.trajectories)
    {
      max_T = std::max(max_T, traj.states.size());
    }
    std::cout << "max_T: " << max_T << std::endl;
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
    if (loop_cnt > 20000)
    {
      if (ANYTIME)
        break;
      return solution;
    }
    if (H_goal != nullptr)
    {
      auto r = rrd(MT);
      if (r < RANDOM_INSERT_PROB1)
      {
        std::cout << "Random Insert" << std::endl;
        OPEN.push_front(H_init);
      }
      else if (r < RANDOM_INSERT_PROB2)
      {
        auto H = OPEN[get_random_int(MT, 0, OPEN.size() - 1)];
        OPEN.push_front(H);
      }
    }
    // do not pop here!
    auto H = OPEN.front(); // high-level node
    H->get_sum_g();        // update its g
    bool livelock = false;
    // check uppwer bounds
    if (H_goal != nullptr && H->sum_g >= H_goal->sum_g)
    {
      OPEN.pop_front();
      solver_info(5, "prune, sum_g=", H->sum_g, " >= ", H_goal->sum_g);
      OPEN.push_front(H_init);
      continue;
    }

    invalid_node = false;
    std::cout << "Loop count: " << loop_cnt << std::endl;
    std::cout << "HL Node ID: " << OPEN.front()->id << std::endl;
    m_time_planner.time_sort_order += timed_fun_void([&]
                                                     { H->order = get_sorted_order(robots, H->Q, problem.goals); });
    if (H->parent != nullptr)
      // check goal condition
      if (H_goal == nullptr && is_close_config(H, problem.goals, /*goal_threshold*/ 0.5))
      {
        H_goal = H;
        solver_info(2, "found solution!");
        if (!ANYTIME)
          break;
        continue;
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
      livelock = false;
      if (H->livelock && std::find(H->unguided.begin(), H->unguided.end(), r) != H->unguided.end())
        livelock = true;
      get_applicable_trajs_precise_exhaustive(H->dbN[r], rolled_robot_data[r], r, livelock);
      // if no applicable motions for the current state, then remove the node
      if (!rolled_robot_data[r].trajectories.size())
      {
        std::cout << "Invalid Node: " << OPEN.front()->id << std::endl;
        std::cout << "robot " << r << " has no applicable motions" << std::endl;
        OPEN.pop_front();
        invalid_node = true;
        break;
      }

      dbN_to.push_back(std::make_shared<AStarNode>());
      double distance_to_goal = robots[r]->distance(H->dbN[r]->state_eig, problem.goals[r]);
      std::cout << "robot " << r << " distance to goal: " << distance_to_goal << std::endl;
    }
    // HL Nodes can be revisitied, fix it?
    H->livelock = false;
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
    auto iter = EXPLORED.find(Q_to);
    if (iter == EXPLORED.end())
    {
      // new one -> insert
      order.clear();
      // order = get_sorted_order(robots, Q_to, problem.goals);
      h_id++;
      auto H_new = new HNode(h_id, Q_to, dbN_to, order, H);
      OPEN.push_front(H_new);
      EXPLORED[H_new->Q] = H_new;
    }
    else
    {
      // known configuration
      std::cout << "Config is already explored!" << std::endl;
      if (rrd(MT) >= RANDOM_INSERT_PROB1)
      {
        OPEN.push_front(iter->second); // usual
      }
      else
      {
        solver_info(3, "random restart");
        OPEN.push_front(H_init); // sometimes
      }
    }
    // check for lifelock detection
    if (H->parent != nullptr && H->parent->parent != nullptr && H->parent->parent->parent != nullptr)
    {
      HNode *Node_ans = H->parent;
      Node_ans->unguided.clear();
      for (size_t p = 0; p < robots.size(); p++)
      {
        if (robots[0]->distance(Q_to[p], problem.goals[p]) > 0.75)
        {
          double h0 = H->dbN[p]->hScore;
          double h1 = H->parent->dbN[p]->hScore;
          double h2 = H->parent->parent->dbN[p]->hScore;
          double h3 = H->parent->parent->parent->dbN[p]->hScore;

          double d1 = h1 - h0;
          double d2 = h2 - h1;
          double d3 = h3 - h2;
          // check oscillation pattern
          bool oscillating = (d1 * d2 < 0 && d2 * d3 < 0);
          if (oscillating)
          {
            std::cout << "robot " << p << " livelock" << std::endl;
            Node_ans->unguided.push_back(p);
          }
        }
      }
      if (Node_ans->unguided.size() != 0)
      {
        auto H = OPEN.front();
        H->livelock = true;
        H->unguided = Node_ans->unguided;
      }
    }
  }
  if (OPEN.empty())
  {
    std::cout << "open set is empty" << std::endl;
    return solution;
  }
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
    size_t p = 0;
    // end of the motion is the beginning of the next one, that's why duplicate states when the stitch
    for (const auto &seg : segments)
    {
      for (size_t id = 0; id < seg.size(); ++id)
      {
        auto &traj_out = solution.trajectories[id];
        const auto &seg_traj = seg[id];
        if (p != 0)
          traj_out.states.insert(traj_out.states.end(), seg_traj.states.begin() + 1, seg_traj.states.end());
        else
          traj_out.states.insert(traj_out.states.end(), seg_traj.states.begin(), seg_traj.states.end());
        traj_out.actions.insert(traj_out.actions.end(), seg_traj.actions.begin(), seg_traj.actions.end());
      }
      p++;
    }
  }
  solution.trimTrajectories(); // remove zeros action-states
  double cost = solution.get_cost();
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

// exhaustive, does check all motions for collision, clustering
void LaCAM::get_applicable_trajs_precise_exhaustive(std::shared_ptr<AStarNode> db_node, RobotData &robot_data, size_t robot_id, bool livelock = false)
{
  // clear
  tmp_lazy_trajs.clear();
  tmp_traj_wrappers.clear();
  robot_data.clear();
  tmp_traj_wrapper = fake_traj_wrappers[robot_id];
  // i. expand applicable motions
  expanders[robot_id].expand_lazy(db_node->state_eig, tmp_lazy_trajs);
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
    if (num_valid_states < 1)
    {
      // std::cout << "num_valid_states failed" << std::endl;
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
      // std::cout << "rollout, state violations" << std::endl;
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
      est(traj.goal, problem, planner_options, robots[robot_id], robot_id, last_state_h,
          heuristics_nn[robot_id], nullptr, *heuristics[robot_id], /*reverse search*/ false);
      if (last_state_h == -1.0)
      {
        continue;
      }
    }
    tmp_data.last_state_h.push_back(last_state_h);
    if (last_state_h < min_h)
      min_h = last_state_h;
    if (last_state_h > max_h)
      max_h = last_state_h;
  }
  if (livelock)
  {
    robot_data = GetTopNPerClusterByRelativeDistance(tmp_data, 1, /*threshold*/ planner_options.cluster_distance);
  }
  else
    robot_data = GetTopNPerClusterByH(tmp_data, /*range*/ planner_options.cluster_range, min_h, max_h, planner_options.cluster_n, 
                  /*shuffle*/ planner_options.cluster_shuffle, planner_options.seed);
}

// 1. cluster based on state distance
// 2. sorted based on h afterwards
// 3. inner-out style re-arranging, where the middle element is the best.
RobotData LaCAM::GetTopNPerClusterByRelativeDistance(
    const RobotData &input,
    size_t N,
    double threshold)
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
  data.reserve(input.trajectories.size());
  for (size_t i = 0; i < input.trajectories.size(); ++i)
  {
    data.push_back({i, input.last_state_h[i], input.last_state_g[i], input.trajectories[i]});
  }

  // ---- 1) Pick one reference trajectory
  // 1) randomly
  // std::random_device rd;
  // std::mt19937 gen(rd());
  // std::uniform_int_distribution<size_t> dist(0, data.size() - 1);
  // size_t ref_idx = dist(gen);
  // const auto &reference = data[ref_idx];

  // 2) min hScore
  auto ref_it = std::min_element(data.begin(), data.end(), [](const IndexedData &a, const IndexedData &b)
                                 { return a.h < b.h; });
  const auto &reference = *ref_it;

  // ---- 2) Compute distances to reference
  struct WithDistance
  {
    IndexedData d;
    double distance;
  };

  std::vector<WithDistance> with_dist;
  with_dist.reserve(data.size());
  for (auto &d : data)
  {
    double dist_val = robots[0]->distance(reference.traj.states.back(), d.traj.states.back());
    with_dist.push_back({d, dist_val});
  }

  // ---- 3) Cluster based on threshold
  // Example: binning into multiples of threshold
  std::map<int, std::vector<IndexedData>> clusters;
  for (auto &wd : with_dist)
  {
    int bin = static_cast<int>(wd.distance / threshold);
    clusters[bin].push_back(wd.d);
  }

  RobotData result;

  // ---- 4) Process each cluster
  for (auto &[bin, cluster] : clusters)
  {
    // sort by h
    std::sort(cluster.begin(), cluster.end(), [](const IndexedData &a, const IndexedData &b)
              {
                if (a.h != b.h)
                  return a.h < b.h;
                return a.g < b.g; });

    // top-N
    size_t take = std::min(N, cluster.size());
    std::vector<IndexedData> selected(cluster.begin(), cluster.begin() + take);

    // inside-out rearrangement
    std::vector<IndexedData> reordered;
    reordered.reserve(selected.size());

    int mid = selected.size() / 2;
    reordered.push_back(selected[mid]);
    for (int offset = 1; offset <= mid; ++offset)
    {
      if (mid - offset >= 0)
        reordered.push_back(selected[mid - offset]);
      if (mid + offset < (int)selected.size())
        reordered.push_back(selected[mid + offset]);
    }

    // append
    for (const auto &r : reordered)
    {
      result.trajectories.push_back(r.traj);
      result.last_state_h.push_back(r.h);
      result.last_state_g.push_back(r.g);
    }
  }

  return result;
}

RobotData LaCAM::GetTopNPerClusterByH(const RobotData &input, double range, double min_h, double max_h, size_t N, bool shuffle, std::optional<unsigned int> seed)
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
    if (shuffle)
    {
      std::mt19937 rng(seed ? *seed : std::random_device{}());
      std::shuffle(current_cluster.begin(), current_cluster.end(), rng);
    }
    size_t take = std::min(N, current_cluster.size());
    for (size_t i = 0; i < take; ++i)
    {
      result.trajectories.push_back(current_cluster[i].traj);
      result.last_state_h.push_back(current_cluster[i].h);
      result.last_state_g.push_back(current_cluster[i].g);
    }
  }
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

// allow threshold for reaching the goal for now. Homogeneous robots
bool LaCAM::is_close_config(HNode *S, std::vector<Eigen::VectorXd> Q2, double threshold)
{
  std::vector<Eigen::VectorXd> Q1 = S->Q;
  assert(Q1.size() == Q2.size());
  int cnt = 0;
  for (size_t i = 0; i < Q1.size(); i++)
  {
    if (robots[0]->distance(Q1.at(i), Q2.at(i)) <= threshold && S->dbN.at(i)->gScore / robots[0]->ref_dt >= max_T)
    {
      // std::cout << "close to final, robot : " << i << ", dist.: " << robots[0]->distance(Q1.at(i), Q2.at(i)) << std::endl;
      cnt++;
      S->dbN[i]->reaches_goal = true;
    }
    else
      S->dbN[i]->reaches_goal = false;
  }
  return (cnt == Q1.size()) ? true : false;
}

bool LaCAM::is_close_to_goal(std::vector<Eigen::VectorXd> Q, double threshold)
{
  int cnt = 0;
  for (size_t i = 0; i < Q.size(); i++)
  {
    if (robots[0]->distance(Q.at(i), problem.goals.at(i)) <= threshold)
      cnt++;
  }
  return (cnt == Q.size()) ? true : false;
}