#include "db_lacam.hpp"
#include "db_pibt.hpp"
#include "utils.hpp"
#include <cassert>

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

HNode::HNode(std::vector<Eigen::VectorXd> _Q, std::vector<std::shared_ptr<AStarNode>> _dbN, std::vector<int> _order, HNode *_parent)
    : Q(_Q),
      dbN(_dbN),
      M_to(Q.size()),
      parent(_parent),
      depth(parent == nullptr ? 0 : parent->depth + 1),
      order(_order),
      search_tree()
{
  // if (parent != nullptr)
  //   parent->neighbors.insert(this);
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
             std::vector<std::shared_ptr<dynoplan::Heu_fun>> _h_funs,
             std::vector<std::shared_ptr<dynobench::Model_robot>> _robots,
             int _verbose,
             const Deadline *_timelimit)
    : problem(_problem),
      dbNodes(_dbNodes),
      expander(_expander),
      h_funs(_h_funs),
      robots(_robots),
      timelimit(_timelimit),
      verbose(_verbose),
      db_pibt(robots),
      H_goal(nullptr),
      OPEN(),
      order(robots.size(), 0),
      loop_cnt(0)
{
  tmp_traj_wrapper.allocate_size(/*max_traj_size*/ 40, robots.at(0)->nx, robots.at(0)->nu);
}

LaCAM::~LaCAM() {}

MultiRobotTrajectory LaCAM::solve()
{
  solver_info(1, "LaCAM begins");
  // setup search
  std::unordered_map<std::vector<Eigen::VectorXd>, HNode *, ConfigHasher, ConfigEqual> EXPLORED;
  // insert initial node
  order = get_sorted_order(robots, problem.starts, problem.goals);
  auto H_init = new HNode(problem.starts, dbNodes, order);
  OPEN.push_front(H_init);
  EXPLORED[H_init->Q] = H_init;
  // search loop
  solver_info(2, "search iteration begins");
  while (!OPEN.empty() && !is_expired(timelimit))
  {
    ++loop_cnt;
    // do not pop here!
    auto H = OPEN.front(); // high-level node
    // check goal condition
    if (H_goal == nullptr && is_close_config(H->Q, problem.goals, robots.at(0), /*threshold*/ 0.5))
    {
      H_goal = H;
      solver_info(2, "found solution!");
      break;
    }
    // extract constraints
    if (H->search_tree.empty())
    {
      OPEN.pop_front();
      continue;
    }
    auto L = H->search_tree.front();
    H->search_tree.pop();
    // get applicable motions once
    for (size_t r = 0; r < robots.size(); r++)
    {
      get_applicable_trajs(H->dbN[r], rolled_robot_data[r], r);
    }
    // low level search
    if (L->depth < H->Q.size())
    {
      const auto i = H->order[L->depth];
      assert(!rolled_robot_data[i].trajectories.empty());
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
      }
    }
    // create successors at the high-level search
    std::vector<Eigen::VectorXd> Q_to;
    Q_to.resize(robots.size());
    std::vector<std::shared_ptr<AStarNode>> dbN_to;
    dbN_to.resize(robots.size());
    std::vector<dynobench::Trajectory> M_to;
    M_to.resize(robots.size());
    auto res = set_new_config(H, L, Q_to, dbN_to, M_to, rolled_robot_data);
    H->M_to = M_to;
    delete L;
    if (!res)
      continue;
    if (EXPLORED.find(Q_to) != EXPLORED.end())
    {
      std::cout << "Config is already explored!" << std::endl;
      continue;
    }
    // always add the node
    order.clear();
    order = get_sorted_order(robots, Q_to, problem.goals);
    auto H_new = new HNode(Q_to, dbN_to, order, H);
    OPEN.push_front(H_new);
    EXPLORED[H_new->Q] = H_new;
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

    for (const auto &seg : segments)
    {
      for (size_t id = 0; id < seg.size(); ++id)
      {
        auto &traj_out = solution.trajectories[id];
        const auto &seg_traj = seg[id];
        traj_out.states.insert(traj_out.states.end(), seg_traj.states.begin(), seg_traj.states.end());
        traj_out.actions.insert(traj_out.actions.end(), seg_traj.actions.begin(), seg_traj.actions.end());
      }
    }
  }
  return solution;
}

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
    hScore = robots[robot_id]->distance(tmp_state, problem.goals[robot_id]); // h_funs[robot_id]->h(tmp_state); // for the last state of the motion
    double cost_motion = (tmp_traj_wrapper.get_size() - 1) * robots[robot_id]->ref_dt;
    gScore = db_node->gScore + cost_motion;
    tmp_traj_wrapper.last_state_g = gScore;
    tmp_traj_wrapper.last_state_h = hScore;
    tmp_traj_wrapper.last_state_f = gScore + hScore;
    tmp_traj_wrappers.push_back(tmp_traj_wrapper);
  }
  // ii. sort based on f-value
  dynobench::TrajWrapper::SortByLastStateF(tmp_traj_wrappers);
  // ii. rollout trajs - env collision free
  Eigen::VectorXd x0 = db_node->state_eig;
  for (size_t k = 0; k < tmp_traj_wrappers.size(); k++)
  {
    auto &traj_wrap = tmp_traj_wrappers[k];
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
  }
  return db_pibt.set_new_config(H->Q, Q_to, dbN_to, M_to, H->order, robot_data_rolled);
}

std::vector<int> LaCAM::get_sorted_order(
    std::vector<std::shared_ptr<dynobench::Model_robot>> &robots,
    const std::vector<Eigen::VectorXd> states,
    const std::vector<Eigen::VectorXd> goal_states)
{
  std::vector<int> orders(robots.size());
  std::iota(orders.begin(), orders.end(), 0);
  std::sort(orders.begin(), orders.end(), [&](size_t i, size_t j)
            { return robots[i]->distance(states[i], goal_states[i]) >
                     robots[j]->distance(states[j], goal_states[j]); });
  return orders;
}