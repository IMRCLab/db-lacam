#include <boost/graph/graphviz.hpp>
#include <boost/heap/d_ary_heap.hpp>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
// OMPL headers
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include "ompl/base/Path.h"
#include "ompl/base/ScopedState.h"
#include <ompl/base/spaces/SE2StateSpace.h>
// dynobench
#include "dynobench/motions.hpp"
#include "dynobench/robot_models.hpp"
#include "dynobench/general_utils.hpp"
#include "dynobench/robot_models_base.hpp"
// boost stuff for the graph
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/undirected_graph.hpp>
#include <boost/property_map/property_map.hpp>
// custom
#include "nigh_custom_spaces.hpp"
// #include "motion.hpp"
#include "db-pibt.hpp"

using dynobench::FMT;
using dynobench::Trajectory;

bool pibt(std::vector<dynobench::TrajWrapper> traj_wrappers,
          dynobench::Model_robot &robot, dynobench::Trajectory &traj_out,
          Eigen::Ref<Eigen::VectorXd> x0)
{
  bool motion_valid;
  Eigen::VectorXd constrained_state(3); // for the unicycle (x,y,theta)
  constrained_state << 1.0, 1.0, 0.;    // hard-coded
  float delta = 0.5;
  bool collision = false;
  // 1. loop over and compute the tmp_traj (only forwars motion)
  for (size_t i = 0; i < traj_wrappers.size(); i++)
  {
    auto &traj_wrap = traj_wrappers[i];
    // 2. apply actions, considering only unicycle
    std::vector<Eigen::VectorXd> us = traj_wrap.get_actions();
    std::vector<Eigen::VectorXd> xs(us.size() + 1,
                                    Eigen::VectorXd::Zero(robot.nx));
    robot.rollout(x0, us, xs);
    dynobench::Trajectory traj;
    // traj from the applicable motion
    traj.start = x0;
    traj.states = xs;
    traj.actions = us;
    traj.goal = traj.states.back();
    // 3. check each for collision checking, no priority at the moment
    // a. traj to motion with enabling computing collision
    Motion motion;
    traj_to_motion(traj, robot, motion, /*compute collision*/ true);
    // b. check for collision
    assert(motion.collision_manager);
    assert(robot.env.get());
    // b.1 check motion-env collision -> DOUBLE CHECK, need to create the
    // collision shape I think
    fcl::DefaultCollisionData<double> collision_data;
    motion.collision_manager->collide(robot.env.get(), &collision_data,
                                      fcl::DefaultCollisionFunction<double>);
    if (collision_data.result.isCollision())
      continue; // if motion collides with env
    // b.2 check for motion-moiton collision (state-to-state) - can be merged
    // AABB b.3 check for the distance
    for (const auto state : traj.states)
    {
      if (robot.distance(state, constrained_state) <= delta)
      {
        std::cout << "collision motion-state! Moving to the next motion"
                  << std::endl;
        collision = true;
        break; // break this loop, and continue with the outer loop
      }
    }
    if (collision)
      continue;
    else
    {
      x0 = traj.states.back();
      // extract out the motion to the solution. Could be changed after, used to
      // visualize
      traj_out.states.insert(traj_out.states.end(), traj.states.begin(),
                             traj.states.end());
      traj_out.actions.insert(traj_out.actions.end(), traj.actions.begin(),
                              traj.actions.end());
      return true;
    }
  }
  return false;
}
