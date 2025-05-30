#pragma once

#include <yaml-cpp/node/node.h>
// dynobench
#include "dynobench/dyno_macros.hpp"
#include "dynobench/motions.hpp"
#include "dynobench/robot_models.hpp"
// ompl
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include "ompl/control/StatePropagator.h"
// FCL
#include "fclHelper.hpp"
#include <fcl/fcl.h>

class Motion
{

  using Trajectory = dynobench::Trajectory;

public:
  std::vector<ompl::base::State *> states;
  std::vector<ompl::control::Control *> actions;
  Trajectory traj;

  std::shared_ptr<ShiftableDynamicAABBTreeCollisionManager<double>>
      collision_manager;
  std::vector<std::unique_ptr<fcl::CollisionObjectd>> collision_objects;

  double cost;
  size_t idx;
  bool disabled = false;
  double get_cost() const { return cost; }

  const Eigen::VectorXd &getStateEig() const { return traj.states.front(); }
  const Eigen::VectorXd &getLastStateEig() const { return traj.states.back(); }
  const Eigen::VectorXd
  getLastStateEigCanonical(size_t translation_invariance) const
  {
    Eigen::VectorXd m = traj.states.back();
    m.head(translation_invariance).setZero();
    return m;
  }
};

enum class MotionPrimitiveFormat
{
  BOOST,
  YAML,
  JSON,
  MSGPACK,
  AUTO
};

void load_motion_primitives_new(
    const std::string &motionsFile, dynobench::Model_robot &robot,
    std::vector<Motion> &motions, int max_motions, bool cut_actions,
    bool shuffle, bool compute_col = true,
    MotionPrimitiveFormat format = MotionPrimitiveFormat::AUTO);

void traj_to_motion(const dynobench::Trajectory &traj,
                    dynobench::Model_robot &robot, Motion &motion_out,
                    bool compute_col);

void compute_col_shape(Motion &m, dynobench::Model_robot &robot);
