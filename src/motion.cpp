#include <memory>
#include <limits>
// nigh
#include "nigh/kdtree_batch.hpp"
#include "nigh/kdtree_median.hpp"
#include "nigh/lp_space.hpp"
#include "nigh/so3_space.hpp"
#include <nigh/cartesian_space.hpp>
#include <nigh/scaled_space.hpp>
// OMPL
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/tools/config/MagicConstants.h>
// dynobench
#include "dynobench/robot_models_base.hpp"
#include "dynobench/dyno_macros.hpp"
// custom
#include "fclHelper.hpp"
#include "motion.hpp"

void load_motion_primitives_new(const std::string &motionsFile,
                                dynobench::Model_robot &robot,
                                std::vector<Motion> &motions, int max_motions,
                                bool cut_actions, bool shuffle,
                                bool compute_col,
                                MotionPrimitiveFormat format)
{

  dynobench::Trajectories trajs;

  if (format == MotionPrimitiveFormat::AUTO)
  {
    std::filesystem::path filePath = motionsFile;
    if (filePath.extension() == ".yaml")
    {
      format = MotionPrimitiveFormat::YAML;
    }
    else if (filePath.extension() == ".json")
      format = MotionPrimitiveFormat::JSON;
    else if (filePath.extension() == ".msgpack")
      format = MotionPrimitiveFormat::MSGPACK;
    else if (filePath.extension() == ".bin")
      format = MotionPrimitiveFormat::BOOST;
  }

  switch (format)
  {
  case MotionPrimitiveFormat::YAML:
  {
    trajs.load_file_yaml(motionsFile.c_str());
  }
  break;

  case MotionPrimitiveFormat::BOOST:
  {
    trajs.load_file_boost(motionsFile.c_str());
  }
  break;

  case MotionPrimitiveFormat::JSON:
  {
    trajs.load_file_json(motionsFile.c_str());
  }
  break;

  case MotionPrimitiveFormat::MSGPACK:
  {
    trajs.load_file_msgpack(motionsFile.c_str());
  }
  break;
  case MotionPrimitiveFormat::AUTO:
  {
    ERROR_WITH_INFO(
        "Incompatible format for motion primitives: should not be here!");
  }
  }

  if (max_motions < trajs.data.size())
    trajs.data.resize(max_motions);

  std::cout << "trajs " << std::endl;
  std::cout << "first state is " << std::endl;
  CSTR_V(trajs.data.front().states.front());

  motions.resize(trajs.data.size());

  bool add_noise_first_state = true;
  CSTR_(add_noise_first_state);

  if (add_noise_first_state)
  {
    std::cout << "WARNING:"
              << "adding noise to first and last state" << std::endl;
    const double noise = 1e-7;
    for (auto &t : trajs.data)
    {
      t.states.front() +=
          noise * Eigen::VectorXd::Random(t.states.front().size());

      t.states.back() +=
          noise * Eigen::VectorXd::Random(t.states.back().size());
    }
  }

  if (startsWith(robot.name, "quad3d"))
  {
    // ensure quaternion
    for (auto &t : trajs.data)
    {
      for (auto &s : t.states)
      {
        s.segment<4>(3).normalize();
      }
    }
  }

  CSTR_(trajs.data.size());
  std::cout << "from boost to motion " << std::endl;

  std::transform(trajs.data.begin(), trajs.data.end(), motions.begin(),
                 [&](const auto &traj)
                 {
                   // traj.to_yaml_format(std::cout, "");
                   Motion m;
                   traj_to_motion(traj, robot, m, compute_col);
                   return m;
                 });

  std::cout << "from boost to motion -- DONE " << std::endl;

  CHECK(motions.size(), AT);
  if (motions.front().cost > 1e5)
  {
    std::cout << "WARNING: motions have infinite cost." << std::endl;
    std::cout << "-- using default cost: TIME" << std::endl;
    for (auto &m : motions)
    {
      m.cost = robot.ref_dt * m.actions.size();
    }
  }

  if (cut_actions)
  {
    NOT_IMPLEMENTED;
  }

  if (shuffle)
  {
    std::random_device rd;
    std::default_random_engine eng(rd()); // Seed with random value
    std::shuffle(std::begin(motions), std::end(motions), eng);
  }

  for (size_t idx = 0; idx < motions.size(); ++idx)
  {
    motions[idx].idx = idx;
  }
}

void traj_to_motion(const dynobench::Trajectory &traj,
                    dynobench::Model_robot &robot, Motion &motion_out,
                    bool compute_col)
{

  motion_out.states.resize(traj.states.size());
  motion_out.traj = traj;

  motion_out.actions.resize(traj.actions.size());

  if (compute_col)
    compute_col_shape(motion_out, robot);
  motion_out.cost = traj.cost;
}

void compute_col_shape(Motion &m, dynobench::Model_robot &robot)
{
  for (auto &x : m.traj.states)
  {

    auto &ts_data = robot.ts_data;
    auto &col_geo = robot.collision_geometries;
    robot.transformation_collision_geometries(x, ts_data);

    for (size_t i = 0; i < ts_data.size(); i++)
    {
      auto &transform = ts_data.at(i);
      auto co = std::make_unique<fcl::CollisionObjectd>(col_geo.at(i));
      co->setTranslation(transform.translation());
      co->setRotation(transform.rotation());
      co->computeAABB();
      m.collision_objects.push_back(std::move(co));
    }
  }

  std::vector<fcl::CollisionObjectd *> cols_ptrs(m.collision_objects.size());
  std::transform(m.collision_objects.begin(), m.collision_objects.end(),
                 cols_ptrs.begin(), [](auto &ptr)
                 { return ptr.get(); });

  m.collision_manager.reset(
      new ShiftableDynamicAABBTreeCollisionManager<double>());
  m.collision_manager->registerObjects(cols_ptrs);
};
