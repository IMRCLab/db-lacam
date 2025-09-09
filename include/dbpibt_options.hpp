#pragma once

#include <string>
#include <vector>
#include <boost/program_options.hpp>                             

struct Planner_options
{

  float delta = .5;          // discontinuity bound
  float goal_delta = .5;     // pibt needs it
  int cost_delta_factor = 0; // take into account the cost for discountinuity
  float alpha =
      .5; // How discontinuity bound is shared between expansion and reaching
  size_t max_motions = 1e4;
  std::string motionsFile = "";
  std::string outFile =
      "/tmp/dynoplan/out_db.yaml"; // output file to write some results
  float maxCost =
      std::numeric_limits<float>::infinity();           // Cost bound during search
  std::vector<dynoplan::Motion> *motions_ptr = nullptr; // Pointer to loaded motions
  size_t max_expands = 5 * 1e3;                         // 1e6
  bool debug = false;
  int limit_branching_factor =
      20;                        // Limit on branching factor to encourage more deep search
  double search_timelimit = 1e4; // in ms
  bool rewire = true;            // to allow rewiring during the search
  double cluster_range = 0.05;   // range to compute the threshold for motion clustering based on h-value
  size_t cluster_n = 8;          // number of elements per cluster to return
  bool merged_aabb = false;      // some problems don't work with merged aabb
  bool refine_solution = false;  // refined the solution using 1) Large Neighborhood Search

  void print() const
  {
    std::cout << "*** options for the planner ***" << std::endl;
    std::cout << "  delta: " << delta << "\n";
    std::cout << "  cost_delta_factor: " << cost_delta_factor << "\n";
    std::cout << "  goal_delta: " << goal_delta << "\n";
    std::cout << "  alpha: " << alpha << "\n";
    std::cout << "  max_motions: " << max_motions << "\n";
    std::cout << "  cluster_range: " << cluster_range << "\n";
    std::cout << "  cluster_n: " << cluster_n << "\n";
    std::cout << "  merged aabb: " << merged_aabb << "\n";
    std::cout << "***" << std::endl;
  }
};