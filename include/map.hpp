#pragma once

#include <vector>
#include <utility>

class OccupancyMap
{
public:
  OccupancyMap(double width_m, double height_m, double grid_resolution, int nil_val = -1);

  void set_occupied(int x_idx, int y_idx, int robot_id);
  int get_cell(int x_idx, int y_idx) const;
  void print_map() const;
  void reset();

private:
  int rows, cols;
  double resolution;
  int NIL;
  std::vector<std::vector<int>> grid;

  bool in_bounds(int x, int y) const;
};
