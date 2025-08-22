#include <vector>
#include <iostream>
#include <iomanip>
#include "map.hpp"

OccupancyMap::OccupancyMap(double width_m, double height_m, double grid_resolution, int nil_val)
    : resolution(grid_resolution), NIL(nil_val)
{

  rows = static_cast<int>(height_m / resolution);
  cols = static_cast<int>(width_m / resolution);
  // Initialize the grid
  grid = std::vector<std::vector<int>>(rows, std::vector<int>(cols, NIL));
  // std::cout << "rows, cols: " << rows << ", " << cols << std::endl;
}

void OccupancyMap::set_occupied(int x_idx, int y_idx, int robot_id)
{
  if (in_bounds(x_idx, y_idx))
  {
    grid[x_idx][y_idx] = robot_id;
  }
}

int OccupancyMap::get_cell(int x_idx, int y_idx) const
{
  if (in_bounds(x_idx, y_idx))
  {
    // std::cout << "x, y: " << x_idx << ", " << y_idx << std::endl;
    return grid[x_idx][y_idx];
  }
  // std::cout << "outside bounds" << std::endl;
  return NIL; // or throw exception
}

void OccupancyMap::print_map() const
{
  for (const auto &row : grid)
  {
    for (auto cell : row)
    {
      std::cout << std::setw(3) << cell << " ";
    }
    std::cout << "\n";
  }
}

void OccupancyMap::reset()
{
  for (auto &row : grid)
  {
    std::fill(row.begin(), row.end(), NIL);
  }
}

bool OccupancyMap::in_bounds(int x, int y) const
{
  return x >= 0 && x < rows && y >= 0 && y < cols;
}