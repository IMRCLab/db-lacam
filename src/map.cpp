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
    return grid[x_idx][y_idx];
  }
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

// int main()
// {
//   double width = 2.0;  // meters
//   double height = 2.0; // meters
//   double grid_size = 0.5;

//   OccupancyMap map(width, height, grid_size);
//   auto [x_idx, y_idx] = map.world_to_grid(0.0, 0.5);
//   std::cout << x_idx << ", " << y_idx << std::endl;
//   map.set_occupied(x_idx, y_idx, 2); // Robot ID
//   map.print_map();
// }