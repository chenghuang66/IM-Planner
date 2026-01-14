#pragma once

#include <limits>
#include <vector>
#include "point.hpp"

// line traversal
void lineTraversal(const Point &start, const Point &goal, const double resolution, std::vector<IntPoint> &visitedGrids)
{
    IntPoint s_grid = {static_cast<int>(std::floor(start.x / resolution)), static_cast<int>(std::floor(start.y / resolution))};
    IntPoint g_grid = {static_cast<int>(std::floor(goal.x / resolution)), static_cast<int>(std::floor(goal.y / resolution))};
    Point vector = {goal.x - start.x, goal.y - start.y};
    double stepX = (vector.x > 0) ? 1 : -1;
    double stepY = (vector.y > 0) ? 1 : -1;
    double next_grid_boundary_x = (s_grid.x + stepX) * resolution;
    double next_grid_boundary_y = (s_grid.y + stepY) * resolution;
    double tMaxX = (vector.x != 0) ? (next_grid_boundary_x - start.x) / vector.x : std::numeric_limits<double>::max();
    double tMaxY = (vector.y != 0) ? (next_grid_boundary_y - start.y) / vector.y : std::numeric_limits<double>::max();
    double tDeltaX = (vector.x != 0) ? resolution / vector.x * stepX : std::numeric_limits<double>::max();
    double tDeltaY = (vector.y != 0) ? resolution / vector.y * stepY : std::numeric_limits<double>::max();
    IntPoint diff = {0, 0};
    IntPoint c_grid = {s_grid.x, s_grid.y};
    visitedGrids.push_back(c_grid);
    bool negative = false;
    if (s_grid.x != g_grid.x && vector.x < 0)
    {
        diff.x--, negative = true;
    }
    if (s_grid.y != g_grid.y && vector.y < 0)
    {
        diff.y--, negative = true;
    }
    if (negative)
    {
        c_grid.x += diff.x;
        c_grid.y += diff.y;
        visitedGrids.push_back(c_grid);
    }
    double tx = tMaxX;
    double ty = tMaxY;
    while (!(c_grid == g_grid))
    {
        if (tx < ty)
        {
            c_grid.x += stepX;
            tx += tDeltaX;
        }
        else
        {
            c_grid.y += stepY;
            ty += tDeltaY;
        }
        visitedGrids.push_back(c_grid);
    }
}