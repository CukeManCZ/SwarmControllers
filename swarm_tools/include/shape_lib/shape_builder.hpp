#pragma once

#include "shape_lib/uniform_grid.hpp"
#include "shape_lib/shape_node.hpp"

#include <octomap/OcTree.h>
#include <vector>

namespace shape_lib
{
    class ShapeBuilder
    {
        public:
        UniformGrid BuildGrid(const octomap::OcTree& tree, double cellSize);
        bool IsSurfaceCell(const UniformGrid& grid, int x, int y, int z);
        std::vector<ShapeNode> ExtractShape(const UniformGrid& grid);
    };
}