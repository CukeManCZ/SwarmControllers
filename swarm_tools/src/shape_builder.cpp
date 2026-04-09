#include "shape_lib/shape_builder.hpp"

namespace shape_lib
{
    void ComputeMapBounds(const octomap::OcTree& tree, Eigen::Vector3d& min,Eigen::Vector3d& max)
    {
        bool first = true;

        for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
            if (!tree.isNodeOccupied(*it)) continue;

            Eigen::Vector3d c(it.getX(), it.getY(), it.getZ());
            double h = it.getSize() * 0.5;

            Eigen::Vector3d vmin = c.array() - h;
            Eigen::Vector3d vmax = c.array() + h;

            if (first) {
                min = vmin;
                max = vmax;
                first = false;
            } else {
                min = min.cwiseMin(vmin);
                max = max.cwiseMax(vmax);
            }
        }
    }

    UniformGrid ShapeBuilder::BuildGrid(const octomap::OcTree& tree, double cellSize)
    {
        UniformGrid grid;
        grid.cellSize = cellSize;

        ComputeMapBounds(tree, grid.min, grid.max);

        // Pad by one cell to avoid boundary issues
        grid.min -= Eigen::Vector3d::Constant(cellSize);
        grid.max += Eigen::Vector3d::Constant(cellSize);

        Eigen::Vector3d dims = grid.max - grid.min;
        grid.sizeX = static_cast<int>(std::ceil(dims.x() / cellSize));
        grid.sizeY = static_cast<int>(std::ceil(dims.y() / cellSize));
        grid.sizeZ = static_cast<int>(std::ceil(dims.z() / cellSize));
        grid.cells.resize(grid.sizeX * grid.sizeY * grid.sizeZ);

        // Initialize grid
        for (int z = 0; z < grid.sizeZ; ++z)
        for (int y = 0; y < grid.sizeY; ++y)
        for (int x = 0; x < grid.sizeX; ++x)
        {
            GridCell &c = grid.at(x,y,z);
            c.state = CellState::Free;
            c.center = grid.min + Eigen::Vector3d(
                (x + 0.5) * cellSize,
                (y + 0.5) * cellSize,
                (z + 0.5) * cellSize);
        }

        // Iterate all nodes (not just leaves)
        for (auto it = tree.begin(); it != tree.end(); ++it)
        {
            if (!tree.isNodeOccupied(*it)) continue;

            Eigen::Vector3d center(it.getX(), it.getY(), it.getZ());
            double voxelSize = it.getSize();

            // Compute voxel bounds
            Eigen::Vector3d vmin = center.array() - voxelSize*0.5;
            Eigen::Vector3d vmax = center.array() + voxelSize*0.5;

            // Compute overlapping grid cell indices
            int startX = static_cast<int>(std::floor((vmin.x() - grid.min.x()) / cellSize));
            int startY = static_cast<int>(std::floor((vmin.y() - grid.min.y()) / cellSize));
            int startZ = static_cast<int>(std::floor((vmin.z() - grid.min.z()) / cellSize));

            int endX = static_cast<int>(std::ceil((vmax.x() - grid.min.x()) / cellSize)) - 1;
            int endY = static_cast<int>(std::ceil((vmax.y() - grid.min.y()) / cellSize)) - 1;
            int endZ = static_cast<int>(std::ceil((vmax.z() - grid.min.z()) / cellSize)) - 1;

            // Clamp indices
            startX = std::max(0, startX); startY = std::max(0, startY); startZ = std::max(0, startZ);
            endX = std::min(grid.sizeX-1, endX); 
            endY = std::min(grid.sizeY-1, endY); 
            endZ = std::min(grid.sizeZ-1, endZ);

            // Fill all cells intersecting voxel
            for (int z = startZ; z <= endZ; ++z)
            for (int y = startY; y <= endY; ++y)
            for (int x = startX; x <= endX; ++x)
            {
                grid.at(x,y,z).state = CellState::Occupied;
            }
        }

        return grid;
    }

    bool ShapeBuilder::IsSurfaceCell(const UniformGrid& grid, int x, int y, int z)
    {
        if (grid.at(x, y, z).state != CellState::Occupied)
            return false;
    
        static const int dirs[6][3] = {
            { 1, 0, 0}, {-1, 0, 0},
            { 0, 1, 0}, { 0,-1, 0},
            { 0, 0, 1}, { 0, 0,-1}
        };

        for (const auto& d : dirs) {
            int nx = x + d[0];
            int ny = y + d[1];
            int nz = z + d[2];

            if (nx < 0 || ny < 0 || nz < 0 ||
                nx >= grid.sizeX ||
                ny >= grid.sizeY ||
                nz >= grid.sizeZ)
                return true; // boundary → surface

            CellState nstate = grid.at(nx, ny, nz).state;
            if (nstate != CellState::Occupied)
                return true;
        }

        return false;
    }

    std::vector<ShapeNode> ShapeBuilder::ExtractShape(const UniformGrid& grid)
    {
        std::vector<ShapeNode> nodes;

        for (int z = 0; z < grid.sizeZ; ++z)
        for (int y = 0; y < grid.sizeY; ++y)
        for (int x = 0; x < grid.sizeX; ++x)
        {
            if (!IsSurfaceCell(grid, x, y, z))
            continue;

            const GridCell& cell = grid.at(x, y, z);

            ShapeNode node;
            node.position = cell.center;
            node.size     = grid.cellSize;
            node.value    = 1; // or neighbor count, curvature, etc.

            nodes.push_back(node);
        }

        return nodes;
    }
} // namespace shape_lib
