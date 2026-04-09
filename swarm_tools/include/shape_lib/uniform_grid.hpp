#pragma once

#include <Eigen/Dense>
#include <vector>

namespace shape_lib
{
    enum class CellState : uint8_t {
      Free,
      Occupied,
      Unknown
    };
    struct GridCell {
        Eigen::Vector3d center;
        CellState state;
    };
    struct UniformGrid {
      Eigen::Vector3d min;
      Eigen::Vector3d max;
      double cellSize;

      int sizeX, sizeY, sizeZ;
      std::vector<GridCell> cells;

      inline int index(int x, int y, int z) const {
          return x + sizeX * (y + sizeY * z);
      }

      GridCell& at(int x, int y, int z) {
          return cells[index(x, y, z)];
      }

      const GridCell& at(int x, int y, int z) const {
          return cells[index(x, y, z)];
      }
    };
}