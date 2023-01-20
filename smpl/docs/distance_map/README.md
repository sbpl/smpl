Distance Map
---

* [`distance_map_inteface.h`](../../include/smpl/distance_map/distance_map_interface.h) - Interface for distance map implementations.
##### Namespace: `smpl`
`DistanceMapInterface`: Abstract base class for distance map implementations. This class specifies methods for returning distance to the nearest occupied cells, both in **cell** units and in **metric** units.
The distances computed for cells may differ from those computed for points within the same cell. This distinction is made by the two functions, `getCellDistance()` and `getMetricDistance()`, and similarly for their squared variants. The `getCellDistance()` function return the distance from the center of the query cell to the center of its nearest obstacle cell. This distance is valid for the cell center but may be an approximation of the distance from another point, even within the same cell, to its nearest obstacle cell. The function `getMetricDistance()` may provide more accurate distances.

This class declares multiple methods:
1. **Modifiers**: Updating a map with new information
    - `addPointsToMap`
    - `removePointsFromMap`
    - `updatePointsInMap`
    - `reset`
2. **Properties**: Properties of the map
    - `originX`, `originY`, `originZ`
    - `sizeX`, `sizeY`, `sizeZ`
    - `resolution`
    - `numCellsX`, `numCellsY`, `numCellsZ`
    - `getUninitializedDistance` - TODO: What is this?
3. **Distance Lookups**: Distance utils for points and cells (to nearest obstacle)
    - `getCellDistance` - Distance between cell centers (doesn't consider inner cell distance).
    - `getMetricDistance` - May be more accurate than `getCellDistance`
    - `getCellSquaredDistance`
    - `getMetricSquaredDistance`
4. **Conversions**: Conversions between metric and cell units
    - `gridToWorld`
    - `worldToGrid`
    - `isCellValid` - **IMPORTANT**

From this class, we have the following derived classes:

---
* [`sparse_distance_map.h`](../../include/smpl/distance_map/sparse_distance_map.h) - Sparse distance map implementation.


* [`distance_map`](../../include/smpl/distance_map/detail/distance_map.hpp) - 
An unsigned distance transform implementation that computes distance values incrementally up to a maximum threshold. 
The distance of each cell is determined as the closer of the distance from its nearest obstacle cell and the distance 
to the nearest border cell. Distances for cells outside the specified bounding volume will be reported as 0. 
Border cells are defined as imaginary cells that are adjacent to valid in-bounds cells but are not themselves within 
the bounding volume. The basis for this class can be found in '_Sebastian Scherer, David Ferguson, and Sanjiv Singh, 
"Efficient C-Space and Cost Function Updates in 3D for Unmanned Aerial Vehicles," Proceedings International Conference 
on Robotics and Automation, May, 2009.'_, albeit with some implementation differences. Notably, it is preferred to 
compute the distance updates for simultaneous obstacle insertion and removal in a two-phase fashion rather than 
interleaving the updates for the insertion and removal of obstacles. While the latter should complete in fewer 
propagation iterations, it is not usually worth the additional overhead to maintain the priority queue correctly 
in this domain. The base class `DistanceMapBase` completes implements functionality independent of the distance 
function that is used to update cells. It is not intended to be used directly. `DistanceMap` implements functionality 
that is dependent on the distance function used. The class passed through as the template parameter specifies the 
distance function used to compute the distance updates from neighboring cells by implementing a member function with 
the following signature: `int distance(const Cell& s, const Cell& n);` that returns the new distance for cell s, being 
updated from adjacent cell n. If the distance function is made private (and it likely should be, since the Cell struct 
is private to DistanceMapBase), the derived class must declare DistanceMap as a friend.

From this class (`DistanceMap`), we have the following derived classes, which implement the _distance_ function:
* [`euclid_distance_map.h`](../../include/smpl/distance_map/euclid_distance_map.h) - `EuclidDistanceMap`
* [`edge_euclid_distance_map.h`](../../include/smpl/distance_map/edge_euclid_distance_map.h) - `EdgeEuclidDistanceMap`
* [`chessboard_distance_map.h`](../../include/smpl/distance_map/chessboard_distance_map.h) - `ChessboardDistanceMap`

---

## Usage:

Defining a distance map:
```cpp
auto df_size_x = 3.0;
auto df_size_y = 3.0;
auto df_size_z = 3.0;
auto df_res = 0.02;
auto df_origin_x = -0.75;
auto df_origin_y = -1.5;
auto df_origin_z = 0.0;
auto max_distance = 1.8;

using DistanceMapType = smpl::EuclidDistanceMap;

auto df = std::make_shared<DistanceMapType>(
      df_origin_x, df_origin_y, df_origin_z,
      df_size_x, df_size_y, df_size_z,
      df_res,
      max_distance);
```
Now, we can use it for constructing an OccupancyGrid:
```cpp
auto ref_counted = false;
smpl::OccupancyGrid grid(df, ref_counted);

grid.setReferenceFrame(planning_frame);
SV_SHOW_INFO(grid.getBoundingBoxVisualization());
```