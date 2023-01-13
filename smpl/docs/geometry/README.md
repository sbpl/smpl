Geometry
---

Geometry functionality is provided by the `geometry` module. The module provides a number of classes for representing geometric objects, such as boxes, lines, triangles, planes, polygons and more. 
The module also provides a number of functions for performing geometric operations, such as computing the distance between two points, or the intersection of two triangles.

The module is organized into the following main submodules:
* `mesh_utils`: Utilities for working and creating meshes. 
* `voxelize`: Utilities for voxelizing meshes. There are, ofcourse, functions for creating directly a voxel object without declaring first a mesh. The function itself will create a mesh and the voxelize it. 
There are other functions for working with voxel objects - `Distance`, `ComputeAxisAlignedBoundingBox`, `IsInDiscreteBoundingBox`, `ScanFill`, `TransformVertices`, `CompareX`, `CompareY`, `CompareZ`.
* `intersect`: Utilities for computing intersections between geometric objects.
* `bounding_spheres`:  Compounds the bounding shperes of geometries and covers these bounds with spheres.

---

## Usage:
### TDOO: Add usage examples