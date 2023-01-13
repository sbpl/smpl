# SMPL

---
### A library of generic graph representations, heuristics, search algorithms, and many utilities for robotic motion planning.

---

#### Important dependencies:
* [Boost](http://www.boost.org/)
* [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page)
* [SBPL](https://github.com/sbpl/sbpl)

---
### Header-only files:

---
* [`smpl/types.h`](docs/types.md) - Define types used throughout the library.
##### Namespace: `smpl` 
In this header file, there are few **hash** structs and **operators** overloading.
For example, the `PointerValueHash` helper struct to compute a hash value for a pointer using the hash value of the object it points to. 
The `PointerValueEqual` helper struct to compare two pointers by their value using the **overloaded equality operator**.

There are also two important *typedefs* here:
* `RobotState` - A vector of doubles representing a robot configuration.
**Note**: future work is to replace it with a `RobotState` class that provides a more robust interface to robot state information.
* `Action` - A vector of doubles representing an action.

##### Namespace: `clutter`

Container of *typedefs*, *data structures* and *operator overloading* for the cluttered environment representation.

---

* [`smpl/time.h`](docs/time.md) - Defines a simple time class.
##### Namespace: `smpl`

There are multiple *classes* for clocks interfaces and multiple *typedefs* to declare the selected clock type.
Additionally, there are two useful functions:
* `to_secondes` - converts a `Duration` to seconds.
* `to_duration` - converts a `double` to a `Duration`.

---
* [`smpl/angles.h`](docs/angles.md) - functions for working with angles.
##### Namespace: `smpl::angles`
Here, we have a bunch of utilities such as `to_degrees`, `to_radians`, `shortest_angle_diff`, `get_euler_zyx`, `get_nearest_planar_rotation`
and many many more.

---
* [`smpl/spatial.h`](docs/spatial.md) - functions for working with spatial transformations.
##### Namespace: `smpl`

Aliases for the most commonly used types from within Eigen to support brevity and, optimistically, ease a possible transition to optional compilation using 32-bit floating-point across the board rather than templating the entire library 
We have here two main types:
* **Matrix Types** such as Vector3, RowVector3, Matrix3, Matrix4, etc.
* **Transform Types** such as Isometry3, Translation3, etc.

Additionally, we have functions for making these types from coordinates - `make_affine2`, `make_affine`.

**IMPORTANT NOTE**: If you are using ubuntu distribution less than 20.04 (18.04), then you work with *Affines*. Otherwise, 
you are using *Isometries* - even though the function called "make_Affine" they return Isometry object.

---
* [`smpl/shapes.h`](docs/shapes.md) - working with shapes for collision checker and more.
##### Namespace: `smpl::collision`

* Enumeration `ShapeType`: Sphere, Cylinder, Cone, Box, Plane, Mesh, Octree.

Here, we have shape data structures. 

---
* [`smpl/extension.h`](docs/extension.md) - // TODO.

### With source files:

---
* [`robot_model`](docs/robot_model.md) - defines the interface for a robot model.
##### Namespace: `smpl`

Here we have 5 key classes:
1. `RobotModel`: The root interface defining the basic requirements for a robot model 
2. `InverseKinematicsInterface`: RobotModel extension for providing inverse kinematics 
3. `ForwardKinematicInterface`: RobotModel extension for providing forward kinematics 
4. `RedundantManipulatorInterface`: RobotModel extension for redundancy utils 
5. `RobotModelChild`: Convenience class allowing a component to implement all root interface methods via an existing extension 

---
* [`post_processing`](docs/post_processing.md) - Package for post-processing motion plans such as *shortcutting paths*, 
*interpolating paths*, *extracting position paths* and more.
##### Namespace: `smpl`

---
* [`planning_params`]() - Parameters wrapper for planning algorithms.
##### Namespace: `smpl`

Two main structures:
* `PlanningParams`: class for storing planning parameters and methods for adding, getting and checking for params
* `ParameterException`: exception struct for parameter errors

---
* [`occupancy_grid`](docs/occupancy_grid.md) - Occupancy grid representation. 
Lightweight wrapper around [DistanceMapInterface](ADD!!!) (explained later) with some additions
##### Namespace: `smpl`

The first additional feature is the presence of a reference frame id that is attached to the distance map. The reference 
frame is used **solely for preparing visualizations** and does there are no active transformations managed by this class. 
The second additional feature is cell reference counting. Each cell may count the number of times it has been inserted 
into the distance map. Only when a cell has been removed the same number of times it has been inserted, 
will it be removed from the distance map. The cell is only actually inserted once into the distance map, regardless of 
the number of consecutive insertions into the occupancy grid. If cell reference counting is enabled with a distance 
field passed to the OccupancyGrid on construction, the caller must not directly modify the distance map. 
This may corrupt the invariant that the obstacle exists in the distance map if its reference count is non-zero. 
An arbitrary distance map implementation may be used with this class. If none is specified, by calling the verbose 
constructor, an instance of `smpl::EuclidDistanceMap` is constructed.

### TODO

---
* [`constants`]() - Declaring constants for the library. 
##### Namespace: `clutter` 

There are simple constants such as _bools_ and _integers_. Additionally, there are more consts such as
**YCB_OBJECTS** , their names. their dims and more.

---
* [`collision_checker`](docs/collision_checker.md) - Collision checker interface.
##### Namespace: `smpl`

Classes:
* `CollisionChecker`: **Abstract** class for collision checking. Contain methods for 
checking validity of states, interpolations, getting collision models, inserting collision objects and more.
(Mostly all are _virtual_)

* `CollisionDistanceExtension` - Interface for collision distance queries.