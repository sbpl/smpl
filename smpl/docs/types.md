

clutter
---
* `Coord` - A vector of ints representing a coordinate in the cluttered environment.
* `State` - A vector of doubles representing a state in the cluttered environment.

* `LatticeState`: coord, state and t (time)

* `Trajectory`: a vector of `LatticeState`
* `STATES`: a vector of `LaticeState` **pointers**

* <u>Equal operator</u> for two `LatticeState`objects: checks if the coordinates and **t** are equal.
