Graph
---

Graph is a library for creating and manipulating graphs.

---

* [`goal_constraint.h`](../../include/smpl/graph/goal_constraint.h) - Goal constraint for graph search. 
##### Namespace: `smpl`

Contains a goal state and a goal tolerance. Can be defined in joint state space or workspace.

---

* [`robot_planning_space`](../../src/graph/robot_planning_space.cpp) 
##### Namespace: `smpl`

Class: `RobotPlanningSpace`
Inherits from `sbpl::discrete_space_information::DiscreteSpaceInformation` (base class for environments defining planning graphs) 
and `extensions`.

As we will soon see, there are many spaces one can use. Thus, each of them will have its own _PlanningSpace_ class which wil inherit from this class.

There are many methods in this class:
* `init` - initializes the planning space. 
* `setStart`, `setGoal` - sets the start(_RobotState_)/goal(_GoalConstraint_) state of the robot.
* `startState`, `goal` - returns the start/goal state of the robot.
* `GetLazySuccs` - returns the successors (lazy) of a given state.
* `GetTrueCost` - returns the true cost of a given two states.
* Heuristic related utilities: 
  * `insertHeuristic` - inserts a heuristic into the planning space.
  * `eraseHeuristic` - erases a heuristic from the planning space.
  * `hasHeuristic` - checks if a heuristic is in the planning space.
  * `GetGoalHeuristic(int state_id)` - returns the heuristic to the goal with the given id state.
  * `GetStartHeuristic(int state_id)` - returns the heuristic to the start with the given id state.
  * `GetFromToHeuristic(int from_id, int to_id)` - returns the heuristic from the state with id `from_id` to the state with id `to_id`.

Additionally, there are **_pure virtual methods_** which are implemented in the derived classes:
* `getStartStateID`, `getGoalStateID` - returns the id of the start/goal state.
* `extractPath` - extracts the path from the start to the goal state.
* `GetSuccs` - returns the successors of a given state.
* `GetPreds` - returns the predecessors of a given state.
* `PrintState` - prints the state with the given id.

---

* [`action_space`](../../include/smpl/graph/action_space.h)
##### Namespace: `smpl`
Interface abstract class: `ActionSpace` - action space for planning graphs.
5 methods:
1. `planningSpace()` - returns the RobotPlanningSpace object.
2. `init()` - initializes the action space.
3. `updateStart()`, `updateGoal()` - updates the start/goal state of the action space.
4. `apply()` - Return the set of actions available from a state.
Each action consists of a sequence of waypoints from the source state describing the approximate motion the robot 
will take to reach a successor state. The sequence of waypoints need not contain the source state. 
The motion between waypoints will be checked via the set CollisionChecker's isStateToStateValid function during a search.

---

* [`workspace_lattice_types.h`](../../include/smpl/graph/workspace_lattice_types.h)
Defines three **important** aliases:
* `WorkspaceState` - a workspace state is a vector of doubles.
* `WorkspaceCoord` - a workspace coordinate is a vector of integers.
* `WorkspaceAction` - a workspace action is a vector of workspace states.

And an important struct:
* `WorkspaceLatticeState` - a workspace lattice state contains two attributes of a RobotState and a WorkspaceCoord.

---

* [`workspace_lattice_base`](../../include/smpl/graph/workspace_lattice_base.h):
Base class for graph representations that represent states via a 1:1 mapping from joint space states to states in SE(3) alongside an array of redundant variables used to uniquely determine the state. This class is responsible for handling those transformations and discretization of the resulting SE(3) + space.
##### Namespace: `smpl`
Contains a class `WorkspaceLatticeBase` which inherits from `RobotPlanningSpace`. This class instantiate FK, IK, Redundant interfaces, and more.
This is one of the most important classes in the graph library. It contains many methods for
conversions between robot states, workspace states, and workspace coords, 
and conversions from discrete coordinates to continuous states.

