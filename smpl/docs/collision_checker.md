Collision Checker Interface
---

Classes:
* `CollisionChecker`: Contains the following methods (check for overriding):
    * `bool isStateValid(const RobotState& state)`: Check if a state is valid. 
    * `bool isStateToStateValid(const RobotState& start, const RobotState& finish)`: Check if a state to state interpolation is valid. Note: Doesn't include endpoints.
    * `bool interpolatePath(const RobotState& start, const RobotState& finish, std::vector<RobotState>& path)`: Linearly interpolate a path between two states.
    * `std::vector\<visual::Marker> getCollisionModelVisualization(const RobotState& state)`: Get a visualization of the collision model. Returns a vector of markers.
    * `std::vector\<visual::Marker> getCollisionWorldVisualization()`: Get a visualization of the collision world model. Returns a vector of markers.
  The following methods defined in collision_space and are used in manip_lattice:
    * `auto getReferenceFrame() const -> const std::string&`: Get the reference frame of the collision model.
    * `bool insertObject(const CollisionObject& object)`: Insert a collision object into the collision model.
    * `bool removeObject(const CollisionObject& object)`: Remove a collision object from the collision model.
    * `smpl::collision::CollisionObject* findCollisionObject(const std::string& id)`: Find a collision object by id.

    Attributes:
  * `m_collision_shapes`: A vector of collision shapes pointers.
  * `m_collision_objects`: A vector of collision objects pointers.

* `CollisionDistanceExtension`:
    * `double distanceToCollision`: return the distance to collision with the nearest obstacle (overloaded to also have a method which checks this collision along a motion).
