////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2015, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
// may be used to endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

#include "collision_world_sbpl.h"

// standard includes
#include <stdexcept>

// system includes
#include <ros/ros.h>
#include <geometric_shapes/shape_operations.h>
#include <smpl/debug/visualize.h>

// module includes
#include "collision_common_sbpl.h"

namespace collision_detection {

static const char* LOG = "world_collisions";

static
auto ComputeWorldAABB(const World& world) -> moveit_msgs::OrientedBoundingBox
{
    moveit_msgs::OrientedBoundingBox bb;
    bb.pose.orientation.w = 1.0;
    bb.pose.orientation.x = 0.0;
    bb.pose.orientation.y = 0.0;
    bb.pose.orientation.z = 0.0;

    geometry_msgs::Point min_pt;
    min_pt.x = std::numeric_limits<double>::max();
    min_pt.y = std::numeric_limits<double>::max();
    min_pt.z = std::numeric_limits<double>::max();
    geometry_msgs::Point max_pt;
    max_pt.x = std::numeric_limits<double>::lowest();
    max_pt.y = std::numeric_limits<double>::lowest();
    max_pt.z = std::numeric_limits<double>::lowest();

    if (world.size() == 0) {
        bb.pose.position.x = bb.pose.position.y = bb.pose.position.z = 0.0;
        bb.extents.x = bb.extents.y = bb.extents.z = 0.0;
        return bb;
    }

    for (auto oit = world.begin(); oit != world.end(); ++oit) {
        auto& object = *oit->second;
        auto& object_id = object.id_;
        auto num_shapes = object.shapes_.size();
        ROS_DEBUG_NAMED(LOG, "%zu shapes in object", num_shapes);
        for (auto i = (size_t)0; i < num_shapes; ++i) {
            auto& pose = object.shape_poses_[i];
            auto shape = object.shapes_[i];
            auto extents = shapes::computeShapeExtents(shape.get());

            auto shape_min = Eigen::Vector3d(Eigen::Vector3d(pose.translation()) - 0.5 * extents);
            auto shape_max = Eigen::Vector3d(Eigen::Vector3d(pose.translation()) + 0.5 * extents);

            min_pt.x = std::min(min_pt.x, shape_min.x());
            min_pt.y = std::min(min_pt.y, shape_min.y());
            min_pt.z = std::min(min_pt.z, shape_min.z());
            max_pt.x = std::max(max_pt.x, shape_max.x());
            max_pt.y = std::max(max_pt.y, shape_max.y());
            max_pt.z = std::max(max_pt.z, shape_max.z());
        }
    }

    bb.pose.position.x = 0.5 * (min_pt.x + max_pt.x);
    bb.pose.position.y = 0.5 * (min_pt.y + max_pt.y);
    bb.pose.position.z = 0.5 * (min_pt.z + max_pt.z);
    bb.extents.x = max_pt.x - min_pt.x;
    bb.extents.y = max_pt.y - min_pt.y;
    bb.extents.z = max_pt.z - min_pt.z;
    return bb;
}

static
bool IsEmptyBoundingBox(const moveit_msgs::OrientedBoundingBox& bb)
{
    return bb.extents.x == 0.0 && bb.extents.y == 0.0 && bb.extents.z == 0.0;
}

static
auto MakeGrid(const CollisionGridConfig& config) -> smpl::OccupancyGridPtr
{
    ROS_DEBUG_NAMED(LOG, "  Creating Distance Field");
    ROS_DEBUG_NAMED(LOG, "    size: (%0.3f, %0.3f, %0.3f)", config.size_x, config.size_y, config.size_z);
    ROS_DEBUG_NAMED(LOG, "    origin: (%0.3f, %0.3f, %0.3f)", config.origin_x, config.origin_y, config.origin_z);
    ROS_DEBUG_NAMED(LOG, "    resolution: %0.3f", config.res_m);
    ROS_DEBUG_NAMED(LOG, "    max_distance: %0.3f", config.max_distance_m);

    auto ref_counted = true;

    auto dmap = std::make_shared<smpl::OccupancyGrid>(
            config.size_x,
            config.size_y,
            config.size_z,
            config.res_m,
            config.origin_x,
            config.origin_y,
            config.origin_z,
            config.max_distance_m,
            ref_counted);
    dmap->setReferenceFrame(config.frame_id);
    return dmap;
}

static
auto MakeCollisionRobotVisualization(
    smpl::collision::RobotCollisionState* rcs,
    smpl::collision::AttachedBodiesCollisionState* abcs,
    int gidx,
    const std_msgs::ColorRGBA* color,
    const std::string* frame_id,
    const std::string* ns)
    -> visualization_msgs::MarkerArray
{
    auto ma = GetCollisionMarkers(*rcs, *abcs, gidx);
    for (auto& m : ma.markers) {
        m.ns = *ns;
        m.header.frame_id = *frame_id;
        m.color = *color;
    }
    return ma;
}

static
auto MakeCollisionRobotVisualization(
    const CollisionWorldSBPL* cworld,
    smpl::collision::RobotCollisionState* rcs,
    smpl::collision::AttachedBodiesCollisionState* abcs,
    int gidx,
    const std_msgs::ColorRGBA* color)
    -> visualization_msgs::MarkerArray
{
    const std::string* frame_id;
    if (cworld->m_grid != NULL) {
        frame_id = &cworld->m_grid->getReferenceFrame();
    } else {
        frame_id = &cworld->m_parent_grid->getReferenceFrame();
    }

    std::string ns("world_collision");
    return MakeCollisionRobotVisualization(
            rcs, abcs, gidx, color, frame_id, &ns);
}

static
auto MakeCollisionRobotValidityVisualization(
    CollisionWorldSBPL* cworld,
    smpl::collision::RobotCollisionState* rcs,
    smpl::collision::AttachedBodiesCollisionState* abcs,
    int gidx,
    bool valid)
    -> visualization_msgs::MarkerArray
{
    std_msgs::ColorRGBA color;
    if (valid) {
        color.g = 1.0;
        color.r = color.b = 0.0;
        color.a = 1.0;
    } else {
        color.r = 1.0;
        color.g = color.b = 0.0;
        color.a = 1.0;
    }
    return MakeCollisionRobotVisualization(cworld, rcs, abcs, gidx, &color);
}

// CollisionWorldSBPL::CollisionWorldSBPL() : CollisionEnv()
// {
//     ROS_INFO_NAMED(LOG, "CollisionWorldSBPL()");
//     construct();
// }

// CollisionWorldSBPL::CollisionWorldSBPL(const WorldPtr& world) :
//     CollisionEnv(world)
// {
//     ROS_INFO_NAMED(LOG, "CollisionWorldSBPL(world = %p)", world.get());
//     construct();
//     registerWorldCallback();
// }

CollisionWorldSBPL::CollisionWorldSBPL(
    const CollisionWorldSBPL& other,
    const WorldPtr& world)
:
    CollisionEnv(other, world) // copies over the world
{
//    ROS_DEBUG_NAMED(LOG, "CollisionWorldSBPL(other = %p, world = %p)", &other, world.get());

    m_wcm_config = other.m_wcm_config;
    m_jcgm_map = other.m_jcgm_map;

    m_parent_grid = other.m_grid ? other.m_grid : other.m_parent_grid;
    m_parent_wcm = other.m_wcm ? other.m_wcm : other.m_parent_wcm;

    m_updaters = other.m_updaters;
    // NOTE: no need to copy observer handle
    registerWorldCallback();
    // NOTE: no need to copy node handle
}

CollisionWorldSBPL::~CollisionWorldSBPL()
{
//    ROS_DEBUG_NAMED(LOG, "~CollisionWorldSBPL()");
    auto& curr_world = getWorld();
    if (curr_world) {
        curr_world->removeObserver(m_observer_handle);
    }
}

auto CollisionWorldSBPL::distanceField(
    const std::string& robot_name,
    const std::string& group_name) const
    -> const smpl::DistanceMapInterface*
{
    if (m_grid) {
        return m_grid->getDistanceField().get();
    } else if (m_parent_grid) {
        return m_parent_grid->getDistanceField().get();
    } else {
        return nullptr;
    }
}

void CollisionWorldSBPL::checkRobotCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    // const CollisionRobotSBPL& robot,
    const robot_state::RobotState& state) const
{
    ROS_INFO_NAMED(LOG, "checkRobotCollision(req, res, robot, state)");

    const_cast<CollisionWorldSBPL*>(this)->checkRobotCollisionMutable(
            req, res, state); // robot
}

void CollisionWorldSBPL::checkRobotCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    // const CollisionRobotSBPL& robot,
    const robot_state::RobotState& state,
    const AllowedCollisionMatrix& acm) const
{
    const_cast<CollisionWorldSBPL*>(this)->checkRobotCollisionMutable(
            req, res, state, acm); // robot
}

void CollisionWorldSBPL::checkRobotCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    // const CollisionRobotSBPL& robot,
    const robot_state::RobotState& state1,
    const robot_state::RobotState& state2) const
{
    ROS_INFO_NAMED(LOG, "checkRobotCollision(req, res, robot, state1, state2)");

    const_cast<CollisionWorldSBPL*>(this)->checkRobotCollisionMutable(
            req, res, state1, state2); // robot
}

void CollisionWorldSBPL::checkRobotCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    // const CollisionRobot& robot,
    const robot_state::RobotState& state1,
    const robot_state::RobotState& state2,
    const AllowedCollisionMatrix& acm) const
{
    const_cast<CollisionWorldSBPL*>(this)->checkRobotCollisionMutable(
            req, res, state1, state2, acm); // robot
}

// void CollisionWorldSBPL::checkWorldCollision(
//     const CollisionRequest& req,
//     CollisionResult& res,
//     const CollisionWorld& other_world) const
// {
//     ROS_INFO_NAMED(LOG, "checkWorldCollision(req, res, other_world)");
//     // TODO: implement
//     setVacuousCollision(res);
// }

// void CollisionWorldSBPL::checkWorldCollision(
//     const CollisionRequest& req,
//     CollisionResult& res,
//     const CollisionWorld& other_world,
//     const AllowedCollisionMatrix& acm) const
// {
//     ROS_INFO_NAMED(LOG, "checkWorldCollision(req, res, other_world, acm)");
//     // TODO: implement
//     setVacuousCollision(res);
// }

#if COLLISION_DETECTION_SBPL_ROS_VERSION == COLLISION_DETECTION_SBPL_ROS_NOETIC

void CollisionWorldSBPL::distanceRobot(
    const collision_detection::DistanceRequest& req,
    collision_detection::DistanceResult& res,
    const moveit::core::RobotState&) const
{
    // TODO: implement
    assert(0);
}

void CollisionWorldSBPL::distanceSelf(
    const collision_detection::DistanceRequest& req,
    collision_detection::DistanceResult& res,
    const moveit::core::RobotState& state) const
{
    // TODO: implement
    assert(0);
}

#else

double CollisionWorldSBPL::distanceRobot(
    const CollisionRobot& robot,
    const robot_state::RobotState& state) const
{
    // TODO: implement
    ROS_INFO_NAMED(LOG, "distanceRobot(robot, state)");
    return -1.0;
}

double CollisionWorldSBPL::distanceRobot(
    const CollisionRobot& robot,
    const robot_state::RobotState& state,
    const AllowedCollisionMatrix& acm) const
{
    // TODO: implement
    ROS_INFO_NAMED(LOG, "distanceRobot(robot, state, acm)");
    return -1.0;
}

double CollisionWorldSBPL::distanceWorld(const CollisionWorld& world) const
{
    // TODO: implement
    ROS_INFO_NAMED(LOG, "distanceWorld(world)");
    return -1.0;
}

double CollisionWorldSBPL::distanceWorld(
    const CollisionWorld& world,
    const AllowedCollisionMatrix& acm) const
{
    // TODO: implement
    ROS_INFO_NAMED(LOG, "distanceWorld(world, acm)");
    return -1.0;
}

#endif

void CollisionWorldSBPL::setWorld(const WorldPtr& world)
{
    // deregister update callback (we should always have a callback registered
    // if we have a world)
    auto& curr_world = getWorld();
    if (curr_world) {
        curr_world->removeObserver(m_observer_handle);
    }

    CollisionEnv::setWorld(world);
    ROS_INFO_NAMED(LOG, "setWorld(const WorldPtr&)");

    registerWorldCallback();
}

void CollisionWorldSBPL::construct()
{
    ros::NodeHandle ph("~");

    auto world_collision_model_param = "world_collision_model";
    LoadCollisionGridConfig(ph, world_collision_model_param, m_wcm_config);

    LoadJointCollisionGroupMap(ph, m_jcgm_map);

    m_grid = MakeGrid(m_wcm_config);
    m_wcm = std::make_shared<smpl::collision::WorldCollisionModel>(m_grid.get());

    // TODO: allowed collisions matrix

    ROS_INFO("Sleep to allow publish to set up");
    ros::Duration(0.5).sleep();
    ROS_INFO("Done sleeping");

    // publish collision world visualizations
    SV_SHOW_INFO_NAMED("collision_world_bounds", m_grid->getBoundingBoxVisualization());
}

void CollisionWorldSBPL::copyOnWrite()
{
    if (!m_wcm) {
        ROS_DEBUG_NAMED(LOG, "Spawn derivative world collision model");
        assert(!m_grid);

        // create our own grid
        m_grid = MakeGrid(m_wcm_config);

        // copy over state from parent world collision model
        if (m_parent_wcm) {
            m_wcm = std::make_shared<smpl::collision::WorldCollisionModel>(
                    *m_parent_wcm, m_grid.get());

            m_parent_grid.reset();
            m_parent_wcm.reset();
        }
    }
}

auto CollisionWorldSBPL::FindObjectRepPair(
    const World::ObjectConstPtr& object)
    -> std::vector<ObjectRepPair>::iterator
{
    auto has_object = [&](const ObjectRepPair& op) {
        return op.world_object == object;
    };
    return std::find_if(
            begin(m_collision_objects), end(m_collision_objects), has_object);
}

auto CollisionWorldSBPL::getCollisionStateUpdater(
    const CollisionRobotSBPL& collision_robot,
    const moveit::core::RobotModel& robot_model)
    -> CollisionStateUpdaterPtr
{
    // return an existing updater if available
    auto it = m_updaters.find(robot_model.getName());
    if (it != m_updaters.end()) {
        return it->second;
    }

    ROS_INFO_NAMED(LOG, "Create Collision State Updater for '%s'", robot_model.getName().c_str());

    auto gm = std::make_shared<CollisionStateUpdater>();
    if (!gm->init(robot_model, collision_robot.robotCollisionModel())) {
        return CollisionStateUpdaterPtr();
    }

    // store the successfully initialized group model
    m_updaters[robot_model.getName()] = gm;
    return gm;
}

void CollisionWorldSBPL::registerWorldCallback()
{
//    ROS_DEBUG_NAMED(LOG, "Registering world observer callback");
    auto ocfn = boost::bind(&CollisionWorldSBPL::worldUpdate, this, _1, _2);
    m_observer_handle = getWorld()->addObserver(ocfn);
}

void CollisionWorldSBPL::worldUpdate(
    const World::ObjectConstPtr& object,
    World::Action action)
{
    ROS_DEBUG_NAMED(LOG, "CollisionWorldSBPL::worldUpdate()");
    ROS_DEBUG_NAMED(LOG, "  id: %s", object->id_.c_str());
    ROS_DEBUG_NAMED(LOG, "  shapes: %zu", object->shapes_.size());
    ROS_DEBUG_NAMED(LOG, "  shape_poses: %zu", object->shape_poses_.size());
    if (action & World::ActionBits::UNINITIALIZED) {
        ROS_DEBUG_NAMED(LOG, "  action: UNINITIALIZED");
        processWorldUpdateUninitialized(object);
    }
    else if (action & World::ActionBits::CREATE) {
        ROS_DEBUG_NAMED(LOG, "  action: CREATE");
        processWorldUpdateCreate(object);
    }
    else if (action & World::ActionBits::DESTROY) {
        ROS_DEBUG_NAMED(LOG, "  action: DESTROY");
        processWorldUpdateDestroy(object);
    }
    else if (action & World::ActionBits::MOVE_SHAPE) {
        ROS_DEBUG_NAMED(LOG, "  action: MOVE_SHAPE");
        processWorldUpdateMoveShape(object);
    }
    else if (action & World::ActionBits::ADD_SHAPE) {
        ROS_DEBUG_NAMED(LOG, "  action: ADD_SHAPE");
        processWorldUpdateAddShape(object);
    }
    else if (action & World::ActionBits::REMOVE_SHAPE)  {
        ROS_DEBUG_NAMED(LOG, "  action: REMOVE_SHAPE");
        processWorldUpdateRemoveShape(object);
    }
}

void CollisionWorldSBPL::setVacuousCollision(CollisionResult& res) const
{
    res.collision = true;
    res.contact_count = 0;
    res.contacts.clear();
    res.cost_sources.clear();
    res.distance = 0.0;
}

void CollisionWorldSBPL::checkRobotCollisionMutable(
    const CollisionRequest& req,
    CollisionResult& res,
    // const CollisionRobot& robot,
    const robot_state::RobotState& state)
{
    // TODO: implement
    ROS_ERROR_NAMED(LOG, "checkRobotCollision(req, res, robot, state)");
    setVacuousCollision(res);
}

/// Note: The output CollisionResult is shared between multiple collision
/// checking calls (i.e. both for world and self collisions). The policy is to
/// only set fields when a collision occurs and not to clear fields.
void CollisionWorldSBPL::checkRobotCollisionMutable(
    const CollisionRequest& req,
    CollisionResult& res,
    // const CollisionRobotSBPL& robot,
    const robot_state::RobotState& state,
    const AllowedCollisionMatrix& acm)
{
    auto& crobot = (const CollisionRobotSBPL&) other;
    auto& rcm = crobot.robotCollisionModel();
    if (state.getRobotModel()->getName() != rcm->name()) {
        ROS_ERROR_NAMED(LOG, "Collision Robot Model does not match Robot Model");
        setVacuousCollision(res);
        return;
    }

    auto gm = getCollisionStateUpdater(crobot, *state.getRobotModel());
    if (!gm) {
        ROS_ERROR_NAMED(LOG, "Failed to get Group Model for robot '%s', group '%s'", state.getRobotModel()->getName().c_str(), req.group_name.c_str());
        setVacuousCollision(res);
        return;
    }

    auto jgcgit = m_jcgm_map.find(req.group_name);
    auto& collision_group_name =
            jgcgit == end(m_jcgm_map) ?  req.group_name : jgcgit->second;

    if (!rcm->hasGroup(collision_group_name)) {
        ROS_ERROR_NAMED(LOG, "No group '%s' found in the Robot Collision Model", collision_group_name.c_str());
        setVacuousCollision(res);
        return;
    }

    auto gidx = rcm->groupIndex(collision_group_name);

    gm->update(state);

    assert(m_wcm != NULL || m_parent_wcm != NULL);
    auto ewcm = m_wcm != NULL ? m_wcm.get() : m_parent_wcm.get();

    smpl::collision::WorldCollisionDetector wcd(rcm.get(), ewcm);

    double dist;
    auto valid = wcd.checkCollision(
            *gm->collisionState(),
            *gm->attachedBodiesCollisionState(),
            gidx,
            dist);

    ROS_INFO_STREAM_COND_NAMED(req.verbose, LOG, "world valid: " << std::boolalpha << valid << ", dist: " << dist);
    ROS_DEBUG_STREAM_COND_NAMED(!req.verbose, LOG, "world valid: " << std::boolalpha << valid << ", dist: " << dist);

    // NOTE: Visualizations used to trigger on verbose requests, but we're
    // opting for a debug channel here for scenarios where there is no interface
    // to set the verbose flag. This is the case when using the provided
    // service call to query the collision detector.
    SV_SHOW_DEBUG_NAMED(
            "world_collision",
            MakeCollisionRobotValidityVisualization(
                    this,
                    gm->collisionState(),
                    gm->attachedBodiesCollisionState(),
                    gidx,
                    valid));

    res.collision = !valid;
    if (req.distance) {
        res.distance = std::min(res.distance, dist);
    }
    if (req.cost) {
        ROS_WARN_ONCE("Cost sources not computed by sbpl collision checker");
    }
    if (req.contacts) {
        ROS_WARN_ONCE("Contacts not computed by sbpl collision checker");
    }
}

void CollisionWorldSBPL::checkRobotCollisionMutable(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionRobot& robot,
    const robot_state::RobotState& state1,
    const robot_state::RobotState& state2)
{
    // TODO: implement
    ROS_ERROR_NAMED(LOG, "checkRobotCollision(req, res, robot, state1, state2)");
    setVacuousCollision(res);
}

void CollisionWorldSBPL::checkRobotCollisionMutable(
    const CollisionRequest& req,
    CollisionResult& res,
    const CollisionRobot& robot,
    const robot_state::RobotState& state1,
    const robot_state::RobotState& state2,
    const AllowedCollisionMatrix& acm)
{
    auto& crobot = (const CollisionRobotSBPL&)robot;
    auto& rcm = crobot.robotCollisionModel();
    auto& rmcm = crobot.robotMotionCollisionModel();

    assert(state1.getRobotModel()->getName() == rcm->name());
    assert(state2.getRobotModel()->getName() == rcm->name());

    auto gm = getCollisionStateUpdater(crobot, *state1.getRobotModel());
    if (!gm) {
        ROS_ERROR_NAMED(LOG, "Failed to get Group Model for robot '%s', group '%s'", state1.getRobotModel()->getName().c_str(), req.group_name.c_str());
        setVacuousCollision(res);
        return;
    }

    auto jgcgit = m_jcgm_map.find(req.group_name);
    auto& collision_group_name =
            jgcgit == m_jcgm_map.end() ? req.group_name : jgcgit->second;

    if (!rcm->hasGroup(collision_group_name)) {
        ROS_ERROR_NAMED(LOG, "No group '%s' found in the Robot Collision Model", collision_group_name.c_str());
        setVacuousCollision(res);
        return;
    }

    int gidx = rcm->groupIndex(collision_group_name);

//    gm->update(state1);

    assert(m_wcm != NULL || m_parent_wcm != NULL);
    auto ewcm = (m_wcm != NULL) ? m_wcm.get() : m_parent_wcm.get();

    smpl::collision::WorldCollisionDetector wcd(rcm.get(), ewcm);

    double dist;

    auto startvars = gm->getVariablesFor(state1);
    auto goalvars = gm->getVariablesFor(state2);
    bool valid = wcd.checkMotionCollision(
        *gm->collisionState(),
        *gm->attachedBodiesCollisionState(),
        *rmcm,
        startvars,
        goalvars,
        gidx,
        dist);

    SV_SHOW_DEBUG_NAMED(
            "world_collision",
            MakeCollisionRobotValidityVisualization(
                    this,
                    gm->collisionState(),
                    gm->attachedBodiesCollisionState(),
                    gidx,
                    valid));

    res.collision = !valid;
    if (req.distance) {
        res.distance = std::min(res.distance, dist);
    }
    if (req.cost) {
        ROS_WARN_ONCE("Cost sources not computed by sbpl collision checker");
    }
    if (req.contacts) {
        ROS_WARN_ONCE("Contacts not computed by sbpl collision checker");
    }
}

void CollisionWorldSBPL::processWorldUpdateUninitialized(
    const World::ObjectConstPtr& object)
{

}

using namespace smpl::collision;

void CollisionWorldSBPL::processWorldUpdateCreate(
    const World::ObjectConstPtr& object)
{
    copyOnWrite();

    // check for existing collision object
    auto it = FindObjectRepPair(object);
    if (it != end(m_collision_objects)) {
        ROS_WARN_NAMED(LOG, "Object '%s' already exists in Collision World", object->id_.c_str());
        return;
    }

    assert(object->shapes_.size() == object->shape_poses_.size());

    ObjectRepPair op;
    op.world_object = object;
    ConvertObjectToCollisionObjectShallow(object, op.shapes, op.collision_object);

    // attempt insertion into the world collision model
    auto inserted = m_wcm->insertObject(op.collision_object.get());
    assert(inserted);

    m_collision_objects.push_back(std::move(op));
}

void CollisionWorldSBPL::processWorldUpdateDestroy(
    const World::ObjectConstPtr& object)
{
    copyOnWrite();

    auto it = FindObjectRepPair(object);
    if (it == end(m_collision_objects)) {
        ROS_WARN_NAMED(LOG, "Object '%s' not in the Collision World", object->id_.c_str());
        return;
    }

    auto removed = m_wcm->removeObject(it->collision_object.get());
    assert(removed);

    m_collision_objects.erase(it);
}

void CollisionWorldSBPL::processWorldUpdateMoveShape(
    const World::ObjectConstPtr& object)
{
    copyOnWrite();

    auto it = FindObjectRepPair(object);
    if (it == end(m_collision_objects)) {
        ROS_WARN_NAMED(LOG, "Object '%s' not in the Collision World", object->id_.c_str());
        return;
    }

    auto res = m_wcm->moveShapes(it->collision_object.get());
    assert(res);
}

void CollisionWorldSBPL::processWorldUpdateAddShape(
    const World::ObjectConstPtr& object)
{
    copyOnWrite();
    auto it = FindObjectRepPair(object);
    if (it == end(m_collision_objects)) {
        ROS_WARN_NAMED(LOG, "Object '%s' not in the Collision World", object->id_.c_str());
        return;
    }

    auto res = m_wcm->insertShapes(it->collision_object.get());
    assert(res);
}

void CollisionWorldSBPL::processWorldUpdateRemoveShape(
    const World::ObjectConstPtr& object)
{
    copyOnWrite();
    auto it = FindObjectRepPair(object);
    if (it == end(m_collision_objects)) {
        ROS_WARN_NAMED(LOG, "Object '%s' not in the Collision World", object->id_.c_str());
        return;
    }

    auto res = m_wcm->removeShapes(it->collision_object.get());
    assert(res);
}

} // collision_detection
