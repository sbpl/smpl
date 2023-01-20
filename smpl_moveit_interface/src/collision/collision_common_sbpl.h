////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
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

#ifndef SMPL_MOVEIT_INTERFACE_COLLISION_COMMON_SBPL_H
#define SMPL_MOVEIT_INTERFACE_COLLISION_COMMON_SBPL_H

// standard includes
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

// system includes
#include <ros/ros.h>
// #include <moveit/collision_detection/collision_world.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/CollisionObject.h>
#include <sbpl_collision_checking/allowed_collisions_interface.h>
#include <sbpl_collision_checking/attached_bodies_collision_model.h>
#include <sbpl_collision_checking/attached_bodies_collision_state.h>
#include <sbpl_collision_checking/robot_collision_model.h>
#include <sbpl_collision_checking/robot_collision_state.h>
#include <sbpl_collision_checking/shapes.h>

namespace collision_detection {

bool LoadJointCollisionGroupMap(
    ros::NodeHandle& nh,
    std::unordered_map<std::string, std::string>& _jcgm_map);

struct CollisionGridConfig
{
    std::string frame_id;
    double size_x;
    double size_y;
    double size_z;
    double origin_x;
    double origin_y;
    double origin_z;
    double res_m;
    double max_distance_m;
};

void LoadCollisionGridConfig(
    ros::NodeHandle& nh,
    const std::string& param_name,
    CollisionGridConfig& config);

struct StringPairHash
{
    using argument_type = std::pair<std::string, std::string>;
    using result_type = std::size_t;
    auto operator()(const argument_type& s) const -> result_type {
        const result_type h1 = std::hash<std::string>()(s.first);
        const result_type h2 = std::hash<std::string>()(s.second);
        return h1 ^ (h2 << 1);
    }
};

using TouchLinkSet = std::unordered_set<std::pair<std::string, std::string>, StringPairHash>;

// Represents an efficient pipeline for converting RobotStates into
// RobotCollisionStates for collision checking routines
class CollisionStateUpdater
{
public:

    CollisionStateUpdater();

    bool init(
        const moveit::core::RobotModel& robot,
        const smpl::collision::RobotCollisionModelConstPtr& rcm);

    void update(const moveit::core::RobotState& state);

    auto getVariablesFor(const moveit::core::RobotState& state)
        -> const std::vector<double>&;

    auto collisionState() -> smpl::collision::RobotCollisionState*
    { return m_rcs.get(); }

    auto collisionState() const -> const smpl::collision::RobotCollisionState*
    { return m_rcs.get(); }

    auto attachedBodiesCollisionModel()
        -> smpl::collision::AttachedBodiesCollisionModel*
    { return m_ab_model.get(); }

    auto attachedBodiesCollisionModel() const
        -> const smpl::collision::AttachedBodiesCollisionModel*
    { return m_ab_model.get(); }

    auto attachedBodiesCollisionState()
        -> smpl::collision::AttachedBodiesCollisionState*
    { return m_ab_state.get(); }

    auto attachedBodiesCollisionState() const
        -> const smpl::collision::AttachedBodiesCollisionState*
    { return m_ab_state.get(); }

    auto touchLinkSet() const
        -> const TouchLinkSet&
    { return m_touch_link_map; }

private:

    // corresponding joint variables indices in
    // RobotCollisionModel/RobotCollisionState
    std::vector<int> m_rcm_var_indices;

    // robot collision state joint variables for batch updating
    std::vector<double> m_rcm_vars;

    // the final RobotCollisionState
    smpl::collision::RobotCollisionStatePtr m_rcs;

    smpl::collision::AttachedBodiesCollisionModelPtr m_ab_model;
    smpl::collision::AttachedBodiesCollisionStatePtr m_ab_state;
    TouchLinkSet m_touch_link_map;

    bool m_inorder;

    bool extractRobotVariables(
        const moveit::core::RobotModel& model,
        std::vector<std::string>& variable_names,
        std::vector<int>& variable_indices);

    bool getRobotCollisionModelJointIndices(
        const std::vector<std::string>& joint_names,
        const smpl::collision::RobotCollisionModel& rcm,
        std::vector<int>& rcm_joint_indices);

    bool updateAttachedBodies(const moveit::core::RobotState& state);
};

using CollisionStateUpdaterPtr = std::shared_ptr<CollisionStateUpdater>;
using CollisionStateUpdaterConstPtr = std::shared_ptr<const CollisionStateUpdater>;

// proxy class to interface with CollisionSpace
class AllowedCollisionMatrixInterface :
    public smpl::collision::AllowedCollisionsInterface
{
public:

    AllowedCollisionMatrixInterface(const AllowedCollisionMatrix& acm) :
        AllowedCollisionsInterface(),
        m_acm(acm)
    { }

    virtual bool getEntry(
        const std::string& name1,
        const std::string& name2,
        smpl::collision::AllowedCollision::Type& type) const override
    {
        return m_acm.getEntry(name1, name2, type);
    }

private:

    const AllowedCollisionMatrix& m_acm;
};

class AllowedCollisionMatrixAndTouchLinksInterface :
    public AllowedCollisionMatrixInterface
{
public:

    AllowedCollisionMatrixAndTouchLinksInterface(
        const AllowedCollisionMatrix& acm,
        const TouchLinkSet& touch_link_map)
    :
        AllowedCollisionMatrixInterface(acm),
        m_touch_link_map(touch_link_map)
    { }

    virtual bool getEntry(
        const std::string& name1,
        const std::string& name2,
        smpl::collision::AllowedCollision::Type& type) const override
    {
        if (AllowedCollisionMatrixInterface::getEntry(name1, name2, type)) {
            return true;
        } else if (m_touch_link_map.find(std::make_pair(name1, name2)) !=
                m_touch_link_map.end())
        {
            type = smpl::collision::AllowedCollision::Type::ALWAYS;
            return true;
        } else {
            return false;
        }
    }

private:

    const TouchLinkSet& m_touch_link_map;
};

bool WorldObjectToCollisionObjectMsgFull(
    const World::Object& object,
    moveit_msgs::CollisionObject& collision_object);

bool WorldObjectToCollisionObjectMsgName(
    const World::Object& object,
    moveit_msgs::CollisionObject& collision_object);

void ConvertObjectToCollisionObjectShallow(
    const World::ObjectConstPtr& o,
    std::vector<std::unique_ptr<smpl::collision::CollisionShape>>& collision_shapes,
    std::unique_ptr<smpl::collision::CollisionObject>& collision_object);

auto GetCollisionMarkers(smpl::collision::RobotCollisionState& rcs)
    -> visualization_msgs::MarkerArray;

auto GetCollisionMarkers(smpl::collision::RobotCollisionState& rcs, int gidx)
    -> visualization_msgs::MarkerArray;

auto GetCollisionMarkers(
    smpl::collision::AttachedBodiesCollisionState& abcs,
    int gidx)
    -> visualization_msgs::MarkerArray;

auto GetCollisionMarkers(
    smpl::collision::RobotCollisionState& rcs,
    smpl::collision::AttachedBodiesCollisionState& abcs,
    int gidx)
    -> visualization_msgs::MarkerArray;

} // namespace collision_detection

#endif
