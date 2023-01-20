////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2008, Benjamin Cohen, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Benjamin Cohen
/// \author Andrew Dornbush
/// \author Dhruv Mauria Saxena

#ifndef SMPL_MANIP_LATTICE_CBS_H
#define SMPL_MANIP_LATTICE_CBS_H

// standard includes
#include <time.h>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// system includes
#include <boost/functional/hash.hpp>
#include <moveit_msgs/CollisionObject.h>

// project includes
#include <smpl/constants.hpp>
#include <smpl/angles.h>
#include <smpl/time.h>
#include <smpl/collision_checker.h>
#include <smpl/occupancy_grid.h>
#include <smpl/planning_params.h>
#include <smpl/robot_model.h>
#include <smpl/types.h>
#include <smpl/graph/robot_planning_space.h>
#include <smpl/graph/action_space.h>

namespace smpl {

class RobotHeuristic;

typedef std::vector<int> RobotCoord;

struct ManipLatticeCBSState
{
    RobotCoord coord;   // discrete coordinate
    RobotState state;   // corresponding continuous coordinate
    int t;              // time coordinate
};

inline
bool operator==(const ManipLatticeCBSState& a, const ManipLatticeCBSState& b)
{
    return (
        a.coord == b.coord &&
        a.t == b.t
    );
}

} // namespace smpl

namespace std {

template <>
struct hash<smpl::ManipLatticeCBSState>
{
    typedef smpl::ManipLatticeCBSState argument_type;
    typedef std::size_t result_type;
    result_type operator()(const argument_type& s) const;
};

} // namespace std

namespace smpl {

/// \class Discrete space constructed by expliciting discretizing each joint
class ManipLatticeCBS :
    public RobotPlanningSpace,
    public PoseProjectionExtension,
    public ExtractRobotStateExtension
{
public:

    ~ManipLatticeCBS();

    bool init(
        RobotModel* robot,
        CollisionChecker* checker,
        const std::vector<double>& resolutions,
        ActionSpace* actions);

    auto resolutions() const -> const std::vector<double>& { return m_coord_deltas; }
    auto actionSpace() -> ActionSpace* { return m_actions; }
    auto actionSpace() const -> const ActionSpace* { return m_actions; }

    auto getStartConfiguration() const -> RobotState;

    void setVisualizationFrameId(const std::string& frame_id);
    auto visualizationFrameId() const -> const std::string&;

    auto getDiscreteCenter(const RobotState& state) const -> RobotState;

    void clearStates();

    /// \name Reimplemented Public Functions from RobotPlanningSpace
    ///@{
    void GetLazySuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs,
        std::vector<bool>* true_costs) override;
    int GetTrueCost(int parent_id, int child_id) override;
    ///@}

    /// \name Required Public Functions from ExtractRobotStateExtension
    ///@{
    auto extractState(int state_id) -> const RobotState& override;
    ///@}

    /// \name Required Public Functions from PoseProjectionExtension
    ///@{
    bool projectToPose(int state_id, Isometry3& pos) override;
    ///@}

    /// \name Required Public Functions from RobotPlanningSpace
    ///@{
    bool setStart(const RobotState& state) override;
    bool setGoal(const GoalConstraint& goal) override;
    bool setPathConstraint(const GoalConstraint& path_constraint) override;
    bool disablePathConstraints() override { m_have_path_constraint = false; };
    int getStartStateID() const override;
    int getGoalStateID() const override;
    bool extractPath(
        const std::vector<int>& ids,
        std::vector<RobotState>& path) override;
    ///@}

    /// \name Required Public Functions from Extension
    ///@{
    virtual Extension* getExtension(size_t class_code) override;
    ///@}

    /// \name Required Public Functions from DiscreteSpaceInformation
    ///@{
    void GetSuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs) override;
    void PrintState(int state_id, bool verbose, FILE* fout = nullptr) override;
    void GetPreds(
        int state_id,
        std::vector<int>* preds,
        std::vector<int>* costs) override;
    ///@}

    void SetConstraints(const std::vector<std::vector<double> >& constraints) override {
        m_constraints = constraints;
        if (!m_constraints.empty()) {
            SMPL_ERROR("%d constraints set", m_constraints.size());
        }
    }
    void InitMovableSet(const std::vector<moveit_msgs::CollisionObject>& movables) override {
        m_movables.insert(m_movables.begin(), movables.begin(), movables.end());
        initMovablesMap();
    }
    void InitMovableAgentsTraj(const std::vector<std::pair<int, clutter::Trajectory>>&
        movable_agents_traj) override {
        m_movable_agents_traj = movable_agents_traj; 
    }

protected:

    /// \name discretization methods
    ///@{
    void coordToState(const RobotCoord& coord, RobotState& state) const;
    void stateToCoord(const RobotState& state, RobotCoord& coord) const;
    ///@}

    ManipLatticeCBSState* getHashEntry(int state_id) const;

    int getHashEntry(const RobotCoord& coord, const int& t);
    int createHashEntry(const RobotCoord& coord, const RobotState& state, const int& t);
    int getOrCreateState(const RobotCoord& coord, const RobotState& state, const int& t);
    int reserveHashEntry();

    Isometry3 computePlanningFrameFK(const RobotState& state) const;

    int cost(
        ManipLatticeCBSState* HashEntry1,
        ManipLatticeCBSState* HashEntry2,
        bool bState2IsGoal,
        bool is_movable_collision) const;

    bool checkAction(const RobotState& state, const Action& action, 
        const int& t, bool& is_movable_collision);

    bool isGoal(const RobotState& state);

    auto getStateVisualization(const RobotState& vars, const std::string& ns)
        -> std::vector<visual::Marker>;

private:

    ForwardKinematicsInterface* m_fk_iface = nullptr;
    ActionSpace* m_actions = nullptr;

    // cached from robot model
    std::vector<double> m_min_limits;
    std::vector<double> m_max_limits;
    std::vector<bool> m_continuous;
    std::vector<bool> m_bounded;

    std::vector<int> m_coord_vals;
    std::vector<double> m_coord_deltas;

    int m_goal_state_id = -1;
    int m_start_state_id = -1;

    // maps from coords to stateID
    typedef ManipLatticeCBSState StateKey;
    typedef PointerValueHash<StateKey> StateHash;
    typedef PointerValueEqual<StateKey> StateEqual;
    hash_map<StateKey*, int, StateHash, StateEqual> m_state_to_id;

    // maps from stateID to coords
    std::vector<ManipLatticeCBSState*> m_states;

    std::string m_viz_frame_id;

    std::vector<moveit_msgs::CollisionObject> m_movables;
    std::unordered_map<int, size_t> m_movables_map;
    std::vector<std::vector<double> > m_constraints;
    std::vector<std::pair<int, clutter::Trajectory>> m_movable_agents_traj;

    GoalConstraint m_path_constraint;
    bool m_have_path_constraint = false;

    bool setGoalPose(const GoalConstraint& goal);
    bool setGoalPoses(const GoalConstraint& goal);
    bool setGoalConfiguration(const GoalConstraint& goal);
    bool setUserGoal(const GoalConstraint& goal);

    void startNewSearch();

    void initMovablesMap();
    void updateMovablePoseFromConstraint(size_t c_id);
    void updateMovablePosesFromTraj(const int& timestep);
    bool processMovable(int mov_id, int idx, bool remove=false);
    bool setMovableMsg(const int& mov_id, bool remove);
    bool processMovableMsg(const int& mov_id, bool remove);
    bool addMovableMsg(const int& mov_id);
    bool removeMovableMsg(const int& mov_id);

    /// \name planning
    ///@{
    ///@}
};

} // namespace smpl

#endif
