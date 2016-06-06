////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Benjamin Cohen, Andrew Dornbush
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

#ifndef sbpl_manip_collision_checker_h
#define sbpl_manip_collision_checker_h

// standard includes
#include <string>
#include <vector>

// system includes
#include <visualization_msgs/MarkerArray.h>

namespace sbpl {
namespace manip {

class CollisionChecker
{
public:

    CollisionChecker();
    virtual ~CollisionChecker();

    /// \brief Return whether a state is valid.
    /// \param[in] angles The joint angles of the default joint group
    /// \param[in] verbose Whether to produce verbose output
    /// \param[in] visualize Whether to store collision details for the next
    ///     call to getVisualization
    /// \param[out] dist The distance to the nearest obstacle
    /// \return Whether the state is valid
    virtual bool isStateValid(
        const std::vector<double>& angles,
        bool verbose,
        bool visualize,
        double &dist) = 0;

    /// \brief Return whether the interpolated path between two points is valid.
    ///
    /// Need not include the endpoints.
    ///
    /// \param[in] angles0 The start configuration of the default joint group
    /// \param[in] angles1 The start configuration of the default joint group
    /// \param[out] path_length The number of waypoints in the path between
    ///     angles0 and angles1
    /// \param[out] num_checks The number of collision checks to perform
    /// \param[out] dist The distance to the nearest obstacle
    /// \return Whether the interpolated path is valid
    virtual bool isStateToStateValid(
        const std::vector<double>& angles0,
        const std::vector<double>& angles1,
        int& path_length,
        int& num_checks,
        double& dist) = 0;

    /// \brief Return a linearly interpolated path between two joint states.
    ///        Oddly pure virtual because of its usage in post_processing.cpp.
    /// \param[in] start The start configuration of the default joint group
    /// \param[in] end The end configuration of the default joint group
    /// \param[in] inc The maximum joint angle increment to be applied between
    ///     each set of waypoints
    /// \param[out] path The output path
    /// \return Whether a valid linearly interpolated path could be constructed
    virtual bool interpolatePath(
        const std::vector<double>& start,
        const std::vector<double>& end,
        const std::vector<double>& inc,
        std::vector<std::vector<double>>& path) = 0;

    /* Visualizations */
    virtual visualization_msgs::MarkerArray getCollisionModelVisualization(
        const std::vector<double>& angles);

    virtual visualization_msgs::MarkerArray getVisualization(
        const std::string& type);
};

} // namespace manip
} // namespace sbpl

#endif
