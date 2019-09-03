/******************************************************************************
Copyright (c) 2018, Yang Chenyu  Tian Changda. All rights reserved. [YCY]

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_HEXPOD_MODEL_H_
#define TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_HEXPOD_MODEL_H_

#include <towr/models/kinematic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/models/endeffector_mappings.h>

namespace towr {

/**
 * @brief The Kinematics of the quadruped robot ANYmal.
 */
class HexpodKinematicModel : public ParallelKinematicModel {
public:
  HexpodKinematicModel () : ParallelKinematicModel(6)
  {
    const double x_nominal_1 = 0.528;
    const double x_nominal_2 = 0;
    const double y_nominal_1 = 0.304;
    const double y_nominal_2 = 0.609;
    const double z_nominal = -0.45;
    const double x_position_1 = 0.131;
    const double x_position_2 = 0.274;
    const double x_position_3 = 0.182;
    const double x_position_4 = 0.092;
    const double y_position_1 = 0.076;
    const double y_position_2 = 0.052;
    const double y_position_3 = 0.211;
    const double y_position_4 = 0.152;
    const double y_position_5 = 0.264;
    const double z_position_0 = 0.117;
    nominal_stance_.at(LA) <<  x_nominal_1,   y_nominal_1, z_nominal;
    nominal_stance_.at(LB) <<  x_nominal_2,   y_nominal_2, z_nominal;
    nominal_stance_.at(LC) << -x_nominal_1,   y_nominal_1, z_nominal;
    nominal_stance_.at(RA) <<  x_nominal_1,  -y_nominal_1, z_nominal;
    nominal_stance_.at(RB) <<  x_nominal_2,  -y_nominal_2, z_nominal;
    nominal_stance_.at(RC) << -x_nominal_1,  -y_nominal_1, z_nominal;


    root_positions.at(LA) <<  x_position_1, y_position_1  ,0,
                              x_position_2, y_position_2  ,z_position_0,
                              x_position_3, y_position_3  ,z_position_0;

    root_positions.at(LB) <<  0,  y_position_4 ,0,
                                x_position_4 , y_position_5  ,z_position_0,
                                -x_position_4  , y_position_5  ,z_position_0;

    root_positions.at(LC) <<  -x_position_1, y_position_1,0,
                               -x_position_3 ,y_position_3  ,z_position_0,
                               -x_position_2, y_position_2  ,z_position_0;

    root_positions.at(RA) <<  x_position_1, -y_position_1  ,0,
                               x_position_3, -y_position_3 ,z_position_0,
                               x_position_2, -y_position_2 ,z_position_0;

    root_positions.at(RB) <<  0, -y_position_4  ,0,
                              -x_position_4, -y_position_5   ,z_position_0,
                              x_position_4, -y_position_5  ,z_position_0;

    root_positions.at(RC) << -x_position_1, -y_position_1     ,0,
                              -x_position_2, -y_position_2 ,z_position_0,
                              -x_position_3, y_position_3  ,z_position_0;

    min_length = 0.4; // note here is the square of the length
    max_length = 0.9;
    max_dev_from_nominal_ << 0.2, 0.2, 0.2;
  }
};

/**
 * @brief The Dynamics of the quadruped robot ANYmal.
 */
class HexpodDynamicModel : public SingleRigidBodyDynamics {
public:
  HexpodDynamicModel()
  : SingleRigidBodyDynamics(29.5,
                    0.946438, 1.94478, 2.01835, 0.000938112, -0.00595386, -0.00146328,
                    6) {}
};

} // namespace towr

#endif /* TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_HEXPOD_MODEL_H_ */
