/******************************************************************************
Copyright (c) 2019, YangChenyu TianChangda Gaoyue. All rights reserved.

Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

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

#ifndef TOWR_MODELS_PARALLEL_KINEMATIC_MODEL_H_
#define TOWR_MODELS_PARALLEL_KINEMATIC_MODEL_H_

#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <towr/models/kinematic_model.h>
namespace towr {

/**
 * @brief  Contains all the robot specific kinematic parameters.
 *
 * This class is mainly used to formulate the @ref RangeOfMotionConstraint,
 * restricting each endeffector to stay inside it's kinematic range.
 *
 * @ingroup Robots
 */
class ParallelKinematicModel: public KinematicModel  {
public:
  using Ptr      = std::shared_ptr<KinematicModel>;
  using Vector3d = Eigen::Vector3d;
  using Matrix3d = Eigen::Matrix3d;
  ParallelKinematicModel(int n_ee) : KinematicModel(n_ee)
  {  }
  
  virtual Matrix3d GetRootPosition(int ee) const
  {
    return root_positions[ee];
  }

  virtual double GetMaximumLength() const
  {
    return max_length;
  }
  
  virtual double GetMinimumLength() const
  {
    return min_length;
  }

protected:
  std::vector<Matrix3d> root_positions;
  double min_length, max_length;
};
} /* namespace towr */

#endif /* TOWR_MODELS_KINEMATIC_MODEL_H_ */
