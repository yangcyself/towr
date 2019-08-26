/******************************************************************************
Copyright (c) 2019, YangChenyu TianChangda Gaoyue. All rights reserved. [YCY]
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

#ifndef TOWR_MODELS_HEXAPED_GAIT_GENERATOR_H_
#define TOWR_MODELS_HEXAPED_GAIT_GENERATOR_H_

#include "gait_generator.h"

namespace towr {

/**
 * @brief Produces the contact sequence for a variety of four-legged gaits.
 *
 * @sa GaitGenerator for more documentation
 */
class HexapedGaitGenerator : public GaitGenerator {
public:
  HexapedGaitGenerator ();
  virtual ~HexapedGaitGenerator () = default;

private:
  GaitInfo GetGait(Gaits gait) const override;

  GaitInfo GetStrideStand() const;
  GaitInfo GetStrideFlight() const;
  GaitInfo GetStrideWalk() const;

  void SetCombo(Combos combo) override;


  // naming convention:, where the circle is is contact, front is right ->.
  // so RA LB and RC in contact is (TTR):   o  .
  //                                      .      o  
  //                                        o  .
  // flight-phase
  ContactState II_; 
  ContactState TT_L;
  ContactState TT_R;
  // stance-phase
  ContactState BB_;
};

} /* namespace towr */

#endif /* TOWR_MODELS_HEXAPED_GAIT_GENERATOR_H_ */
