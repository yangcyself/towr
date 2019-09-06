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

#include <towr/initialization/hexaped_gait_generator.h>

#include <cassert>
#include <iostream>

#include <towr/models/endeffector_mappings.h>

namespace towr {

HexapedGaitGenerator::HexapedGaitGenerator ()
{
  int n_ee = 6;
  ContactState init(n_ee, false);
  II_ = init;
  TT_R = TT_L = init;
  TF_A = TF_B = TF_C = init;
  BB_ = init;

  // 3-3 gait
  TT_R.at(LA) = true; TT_R.at(RB) = true; TT_R.at(LC) = true;
  TT_L.at(RA) = true; TT_L.at(LB) = true; TT_L.at(RC) = true;

  // 2-4 gait
  TF_A.at(LB) = true; TF_A.at(RB) = true; TF_A.at(LC) = true; TF_A.at(RC) = true;
  TF_B.at(LA) = true; TF_B.at(RA) = true; TF_B.at(LC) = true; TF_B.at(RC) = true;
  TF_C.at(LB) = true; TF_C.at(RB) = true; TF_C.at(LA) = true; TF_C.at(RA) = true;


  BB_ = ContactState(n_ee, true);

  SetGaits({Stand});
}

void
HexapedGaitGenerator::SetCombo (Combos combo)
{
  std::cout<<"combo "<<combo<<std::endl;
  switch (combo) {
    case C0: SetGaits({Stand, Walk1, Walk1, Walk1, Walk1, Stand}); break;
    case C1: SetGaits({Stand, Walk2, Walk2, Walk2, Walk2E, Stand}); break;
    case C2: SetGaits({Stand, Walk1, Walk1, Walk1, Walk1, Walk1, Walk1, Stand}); break;
    case C3: SetGaits({Stand, Walk2, Walk2, Walk2, Walk2, Walk2, Walk2E, Stand}); break;
    default: assert(false); std::cout << "Gait not defined\n"; break;
  }
}

HexapedGaitGenerator::GaitInfo
HexapedGaitGenerator::GetGait (Gaits gait) const
{
  switch (gait) {
    case Stand:   return GetStrideStand();
    case Flight:  return GetStrideFlight();
    case Walk1:   return Get33Walk();
    case Walk2:   return Get24Walk();
    case Walk2E:  return RemoveTransition(Get24Walk());
    default: assert(false); // gait not implemented
  }
}

HexapedGaitGenerator::GaitInfo
HexapedGaitGenerator::GetStrideStand () const
{
  auto times =
  {
      0.2,
  };
  auto contacts =
  {
      BB_,
  };

  return std::make_pair(times, contacts);
}

HexapedGaitGenerator::GaitInfo
HexapedGaitGenerator::GetStrideFlight () const
{
  auto times =
  {
      0.5,
  };
  auto contacts =
  {
      II_,
  };

  return std::make_pair(times, contacts);
}

HexapedGaitGenerator::GaitInfo
HexapedGaitGenerator::Get33Walk () const
{
  double step = 0.3;
  double stance = 0.05;
  auto times =
  {
      step, stance,
      step, stance,
  };
  auto phase_contacts =
  {
      TT_L, BB_, // swing left foot
      TT_R, BB_, // swing right foot
  };

  return std::make_pair(times, phase_contacts);
}

HexapedGaitGenerator::GaitInfo
HexapedGaitGenerator::Get24Walk () const
{
  double step = 0.3;
  double stance = 0.05;
  auto times =
  {
      step, stance,
      step, stance,
      step, stance,
  };
  auto phase_contacts =
  {
      TF_A, BB_, // swing front foot
      TF_B, BB_, // swing middle foot
      TF_C, BB_, // swing back foot
  };

  return std::make_pair(times, phase_contacts);
}




} /* namespace towr */
