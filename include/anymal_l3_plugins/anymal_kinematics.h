//=================================================================================================
// Copyright (c) 2019, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef ANYMAL_KINEMATICS_PLUGIN_H__
#define ANYMAL_KINEMATICS_PLUGIN_H__

#include <ros/ros.h>

#include <l3_plugins/std/kdl_kinematics.h>

namespace anymal_l3
{
using namespace l3;

enum LEG_IDS
{
  LF_LEG = 0,
  RF_LEG = 1,
  LH_LEG = 2,
  RH_LEG = 3
};

class AnymalKinematicsPlugin
  : public KdlKinematics
{
public:
  // typedefs
  typedef boost::shared_ptr<AnymalKinematicsPlugin> Ptr;
  typedef boost::shared_ptr<const AnymalKinematicsPlugin> ConstPtr;

  AnymalKinematicsPlugin();

  bool loadParams(const vigir_generic_params::ParameterSet& params) override;
  bool initialize(const vigir_generic_params::ParameterSet& params) override;

  Pose calcFeetCenter(const FootholdArray& footholds) const  override;
  Pose calcFeetCenter(const FootholdConstPtrArray& footholds) const override;

  bool calcLegIK(const Pose& base_pose, const Foothold& foothold, const std::vector<double>& cur_q, std::vector<double>& q) const override;

protected:
  std::vector<std::vector<double>> neutral_stance_;

  bool use_ball_foot_;
};
}

#endif
