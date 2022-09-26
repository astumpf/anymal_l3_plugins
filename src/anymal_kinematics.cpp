#include <anymal_l3_plugins/anymal_kinematics.h>

#include <l3_math/angles.h>

namespace anymal_l3
{
AnymalKinematicsPlugin::AnymalKinematicsPlugin()
  : KdlKinematics("anymal_kinematics")
{}

bool AnymalKinematicsPlugin::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!KdlKinematics::loadParams(params))
    return false;

  std::string foot_type = param("foot_type", std::string("ball_foot"), true);
  if (foot_type == "ball_foot")
    use_ball_foot_ = true;
  else
    use_ball_foot_ = false;

  return true;
}

bool AnymalKinematicsPlugin::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!KdlKinematics::initialize(params))
    return false;

  neutral_stance_.resize(4);
  neutral_stance_[LF_LEG] = { -0.2158078123039098, 0.9601238519723583, -1.927173597711323 };
  neutral_stance_[RF_LEG] = { -neutral_stance_[LF_LEG][0], neutral_stance_[LF_LEG][1], neutral_stance_[LF_LEG][2] };
  neutral_stance_[LH_LEG] = { neutral_stance_[LF_LEG][0], -neutral_stance_[LF_LEG][1], -neutral_stance_[LF_LEG][2] };
  neutral_stance_[RH_LEG] = { -neutral_stance_[LF_LEG][0], -neutral_stance_[LF_LEG][1], -neutral_stance_[LF_LEG][2] };

  return true;
}

Pose AnymalKinematicsPlugin::calcFeetCenter(const FootholdArray& footholds) const
{
  Pose pose = l3::calcFeetCenter(footholds);

  FootholdMap map = l3::footholdArrayToMap<FootholdMap>(footholds);

  ROS_ASSERT(map.find(LF_LEG) != map.end());
  ROS_ASSERT(map.find(RF_LEG) != map.end());
  ROS_ASSERT(map.find(LH_LEG) != map.end());
  ROS_ASSERT(map.find(RH_LEG) != map.end());

  // estimate body yaw using computing the vectors between the diagonal opposing foot poses
  Vector3 vec1 = map[LF_LEG].pose().getPosition() - map[RH_LEG].pose().getPosition();
  Vector3 vec2 = map[RF_LEG].pose().getPosition() - map[LH_LEG].pose().getPosition();

  double yaw1 = atan2(vec1.y(), vec1.x());
  double yaw2 = atan2(vec2.y(), vec2.x());
  double dyaw = shortestAngularDistance(yaw1, yaw2);

  pose.setYaw(yaw1 + 0.5 * dyaw);

  return pose;
}

Pose AnymalKinematicsPlugin::calcFeetCenter(const FootholdConstPtrArray& footholds) const
{
  FootholdArray temp;
  for (Foothold::ConstPtr f : footholds)
    temp.push_back(*f);
  return calcFeetCenter(temp);
}

bool AnymalKinematicsPlugin::calcLegIK(const Pose& base_pose, const Foothold& foothold, const std::vector<double>& cur_q, std::vector<double>& q) const
{
  // provide better start point for KDL
  if (cur_q.empty() && foothold.idx < neutral_stance_.size())
    return KdlKinematics::calcLegIK(base_pose, foothold, neutral_stance_[foothold.idx], q);
  else
    return KdlKinematics::calcLegIK(base_pose, foothold, cur_q, q);
}
}  // namespace anymal_l3

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(anymal_l3::AnymalKinematicsPlugin, l3::KinematicsPlugin)
