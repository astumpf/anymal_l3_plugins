#include <anymal_l3_plugins/anymal_turning_state_generator.h>

#include <l3_libs/yaml_parser.h>

#include <l3_plugins/robot_model.h>

#include <l3_footstep_planning_plugins/std/step_range_polygon.h>

#include <anymal_l3_plugins/anymal_kinematics.h>

namespace anymal_footstep_planning
{
using namespace anymal_l3;

AnymalTurningStateGenerator::AnymalTurningStateGenerator()
  : UseMaskGeneratorPlugin("anymal_turning_state_generator")
  , PolygonalStateGenerator("anymal_turning_state_generator")
  , travel_yaw_use_mask_(8)
{}

bool AnymalTurningStateGenerator::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!UseMaskGeneratorPlugin::loadParams(params))
    return false;

  //  if (!PolygonalStateGenerator::loadParams(params)) // we do not rely on classic polygonal state generator
  if (!StateGeneratorPlugin::loadParams(params))
    return false;

  goal_yaw_use_mask_ = param("goal_yaw_use_mask", static_cast<unsigned int>(NO_USE), true);
  travel_yaw_use_mask_ = param("travel_yaw_use_mask", static_cast<unsigned int>(NO_USE), true);
  use_superimposition_ = param("use_superimposition", true, true);
  min_weight_ = param("min_weight", 0.1, true);

  if (!hasParam("use_mask"))
    setUseMask(goal_yaw_use_mask_ | travel_yaw_use_mask_);

  max_goal_dist_sq_ = param("max_goal_dist", 0.0, true);
  max_goal_dist_sq_ *= max_goal_dist_sq_;
  min_goal_dyaw_ = param("min_goal_dyaw", M_PI, true);

  if (min_goal_dyaw_ > M_PI)
    ROS_WARN("[AnymalTurningStateGenerator] 'min_goal_dyaw' cannot be greater than %0.4f!", M_PI);

  min_travel_dist_sq_ = param("min_travel_dist", 0.0, true);
  min_travel_dist_sq_ *= min_travel_dist_sq_;
  min_travel_dyaw_ = param("min_travel_dyaw", 0.0, true);

  if (min_travel_dyaw_ > M_PI_2)
    ROS_WARN("[AnymalTurningStateGenerator] 'min_travel_dyaw' cannot be greater than %0.4f!", M_PI_2);

  /// generate footsteps for rotation

  DiscreteResolution res;

  // read resolution
  if (hasParam("resolution"))
    res = DiscreteResolution(getSubset("resolution"));
  // use planner resolution
  else
    res = DiscreteResolution(params.getSubset("resolution"));

  // check parameter format
  XmlRpc::XmlRpcValue p;
  getParam("reachability_polygons/feet", p, XmlRpc::XmlRpcValue());

  if (p.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR_NAMED("AnymalTurningStateGenerator", "[AnymalTurningStateGenerator] Parameter 'reachability_polygons/feet' must be given as array (currently '%s')!",
                    l3::toString(p.getType()).c_str());
    return false;
  }

  // iterate all feet
  for (size_t i = 0; i < p.size(); i++)
  {
    FootIndex foot_idx;
    if (!getYamlValue(p[i], "idx", foot_idx))
      return false;

    const Pose& neutral_stance = RobotModel::description()->getFootInfo(foot_idx).neutral_stance;

    StepRangePolygon polygon;
    if (!polygon.fromYaml(p[i], res))
      return false;

    for (int y = polygon.min_y; y <= polygon.max_y; y++)
    {
      for (int x = polygon.min_x; x <= polygon.max_x; x++)
      {
        if (static_cast<double>(std::abs(x)) / static_cast<double>(std::abs(y)) > 1.0)
          continue;

        if (!polygon.pointWithinPolygon(x, y))
          continue;

        // special handling for generation of turning steps for ANYmal
        bool front_legs;
        if (foot_idx == LF_LEG || foot_idx == RF_LEG)
          front_legs = true;
        else if (foot_idx == LH_LEG || foot_idx == RH_LEG)
          front_legs = false;
        else
        {
          ROS_ERROR_NAMED("AnymalTurningStateGenerator", "[AnymalTurningStateGenerator] Ignoring unexpected foot index %u given in reachability polygon. Fix it immediately!",
                          foot_idx);
          continue;
        }

        if (front_legs)
        {
          if (y < 0)
            footstep_actions_right_turn_[foot_idx].push_back(FootStepAction(neutral_stance, foot_idx, res.toContX(x), res.toContY(y), 0.0, 0.0, res));
          else
            footstep_actions_left_turn_[foot_idx].push_back(FootStepAction(neutral_stance, foot_idx, res.toContX(x), res.toContY(y), 0.0, 0.0, res));
        }
        else
        {
          if (y < 0)
            footstep_actions_left_turn_[foot_idx].push_back(FootStepAction(neutral_stance, foot_idx, res.toContX(x), res.toContY(y), 0.0, 0.0, res));
          else
            footstep_actions_right_turn_[foot_idx].push_back(FootStepAction(neutral_stance, foot_idx, res.toContX(x), res.toContY(y), 0.0, 0.0, res));
        }
      }
    }
  }

  return true;
}

std::list<StateGenResult> AnymalTurningStateGenerator::generatePredStateResults(const PlanningState& state, const State& start, const State& goal, const ExpandStatesIdx& state_expansion_idx) const
{
  const State& current = start;
  const State& target = *state.getState();

  // near goal case
  double dyaw = 0.0;

  // determine turning direction
  if (checkIfPositionInTarget(current, target))
    dyaw = shortestAngularDistance(current.getFeetCenter().yaw(), target.getFeetCenter().yaw());
  else
  {
    double heading = calcHeading(current.getFeetCenter(), target.getFeetCenter());

    if (std::abs(shortestAngularDistance(heading, target.getFeetCenter().yaw())) > M_PI_2)
      dyaw = shortestAngularDistance(current.getFeetCenter().yaw() + M_PI, heading);
    else
      dyaw = shortestAngularDistance(current.getFeetCenter().yaw(), heading);
  }

  const FootStepActionSetMap& footsteps = dyaw > 0.0 ? footstep_actions_left_turn_ : footstep_actions_right_turn_;

  return PolygonalStateGenerator::generatePredFootholds(state, state_expansion_idx, footsteps);
}

std::list<StateGenResult> AnymalTurningStateGenerator::generateSuccStateResults(const PlanningState& state, const State& start, const State& goal, const ExpandStatesIdx& state_expansion_idx) const
{
  const State& current = *state.getState();
  const State& target = goal;

  // near goal case
  double dyaw = 0.0;

  // determine turning direction
  if (checkIfPositionInTarget(current, target))
    dyaw = shortestAngularDistance(current.getFeetCenter().yaw(), target.getFeetCenter().yaw());
  else
  {
    double heading = calcHeading(current.getFeetCenter(), target.getFeetCenter());

    if (std::abs(shortestAngularDistance(heading, target.getFeetCenter().yaw())) > M_PI_2)
      dyaw = shortestAngularDistance(current.getFeetCenter().yaw() + M_PI, heading);
    else
      dyaw = shortestAngularDistance(current.getFeetCenter().yaw(), heading);
  }

  const FootStepActionSetMap& footsteps = dyaw > 0.0 ? footstep_actions_left_turn_ : footstep_actions_right_turn_;

  return PolygonalStateGenerator::generateSuccFootholds(state, state_expansion_idx, footsteps);
}

UseMask AnymalTurningStateGenerator::determineUseMask(const State& current, const State& start, const State& goal, bool superimposition,
                                                      std::list<UseMaskSuperimposition>& masks) const
{
  UseMask mask = NO_USE;
  double neg_weight = 0.0;

  if (forwardSearch())
  {
    // near goal case
    double dyaw = std::abs(shortestAngularDistance(current.getFeetCenter().yaw(), goal.getFeetCenter().yaw()));

    // 1. Check goal case (highest priority)
    if (dyaw >= min_goal_dyaw_)
    {
      if (checkIfPositionInTarget(current, goal))
      {
        // ROS_INFO("Goal");
        mask = goal_yaw_use_mask_;
        if (!superimposition)
          return mask;
      }
    }

    if (superimposition)
    {
      if (mask & goal_yaw_use_mask_)
      {
        if (min_goal_dyaw_ > 0.0 && min_goal_dyaw_ < M_PI)
        {
          neg_weight = lerp(1.0, 0.0, M_PI - min_goal_dyaw_, dyaw - min_goal_dyaw_);
          neg_weight *= neg_weight;
        }
      }
      else
      {
        double weight = 0.0;

        if (dyaw < min_goal_dyaw_)
          weight = dyaw / min_goal_dyaw_;
        //      if (min_dist_sq > max_goal_dist_sq_ && min_dist_sq < 4.0 * max_goal_dist_sq_)
        //        weight = std::min(weight, max_goal_dist_sq_ / min_dist_sq);

        weight *= weight;
        if (weight > min_weight_)
          masks.push_back(UseMaskSuperimposition(goal_yaw_use_mask_, weight));
      }
    }

    // 2. check travel case
    Point p = goal.getFeetCenter().getPosition() - current.getFeetCenter().getPosition();
    double dist_sq = norm_sq(p.x(), p.y());

    double travel_yaw = calcOrientation(p);
    double travel_forward_dyaw = std::abs(shortestAngularDistance(current.getFeetCenter().yaw(), travel_yaw));
    double travel_backward_dyaw = std::abs(shortestAngularDistance(current.getFeetCenter().yaw() + M_PI, travel_yaw));
    double travel_dyaw = std::min(travel_forward_dyaw, travel_backward_dyaw);

    if (travel_dyaw >= min_travel_dyaw_ && dist_sq >= min_travel_dist_sq_)
    {
      if (mask == NO_USE)
      {
        // ROS_INFO("Travel");
        mask = travel_yaw_use_mask_;
        if (!superimposition)
          return mask;
      }
    }

    if (superimposition)
    {
      if (mask & travel_yaw_use_mask_)
      {
        if (min_travel_dyaw_ < M_PI_2)
          neg_weight = lerp(1.0, 0.0, M_PI_2 - min_travel_dyaw_, travel_dyaw - min_travel_dyaw_);
        neg_weight = std::max(neg_weight, min_travel_dist_sq_ / dist_sq);

        neg_weight *= neg_weight;
      }
      else
      {
        double weight = 0.0;

        if (travel_dyaw < min_travel_dyaw_)
          weight = travel_dyaw / min_travel_dyaw_;
        if (dist_sq < min_travel_dist_sq_)
          weight = std::min(weight, dist_sq / min_travel_dist_sq_);

        weight *= weight;
        if (weight > min_weight_)
          masks.push_back(UseMaskSuperimposition(travel_yaw_use_mask_, weight));
      }
    }
  }
  else
  {
    /// @todo
    ROS_ERROR_ONCE_NAMED("AnymalTurningStateGenerator", "[AnymalTurningStateGenerator] Backward planning not implemented yet!");
  }

  if (mask != NO_USE)
  {
    if (superimposition && neg_weight > min_weight_)
      masks.push_back(UseMaskSuperimposition(getNegUseMask(), neg_weight));
    return mask;
  }
  else
  {
    // ROS_INFO("Negative");
    return getNegUseMask();
  }
}

bool AnymalTurningStateGenerator::checkIfPositionInTarget(const State& current, const State& target) const
{
  Point p = target.getFeetCenter().getPosition() - current.getFeetCenter().getPosition();

  // check if robot center in goal
  if (norm_sq(p.x(), p.y()) <= max_goal_dist_sq_)
    return true;
  // otherwise check if at least on foot is in goal
  else
  {
    for (Foothold::ConstPtr f_current : current.getFootholds())
    {
      ROS_ASSERT(f_current);

      Foothold::ConstPtr f_target = target.getFoothold(f_current->idx);
      if (!f_target)
        continue;

      p = f_target->pose().getPosition() - f_current->pose().getPosition();
      if (norm_sq(p.x(), p.y()) <= max_goal_dist_sq_)
        return true;
    }
  }

  return false;
}
}  // namespace anymal_footstep_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(anymal_footstep_planning::AnymalTurningStateGenerator, l3_footstep_planning::StateGeneratorPlugin)
