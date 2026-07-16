#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <ros/ros.h>

namespace
{
constexpr const char* kGroupName = "arm";
constexpr const char* kTcpLink = "gripper_tcp";
constexpr double kCell = 0.01;
constexpr double kNeighborRadius = 0.02;
constexpr double kMinRecommendMargin = 0.10;
constexpr double kCenterJoints[] = {
  -0.6814246216, 1.605672351, -1.34330569, -0.02285650537, 1.06290467, -1.808094127
};

struct Bounds
{
  double lower;
  double upper;
};

struct Candidate
{
  std::vector<double> joints;
  Eigen::Vector3d xyz;
  Eigen::Quaterniond quat;
  Eigen::Vector3d approach;
  Eigen::Vector3d closing;
  double approach_angle_deg{ 0.0 };
  double closing_yaw_deg{ 0.0 };
  double min_margin{ 0.0 };
  double joint_distance{ 0.0 };
  bool collision_free{ false };
  int pass{ 0 };
};

struct Cluster
{
  std::vector<const Candidate*> members;
  Eigen::Vector3d center{ Eigen::Vector3d::Zero() };
  Eigen::Vector3d min_xyz{ Eigen::Vector3d::Zero() };
  Eigen::Vector3d max_xyz{ Eigen::Vector3d::Zero() };
  double angle_min{ 0.0 };
  double angle_median{ 0.0 };
  double angle_max{ 0.0 };
  double min_margin{ 0.0 };
  double yaw_min{ 0.0 };
  double yaw_max{ 0.0 };
  const Candidate* representative{ nullptr };
  double angle_limit{ 0.0 };
  std::string label;
};

struct Validation
{
  bool ik_success{ false };
  std::string ik_error;
  bool state_valid{ false };
  bool transit_success{ false };
  std::string transit_error;
  std::size_t transit_points{ 0 };
  bool descend_all{ false };
  bool lift_all{ false };
  double best_depth{ 0.0 };
  std::map<double, double> descend_fraction;
  std::map<double, double> lift_fraction;
};

std::string jsonEscape(const std::string& value)
{
  std::ostringstream out;
  for (char c : value)
  {
    if (c == '"' || c == '\\')
      out << '\\' << c;
    else if (c == '\n')
      out << "\\n";
    else
      out << c;
  }
  return out.str();
}

std::string q(const std::string& value)
{
  return "\"" + jsonEscape(value) + "\"";
}

std::string num(double value)
{
  if (!std::isfinite(value))
    return "null";
  std::ostringstream out;
  out << std::setprecision(10) << value;
  return out.str();
}

std::string boolJson(bool value)
{
  return value ? "true" : "false";
}

std::string vecJson(const std::vector<double>& values)
{
  std::ostringstream out;
  out << "[";
  for (std::size_t i = 0; i < values.size(); ++i)
  {
    if (i)
      out << ",";
    out << num(values[i]);
  }
  out << "]";
  return out.str();
}

std::string vecJson(const Eigen::Vector3d& values)
{
  return vecJson(std::vector<double>{ values.x(), values.y(), values.z() });
}

std::string quatJson(const Eigen::Quaterniond& quat)
{
  return vecJson(std::vector<double>{ quat.x(), quat.y(), quat.z(), quat.w() });
}

void printJson(const std::string& prefix, const std::string& body)
{
  std::cout << prefix << " " << body << std::endl;
}

double halton(int index, int base)
{
  double result = 0.0;
  double f = 1.0 / static_cast<double>(base);
  int i = index;
  while (i > 0)
  {
    result += f * static_cast<double>(i % base);
    i /= base;
    f /= static_cast<double>(base);
  }
  return result;
}

double angleToDownDeg(const Eigen::Vector3d& approach)
{
  const Eigen::Vector3d down(0.0, 0.0, -1.0);
  const double dot = std::max(-1.0, std::min(1.0, approach.normalized().dot(down)));
  return std::acos(dot) * 180.0 / M_PI;
}

double jointDistance(const std::vector<double>& a, const std::vector<double>& b)
{
  double sum = 0.0;
  for (std::size_t i = 0; i < a.size() && i < b.size(); ++i)
    sum += (a[i] - b[i]) * (a[i] - b[i]);
  return std::sqrt(sum);
}

double minJointMargin(const std::vector<double>& joints, const std::vector<Bounds>& bounds)
{
  double margin = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < joints.size() && i < bounds.size(); ++i)
    margin = std::min(margin, std::min(joints[i] - bounds[i].lower, bounds[i].upper - joints[i]));
  return margin;
}

bool inWorkspace(const Eigen::Vector3d& xyz)
{
  return xyz.x() >= 0.15 && xyz.x() <= 0.55 && xyz.y() >= -0.30 && xyz.y() <= 0.30 && xyz.z() >= 0.12 &&
         xyz.z() <= 0.30;
}

std::vector<double> localHaltonSample(int index, const std::vector<double>& center, const std::vector<Bounds>& bounds,
                                      double radius)
{
  static const int primes[] = { 2, 3, 5, 7, 11, 13 };
  std::vector<double> joints;
  joints.reserve(center.size());
  for (std::size_t i = 0; i < center.size(); ++i)
  {
    const double t = halton(index, primes[i]) * 2.0 - 1.0;
    const double value = center[i] + t * radius;
    joints.push_back(std::max(bounds[i].lower, std::min(bounds[i].upper, value)));
  }
  return joints;
}

std::vector<std::vector<double>> structuredLocalSamples(const std::vector<double>& center,
                                                        const std::vector<Bounds>& bounds, double radius)
{
  std::vector<std::vector<double>> samples;
  samples.push_back(center);
  for (std::size_t joint = 0; joint < center.size(); ++joint)
  {
    for (double delta : { -radius, -radius * 0.5, radius * 0.5, radius })
    {
      std::vector<double> values = center;
      values[joint] = std::max(bounds[joint].lower, std::min(bounds[joint].upper, center[joint] + delta));
      samples.push_back(values);
    }
  }
  for (double d1 : { -radius, 0.0, radius })
  {
    for (double d2 : { -radius, 0.0, radius })
    {
      for (double d5 : { -radius, 0.0, radius })
      {
        std::vector<double> values = center;
        values[0] = std::max(bounds[0].lower, std::min(bounds[0].upper, center[0] + d1));
        values[1] = std::max(bounds[1].lower, std::min(bounds[1].upper, center[1] + d2));
        values[4] = std::max(bounds[4].lower, std::min(bounds[4].upper, center[4] + d5));
        samples.push_back(values);
      }
    }
  }
  return samples;
}

std::string cellKey(const Eigen::Vector3d& xyz)
{
  const int ix = static_cast<int>(std::floor(xyz.x() / kCell));
  const int iy = static_cast<int>(std::floor(xyz.y() / kCell));
  const int iz = static_cast<int>(std::floor(xyz.z() / kCell));
  return std::to_string(ix) + ":" + std::to_string(iy) + ":" + std::to_string(iz);
}

std::tuple<int, int, int> cellIndex(const Eigen::Vector3d& xyz)
{
  return std::make_tuple(static_cast<int>(std::floor(xyz.x() / kCell)),
                         static_cast<int>(std::floor(xyz.y() / kCell)),
                         static_cast<int>(std::floor(xyz.z() / kCell)));
}

std::string candidateJson(const Candidate& c)
{
  std::ostringstream out;
  out << "{";
  out << q("joint_positions") << ":" << vecJson(c.joints) << ",";
  out << q("tcp_xyz") << ":" << vecJson(c.xyz) << ",";
  out << q("tcp_quaternion") << ":" << quatJson(c.quat) << ",";
  out << q("approach_axis") << ":" << vecJson(c.approach) << ",";
  out << q("approach_angle_to_down_deg") << ":" << num(c.approach_angle_deg) << ",";
  out << q("closing_axis") << ":" << vecJson(c.closing) << ",";
  out << q("closing_axis_yaw_deg") << ":" << num(c.closing_yaw_deg) << ",";
  out << q("minimum_joint_limit_margin_rad") << ":" << num(c.min_margin) << ",";
  out << q("joint_distance_from_current") << ":" << num(c.joint_distance) << ",";
  out << q("pass") << ":" << c.pass << ",";
  out << q("collision_free") << ":" << boolJson(c.collision_free);
  out << "}";
  return out.str();
}

std::string moveItErrorName(const moveit::core::MoveItErrorCode& code)
{
  if (code == moveit::core::MoveItErrorCode::SUCCESS)
    return "SUCCESS";
  if (code == moveit::core::MoveItErrorCode::FAILURE)
    return "FAILURE";
  if (code == moveit::core::MoveItErrorCode::PLANNING_FAILED)
    return "PLANNING_FAILED";
  if (code == moveit::core::MoveItErrorCode::TIMED_OUT)
    return "TIMED_OUT";
  return "MOVEIT_ERROR_" + std::to_string(code.val);
}

std::string moveItMsgErrorName(int value)
{
  if (value == moveit_msgs::MoveItErrorCodes::SUCCESS)
    return "SUCCESS";
  if (value == moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION)
    return "NO_IK_SOLUTION";
  if (value == moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION)
    return "GOAL_IN_COLLISION";
  if (value == moveit_msgs::MoveItErrorCodes::TIMED_OUT)
    return "TIMED_OUT";
  return "MOVEIT_ERROR_" + std::to_string(value);
}

std::vector<Cluster> clusterCandidates(const std::vector<Candidate>& candidates, double angle_limit,
                                       const std::string& label)
{
  std::map<std::tuple<int, int, int>, std::vector<const Candidate*>> bins;
  for (const auto& c : candidates)
  {
    if (!c.collision_free || c.approach_angle_deg > angle_limit || c.min_margin < kMinRecommendMargin)
      continue;
    bins[cellIndex(c.xyz)].push_back(&c);
  }

  std::set<std::tuple<int, int, int>> visited;
  std::vector<Cluster> clusters;
  for (const auto& kv : bins)
  {
    if (visited.count(kv.first))
      continue;
    std::vector<const Candidate*> members;
    std::queue<std::tuple<int, int, int>> queue;
    queue.push(kv.first);
    visited.insert(kv.first);
    while (!queue.empty())
    {
      const auto current = queue.front();
      queue.pop();
      const auto it = bins.find(current);
      if (it != bins.end())
        members.insert(members.end(), it->second.begin(), it->second.end());
      int ix, iy, iz;
      std::tie(ix, iy, iz) = current;
      for (int dx = -2; dx <= 2; ++dx)
      {
        for (int dy = -2; dy <= 2; ++dy)
        {
          for (int dz = -2; dz <= 2; ++dz)
          {
            const double dist = std::sqrt(dx * dx + dy * dy + dz * dz) * kCell;
            if (dist > kNeighborRadius + 1e-9)
              continue;
            const auto neighbor = std::make_tuple(ix + dx, iy + dy, iz + dz);
            if (!visited.count(neighbor) && bins.count(neighbor))
            {
              visited.insert(neighbor);
              queue.push(neighbor);
            }
          }
        }
      }
    }
    if (members.size() < 50)
      continue;

    Cluster cluster;
    cluster.members = members;
    cluster.label = label;
    cluster.angle_limit = angle_limit;
    cluster.center = Eigen::Vector3d::Zero();
    cluster.min_xyz = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    cluster.max_xyz = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
    cluster.angle_min = std::numeric_limits<double>::infinity();
    cluster.angle_max = -std::numeric_limits<double>::infinity();
    cluster.min_margin = std::numeric_limits<double>::infinity();
    cluster.yaw_min = std::numeric_limits<double>::infinity();
    cluster.yaw_max = -std::numeric_limits<double>::infinity();
    std::vector<double> angles;
    for (const Candidate* c : members)
    {
      cluster.center += c->xyz;
      cluster.min_xyz = cluster.min_xyz.cwiseMin(c->xyz);
      cluster.max_xyz = cluster.max_xyz.cwiseMax(c->xyz);
      cluster.angle_min = std::min(cluster.angle_min, c->approach_angle_deg);
      cluster.angle_max = std::max(cluster.angle_max, c->approach_angle_deg);
      cluster.min_margin = std::min(cluster.min_margin, c->min_margin);
      cluster.yaw_min = std::min(cluster.yaw_min, c->closing_yaw_deg);
      cluster.yaw_max = std::max(cluster.yaw_max, c->closing_yaw_deg);
      angles.push_back(c->approach_angle_deg);
      if (!cluster.representative || std::make_tuple(c->approach_angle_deg, -c->min_margin, c->joint_distance) <
                                         std::make_tuple(cluster.representative->approach_angle_deg,
                                                         -cluster.representative->min_margin,
                                                         cluster.representative->joint_distance))
        cluster.representative = c;
    }
    cluster.center /= static_cast<double>(members.size());
    std::sort(angles.begin(), angles.end());
    cluster.angle_median = angles[angles.size() / 2];
    clusters.push_back(cluster);
  }

  std::sort(clusters.begin(), clusters.end(), [](const Cluster& a, const Cluster& b) {
    return std::make_tuple(a.angle_min, -a.min_margin, -static_cast<int>(a.members.size()),
                           a.representative ? a.representative->joint_distance : 999.0) <
           std::make_tuple(b.angle_min, -b.min_margin, -static_cast<int>(b.members.size()),
                           b.representative ? b.representative->joint_distance : 999.0);
  });
  return clusters;
}

std::string clusterJson(const Cluster& cluster, int rank)
{
  const Candidate& r = *cluster.representative;
  std::ostringstream out;
  out << "{";
  out << q("rank") << ":" << rank << ",";
  out << q("label") << ":" << q(cluster.label) << ",";
  out << q("angle_limit_deg") << ":" << num(cluster.angle_limit) << ",";
  out << q("cluster_size") << ":" << cluster.members.size() << ",";
  out << q("xyz_center") << ":" << vecJson(cluster.center) << ",";
  out << q("xyz_min") << ":" << vecJson(cluster.min_xyz) << ",";
  out << q("xyz_max") << ":" << vecJson(cluster.max_xyz) << ",";
  out << q("approach_angle_min_deg") << ":" << num(cluster.angle_min) << ",";
  out << q("approach_angle_median_deg") << ":" << num(cluster.angle_median) << ",";
  out << q("approach_angle_max_deg") << ":" << num(cluster.angle_max) << ",";
  out << q("minimum_joint_limit_margin_rad") << ":" << num(cluster.min_margin) << ",";
  out << q("available_gripper_yaw_range_deg") << ":[" << num(cluster.yaw_min) << "," << num(cluster.yaw_max)
      << "],";
  out << q("representative") << ":" << candidateJson(r);
  out << "}";
  return out.str();
}

geometry_msgs::Pose eigenPoseToMsg(const Eigen::Vector3d& xyz, const Eigen::Quaterniond& quat)
{
  geometry_msgs::Pose pose;
  pose.position.x = xyz.x();
  pose.position.y = xyz.y();
  pose.position.z = xyz.z();
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();
  return pose;
}

moveit::core::RobotState stateFromTrajectoryEnd(const moveit::core::RobotState& seed,
                                                const moveit_msgs::RobotTrajectory& trajectory,
                                                const moveit::core::JointModelGroup* jmg)
{
  moveit::core::RobotState state(seed);
  const auto& points = trajectory.joint_trajectory.points;
  if (!points.empty())
    state.setJointGroupPositions(jmg, points.back().positions);
  state.update();
  return state;
}

Validation validateCandidate(const Candidate& rep, const moveit::core::RobotState& current_state,
                             const moveit::core::RobotModelPtr& model,
                             const moveit::core::JointModelGroup* jmg,
                             const std::vector<std::string>& joint_names,
                             planning_scene_monitor::LockedPlanningSceneRO& scene,
                             ros::ServiceClient& ik_client,
                             moveit::planning_interface::MoveGroupInterface& move_group)
{
  Validation validation;
  moveit::core::RobotState seed_state(current_state);
  seed_state.setJointGroupPositions(jmg, rep.joints);
  seed_state.update();
  moveit_msgs::RobotState seed_msg;
  moveit::core::robotStateToRobotStateMsg(seed_state, seed_msg);

  moveit_msgs::GetPositionIK srv;
  srv.request.ik_request.group_name = kGroupName;
  srv.request.ik_request.ik_link_name = kTcpLink;
  srv.request.ik_request.robot_state = seed_msg;
  srv.request.ik_request.avoid_collisions = true;
  srv.request.ik_request.timeout = ros::Duration(2.0);
  srv.request.ik_request.pose_stamped.header.frame_id = model->getModelFrame();
  srv.request.ik_request.pose_stamped.header.stamp = ros::Time::now();
  srv.request.ik_request.pose_stamped.pose = eigenPoseToMsg(rep.xyz, rep.quat);
  const bool ik_call_ok = ik_client.call(srv);
  validation.ik_success = ik_call_ok && srv.response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
  validation.ik_error = ik_call_ok ? moveItMsgErrorName(srv.response.error_code.val) : "IK_SERVICE_CALL_FAILED";

  collision_detection::CollisionRequest collision_request;
  collision_request.group_name = kGroupName;
  collision_request.contacts = false;
  collision_detection::CollisionResult validity_result;
  scene->checkCollision(collision_request, validity_result, seed_state);
  validation.state_valid = !validity_result.collision;

  move_group.setStartStateToCurrentState();
  move_group.clearPoseTargets();
  move_group.setPoseTarget(eigenPoseToMsg(rep.xyz, rep.quat), kTcpLink);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const moveit::core::MoveItErrorCode plan_code = move_group.plan(plan);
  validation.transit_error = moveItErrorName(plan_code);
  validation.transit_points = plan.trajectory_.joint_trajectory.points.size();
  validation.transit_success = plan_code == moveit::core::MoveItErrorCode::SUCCESS && validation.transit_points > 0;

  const std::vector<double> depths{ 0.04, 0.05, 0.06, 0.07 };
  validation.descend_all = validation.transit_success;
  validation.lift_all = validation.transit_success;
  if (!validation.transit_success)
    return validation;

  moveit::core::RobotState hover_state = stateFromTrajectoryEnd(current_state, plan.trajectory_, jmg);
  for (double depth : depths)
  {
    geometry_msgs::Pose descend_pose = eigenPoseToMsg(rep.xyz + Eigen::Vector3d(0.0, 0.0, -depth), rep.quat);
    std::vector<geometry_msgs::Pose> descend_waypoints{ descend_pose };
    moveit_msgs::RobotTrajectory descend_traj;
    move_group.setStartState(hover_state);
    const double descend_fraction = move_group.computeCartesianPath(descend_waypoints, 0.01, 0.0, descend_traj, true);
    validation.descend_fraction[depth] = descend_fraction;
    validation.descend_all = validation.descend_all && descend_fraction >= 0.999;
    if (descend_fraction < 0.999)
    {
      validation.lift_fraction[depth] = 0.0;
      validation.lift_all = false;
      continue;
    }

    moveit::core::RobotState descend_state = stateFromTrajectoryEnd(hover_state, descend_traj, jmg);
    geometry_msgs::Pose hover_pose = eigenPoseToMsg(rep.xyz, rep.quat);
    std::vector<geometry_msgs::Pose> lift_waypoints{ hover_pose };
    moveit_msgs::RobotTrajectory lift_traj;
    move_group.setStartState(descend_state);
    const double lift_fraction = move_group.computeCartesianPath(lift_waypoints, 0.01, 0.0, lift_traj, true);
    validation.lift_fraction[depth] = lift_fraction;
    validation.lift_all = validation.lift_all && lift_fraction >= 0.999;
    if (descend_fraction >= 0.999 && lift_fraction >= 0.999)
      validation.best_depth = depth;
  }
  return validation;
}

std::string validationJson(const Validation& validation)
{
  std::ostringstream out;
  out << "{";
  out << q("ik_success") << ":" << boolJson(validation.ik_success) << ",";
  out << q("ik_error") << ":" << q(validation.ik_error) << ",";
  out << q("state_validity") << ":" << boolJson(validation.state_valid) << ",";
  out << q("transit_success") << ":" << boolJson(validation.transit_success) << ",";
  out << q("transit_error") << ":" << q(validation.transit_error) << ",";
  out << q("transit_trajectory_points") << ":" << validation.transit_points << ",";
  out << q("descend_all_depths_fraction_1") << ":" << boolJson(validation.descend_all) << ",";
  out << q("lift_all_depths_fraction_1") << ":" << boolJson(validation.lift_all) << ",";
  out << q("validated_descend_depth_m") << ":" << num(validation.best_depth) << ",";
  out << q("descend_fractions") << ":{";
  bool first = true;
  for (const auto& kv : validation.descend_fraction)
  {
    if (!first)
      out << ",";
    first = false;
    out << q(num(kv.first)) << ":" << num(kv.second);
  }
  out << "},";
  out << q("lift_fractions") << ":{";
  first = true;
  for (const auto& kv : validation.lift_fraction)
  {
    if (!first)
      out << ",";
    first = false;
    out << q(num(kv.first)) << ":" << num(kv.second);
  }
  out << "}";
  out << "}";
  return out.str();
}

void writeYaml(const std::string& path, const Cluster& cluster, const Candidate& rep, const Validation& validation,
               const std::string& source_commit)
{
  std::ofstream out(path.c_str());
  if (!out)
  {
    ROS_ERROR_STREAM("Failed to open validated grasp region YAML for writing: " << path);
    return;
  }
  out << "source_commit: " << source_commit << "\n";
  out << "regions:\n";
  out << "  - name: local_validated_representative_region\n";
  out << "    usage: [source_pick, destination_place]\n";
  out << "    tcp_hover_xyz:\n";
  out << "      center: " << vecJson(cluster.center) << "\n";
  out << "      min: " << vecJson(cluster.min_xyz) << "\n";
  out << "      max: " << vecJson(cluster.max_xyz) << "\n";
  out << "    cup_surface_regions:\n";
  for (double hover : { 0.05, 0.07, 0.10 })
  {
    Eigen::Vector3d center = cluster.center;
    Eigen::Vector3d minv = cluster.min_xyz;
    Eigen::Vector3d maxv = cluster.max_xyz;
    center.z() -= hover;
    minv.z() -= hover;
    maxv.z() -= hover;
    out << "      \"" << num(hover) << "\":\n";
    out << "        center: " << vecJson(center) << "\n";
    out << "        min: " << vecJson(minv) << "\n";
    out << "        max: " << vecJson(maxv) << "\n";
  }
  out << "    representative_cup_surface_xyz:\n";
  for (double hover : { 0.05, 0.07, 0.10 })
  {
    Eigen::Vector3d surface = rep.xyz;
    surface.z() -= hover;
    out << "      \"" << num(hover) << "\": " << vecJson(surface) << "\n";
  }
  out << "    representative_tcp_quaternion: " << quatJson(rep.quat) << "\n";
  out << "    representative_joint_state: " << vecJson(rep.joints) << "\n";
  out << "    allowed_approach_angle_deg: [0.0, " << num(cluster.angle_limit) << "]\n";
  out << "    observed_approach_angle_deg:\n";
  out << "      min: " << num(cluster.angle_min) << "\n";
  out << "      median: " << num(cluster.angle_median) << "\n";
  out << "      max: " << num(cluster.angle_max) << "\n";
  out << "    closing_axis_yaw_range_deg: [" << num(cluster.yaw_min) << ", " << num(cluster.yaw_max) << "]\n";
  out << "    minimum_joint_limit_margin_rad: " << num(cluster.min_margin) << "\n";
  out << "    cluster_size: " << cluster.members.size() << "\n";
  out << "    validated_descend_depth_m: " << num(validation.best_depth) << "\n";
  out << "    plan_validation:\n";
  out << "      ik_success: " << (validation.ik_success ? "true" : "false") << "\n";
  out << "      state_validity: " << (validation.state_valid ? "true" : "false") << "\n";
  out << "      transit_success: " << (validation.transit_success ? "true" : "false") << "\n";
  out << "      transit_trajectory_points: " << validation.transit_points << "\n";
  out << "      descend_all_depths_fraction_1: " << (validation.descend_all ? "true" : "false") << "\n";
  out << "      lift_all_depths_fraction_1: " << (validation.lift_all ? "true" : "false") << "\n";
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "piper_fk_local_workspace_mapper");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle nh("~");

  int pass1_samples = 250000;
  int pass2_samples = 250000;
  std::string output_yaml = "/root/ABot-Claw/robot_layer/arm_piper/agent_server/config/piper_validated_grasp_regions.yaml";
  std::string source_commit = "ae5c1f705755f864524aae2741b8fa17ee912c93";
  nh.param("pass1_samples", pass1_samples, pass1_samples);
  nh.param("pass2_samples", pass2_samples, pass2_samples);
  nh.param("output_yaml", output_yaml, output_yaml);
  nh.param("source_commit", source_commit, source_commit);
  if (pass1_samples < 250000 || pass2_samples < 250000)
  {
    ROS_ERROR_STREAM("pass1_samples and pass2_samples must each be at least 250000");
    return 2;
  }

  const auto start_wall = ros::WallTime::now();
  robot_model_loader::RobotModelLoader loader("robot_description");
  const moveit::core::RobotModelPtr model = loader.getModel();
  if (!model)
    return 1;
  const moveit::core::JointModelGroup* jmg = model->getJointModelGroup(kGroupName);
  const moveit::core::LinkModel* tcp_link = model->getLinkModel(kTcpLink);
  if (!jmg || !tcp_link)
    return 1;

  planning_scene_monitor::PlanningSceneMonitorPtr psm =
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  psm->startSceneMonitor();
  psm->startWorldGeometryMonitor();
  psm->startStateMonitor();
  psm->requestPlanningSceneState();
  ros::Duration(1.0).sleep();

  planning_scene_monitor::LockedPlanningSceneRO scene(psm);
  moveit::core::RobotState current_state = scene->getCurrentState();
  current_state.update();
  const std::vector<std::string> joint_names = jmg->getVariableNames();
  std::vector<double> current_joints;
  current_state.copyJointGroupPositions(jmg, current_joints);

  std::vector<double> center(std::begin(kCenterJoints), std::end(kCenterJoints));
  std::vector<Bounds> bounds;
  for (const auto& name : joint_names)
  {
    const moveit::core::VariableBounds& b = model->getVariableBounds(name);
    bounds.push_back({ b.min_position_, b.max_position_ });
  }

  std::ostringstream context;
  context << "{";
  context << q("planning_frame") << ":" << q(model->getModelFrame()) << ",";
  context << q("tcp_link") << ":" << q(kTcpLink) << ",";
  context << q("joint_names") << ":[";
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    if (i)
      context << ",";
    context << q(joint_names[i]);
  }
  context << "],";
  context << q("current_joints") << ":" << vecJson(current_joints) << ",";
  context << q("local_center_joints") << ":" << vecJson(center) << ",";
  context << q("pass1_radius_rad") << ":" << num(0.10) << ",";
  context << q("pass2_radius_rad") << ":" << num(0.05);
  context << "}";
  printJson("PIPER_LOCAL_FK_CONTEXT_JSON", context.str());

  std::vector<std::vector<double>> samples = structuredLocalSamples(center, bounds, 0.10);
  for (int i = 1; i <= pass1_samples; ++i)
    samples.push_back(localHaltonSample(i, center, bounds, 0.10));
  const std::size_t pass2_start = samples.size();
  std::vector<std::vector<double>> pass2_structured = structuredLocalSamples(center, bounds, 0.05);
  samples.insert(samples.end(), pass2_structured.begin(), pass2_structured.end());
  for (int i = 1; i <= pass2_samples; ++i)
    samples.push_back(localHaltonSample(i, center, bounds, 0.05));

  std::vector<Candidate> candidates;
  candidates.reserve(samples.size() / 2);
  std::size_t collision_checked = 0;
  std::size_t collision_free = 0;
  std::size_t safe20 = 0;
  std::size_t safe25 = 0;
  std::size_t exploratory30 = 0;
  bool exact5 = false;
  double min_angle = std::numeric_limits<double>::infinity();

  collision_detection::CollisionRequest collision_request;
  collision_request.group_name = kGroupName;
  collision_request.contacts = false;
  moveit::core::RobotState sample_state(current_state);
  for (std::size_t i = 0; i < samples.size(); ++i)
  {
    sample_state.setJointGroupPositions(jmg, samples[i]);
    sample_state.enforceBounds(jmg);
    sample_state.update();
    const Eigen::Isometry3d& tf = sample_state.getGlobalLinkTransform(kTcpLink);
    const Eigen::Vector3d xyz = tf.translation();
    const Eigen::Matrix3d rot = tf.rotation();
    const Eigen::Vector3d approach = rot * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d closing = rot * Eigen::Vector3d::UnitY();
    const double angle = angleToDownDeg(approach);
    const double margin = minJointMargin(samples[i], bounds);
    if (!inWorkspace(xyz) || margin < kMinRecommendMargin || angle > 30.0)
      continue;
    min_angle = std::min(min_angle, angle);
    exact5 = exact5 || angle <= 5.0;
    ++collision_checked;
    collision_detection::CollisionResult collision_result;
    scene->checkCollision(collision_request, collision_result, sample_state);
    if (collision_result.collision)
      continue;
    ++collision_free;
    Candidate c;
    c.joints = samples[i];
    c.xyz = xyz;
    c.quat = Eigen::Quaterniond(rot).normalized();
    c.approach = approach;
    c.closing = closing;
    c.approach_angle_deg = angle;
    c.closing_yaw_deg = std::atan2(closing.y(), closing.x()) * 180.0 / M_PI;
    c.min_margin = margin;
    c.joint_distance = jointDistance(samples[i], current_joints);
    c.collision_free = true;
    c.pass = i >= pass2_start ? 2 : 1;
    candidates.push_back(c);
    if (angle <= 20.0)
      ++safe20;
    if (angle <= 25.0)
      ++safe25;
    if (angle <= 30.0)
      ++exploratory30;
  }

  std::vector<Cluster> clusters20 = clusterCandidates(candidates, 20.0, "preferred_20_deg");
  std::vector<Cluster> clusters25 = clusterCandidates(candidates, 25.0, "acceptable_25_deg");
  std::vector<Cluster> clusters30 = clusterCandidates(candidates, 30.0, "exploratory_30_deg");
  std::vector<Cluster> all_clusters = clusters20;
  all_clusters.insert(all_clusters.end(), clusters25.begin(), clusters25.end());
  all_clusters.insert(all_clusters.end(), clusters30.begin(), clusters30.end());
  std::sort(all_clusters.begin(), all_clusters.end(), [](const Cluster& a, const Cluster& b) {
    return std::make_tuple(a.angle_limit, a.angle_min, -a.min_margin, -static_cast<int>(a.members.size()),
                           a.representative ? a.representative->joint_distance : 999.0) <
           std::make_tuple(b.angle_limit, b.angle_min, -b.min_margin, -static_cast<int>(b.members.size()),
                           b.representative ? b.representative->joint_distance : 999.0);
  });

  const double elapsed = (ros::WallTime::now() - start_wall).toSec();
  std::ostringstream summary;
  summary << "{";
  summary << q("sample_count") << ":" << samples.size() << ",";
  summary << q("collision_checked_count") << ":" << collision_checked << ",";
  summary << q("collision_free_candidate_count") << ":" << collision_free << ",";
  summary << q("safe_20_deg_margin_0_10_count") << ":" << safe20 << ",";
  summary << q("safe_25_deg_margin_0_10_count") << ":" << safe25 << ",";
  summary << q("exploratory_30_deg_count") << ":" << exploratory30 << ",";
  summary << q("exact_5_deg_tabletop_exists") << ":" << boolJson(exact5) << ",";
  summary << q("minimum_local_approach_angle_deg") << ":" << num(min_angle) << ",";
  summary << q("preferred_20_cluster_count") << ":" << clusters20.size() << ",";
  summary << q("acceptable_25_cluster_count") << ":" << clusters25.size() << ",";
  summary << q("exploratory_30_cluster_count") << ":" << clusters30.size() << ",";
  summary << q("elapsed_s") << ":" << num(elapsed) << ",";
  summary << q("samples_per_second") << ":" << num(samples.size() / std::max(1e-9, elapsed));
  summary << "}";
  printJson("PIPER_LOCAL_FK_SUMMARY_JSON", summary.str());

  int rank = 1;
  for (const auto& cluster : all_clusters)
    printJson("PIPER_LOCAL_FK_CLUSTER_JSON", clusterJson(cluster, rank++));

  ros::ServiceClient ik_client = nh.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
  ik_client.waitForExistence(ros::Duration(5.0));
  moveit::planning_interface::MoveGroupInterface move_group(kGroupName);
  move_group.setEndEffectorLink(kTcpLink);
  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(5);
  move_group.setGoalPositionTolerance(0.005);
  move_group.setGoalOrientationTolerance(5.0 * M_PI / 180.0);

  std::vector<std::pair<const Cluster*, Validation>> validated;
  const std::size_t validation_count = std::min<std::size_t>(10, all_clusters.size());
  for (std::size_t i = 0; i < validation_count; ++i)
  {
    const Cluster& cluster = all_clusters[i];
    const Candidate& rep = *cluster.representative;
    Validation validation =
        validateCandidate(rep, current_state, model, jmg, joint_names, scene, ik_client, move_group);
    validated.push_back(std::make_pair(&cluster, validation));
    std::ostringstream row;
    row << "{";
    row << q("rank") << ":" << (i + 1) << ",";
    row << q("cluster") << ":" << clusterJson(cluster, static_cast<int>(i + 1)) << ",";
    row << q("validation") << ":" << validationJson(validation);
    row << "}";
    printJson("PIPER_LOCAL_VALIDATION_JSON", row.str());
  }

  const Cluster* best_cluster = nullptr;
  const Validation* best_validation = nullptr;
  for (const auto& item : validated)
  {
    const Validation& validation = item.second;
    if (validation.ik_success && validation.state_valid && validation.transit_success && validation.descend_all &&
        validation.lift_all)
    {
      best_cluster = item.first;
      best_validation = &item.second;
      break;
    }
  }

  if (best_cluster && best_validation)
  {
    writeYaml(output_yaml, *best_cluster, *best_cluster->representative, *best_validation, source_commit);
    std::ostringstream safe;
    safe << "{";
    safe << q("output_yaml") << ":" << q(output_yaml) << ",";
    safe << q("recommended_tcp_hover_region") << ":{";
    safe << q("center") << ":" << vecJson(best_cluster->center) << ",";
    safe << q("min") << ":" << vecJson(best_cluster->min_xyz) << ",";
    safe << q("max") << ":" << vecJson(best_cluster->max_xyz) << "},";
    safe << q("corresponding_cup_surface_regions") << ":{";
    bool first = true;
    for (double hover : { 0.05, 0.07, 0.10 })
    {
      if (!first)
        safe << ",";
      first = false;
      Eigen::Vector3d center = best_cluster->center;
      Eigen::Vector3d minv = best_cluster->min_xyz;
      Eigen::Vector3d maxv = best_cluster->max_xyz;
      center.z() -= hover;
      minv.z() -= hover;
      maxv.z() -= hover;
      safe << q(num(hover)) << ":{" << q("center") << ":" << vecJson(center) << "," << q("min") << ":"
           << vecJson(minv) << "," << q("max") << ":" << vecJson(maxv) << "}";
    }
    safe << "},";
    safe << q("representative_tcp_quaternion") << ":" << quatJson(best_cluster->representative->quat) << ",";
    safe << q("representative_joint_state") << ":" << vecJson(best_cluster->representative->joints) << ",";
    safe << q("minimum_joint_limit_margin_rad") << ":" << num(best_cluster->min_margin) << ",";
    safe << q("cluster_size") << ":" << best_cluster->members.size() << ",";
    safe << q("plan_validation") << ":" << validationJson(*best_validation);
    safe << "}";
    printJson("PIPER_VALIDATED_LOCAL_GRASP_REGION_JSON", safe.str());
  }
  else
  {
    printJson("PIPER_VALIDATED_LOCAL_GRASP_REGION_JSON",
              std::string("{") + q("recommended_tcp_hover_region") + ":null," + q("reason") +
                  ":" + q("no local cluster passed IK, state-validity, transit, descend, and lift validation") + "}");
  }

  std::cout << "NO_HARDWARE_COMMANDS_ISSUED" << std::endl;
  return 0;
}
