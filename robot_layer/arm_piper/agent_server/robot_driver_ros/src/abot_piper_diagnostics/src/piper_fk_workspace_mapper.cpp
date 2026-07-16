#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <ros/ros.h>

namespace
{
constexpr const char* kGroupName = "arm";
constexpr const char* kTcpLink = "gripper_tcp";
constexpr double kCupSurfaceX = 0.2288215;
constexpr double kCupSurfaceY = -0.0172091;
constexpr double kCupSurfaceZ = 0.0897315;
constexpr double kClusterRadius = 0.03;
constexpr double kClusterCell = 0.03;
constexpr double kMinRecommendMargin = 0.10;
constexpr double kPreferredMargin = 0.15;

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
  double approach_angle_deg;
  double min_margin;
  double joint_distance;
  bool workspace;
  bool collision_free;
};

struct Cluster
{
  std::vector<const Candidate*> members;
  Eigen::Vector3d center;
  Eigen::Vector3d min_xyz;
  Eigen::Vector3d max_xyz;
  double angle_min;
  double angle_median;
  double angle_max;
  double min_margin;
  double yaw_min;
  double yaw_max;
  const Candidate* representative;
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

std::string quatJson(const Eigen::Quaterniond& qv)
{
  return vecJson(std::vector<double>{ qv.x(), qv.y(), qv.z(), qv.w() });
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

bool inTabletopWorkspace(const Eigen::Vector3d& xyz)
{
  return xyz.x() >= 0.15 && xyz.x() <= 0.55 && xyz.y() >= -0.30 && xyz.y() <= 0.30 && xyz.z() >= 0.12 &&
         xyz.z() <= 0.35;
}

bool inCollisionFilterWorkspace(const Eigen::Vector3d& xyz)
{
  return xyz.x() >= 0.15 && xyz.x() <= 0.55 && xyz.y() >= -0.30 && xyz.y() <= 0.30 && xyz.z() >= 0.12 &&
         xyz.z() <= 0.35;
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
  out << q("minimum_joint_limit_margin_rad") << ":" << num(c.min_margin) << ",";
  out << q("joint_distance_from_current") << ":" << num(c.joint_distance) << ",";
  out << q("inside_tabletop_workspace") << ":" << boolJson(c.workspace) << ",";
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

std::vector<double> haltonSample(int index, const std::vector<Bounds>& bounds)
{
  static const int primes[] = { 2, 3, 5, 7, 11, 13 };
  std::vector<double> joints;
  joints.reserve(bounds.size());
  for (std::size_t i = 0; i < bounds.size(); ++i)
  {
    const double t = halton(index, primes[i]);
    joints.push_back(bounds[i].lower + t * (bounds[i].upper - bounds[i].lower));
  }
  return joints;
}

std::vector<std::vector<double>> structuredSamples(const std::vector<double>& current,
                                                   const std::vector<Bounds>& bounds)
{
  std::vector<std::vector<double>> samples;
  samples.push_back(current);
  std::vector<double> midpoint;
  midpoint.reserve(bounds.size());
  for (const auto& b : bounds)
    midpoint.push_back((b.lower + b.upper) * 0.5);
  samples.push_back(midpoint);

  for (std::size_t joint = 0; joint < bounds.size(); ++joint)
  {
    for (double f : { 0.2, 0.5, 0.8 })
    {
      std::vector<double> values = midpoint;
      values[joint] = bounds[joint].lower + f * (bounds[joint].upper - bounds[joint].lower);
      samples.push_back(values);
    }
  }
  for (double j1 : { 0.25, 0.5, 0.75 })
  {
    for (double j2 : { 0.25, 0.5, 0.75 })
    {
      std::vector<double> values = midpoint;
      values[0] = bounds[0].lower + j1 * (bounds[0].upper - bounds[0].lower);
      values[1] = bounds[1].lower + j2 * (bounds[1].upper - bounds[1].lower);
      samples.push_back(values);
    }
  }
  return samples;
}

std::string cellKey(const Eigen::Vector3d& xyz)
{
  const int ix = static_cast<int>(std::floor(xyz.x() / kClusterCell));
  const int iy = static_cast<int>(std::floor(xyz.y() / kClusterCell));
  const int iz = static_cast<int>(std::floor(xyz.z() / kClusterCell));
  return std::to_string(ix) + ":" + std::to_string(iy) + ":" + std::to_string(iz);
}

std::vector<Cluster> clusterCandidates(const std::vector<Candidate>& candidates,
                                       const std::vector<double>& current_joints)
{
  std::map<std::string, std::vector<const Candidate*>> bins;
  for (const auto& c : candidates)
  {
    if (!c.collision_free || !c.workspace || c.approach_angle_deg > 30.0 || c.min_margin < kMinRecommendMargin)
      continue;
    bins[cellKey(c.xyz)].push_back(&c);
  }

  std::vector<Cluster> clusters;
  for (const auto& kv : bins)
  {
    if (kv.second.size() < 20)
      continue;
    Cluster cluster;
    cluster.members = kv.second;
    cluster.center = Eigen::Vector3d::Zero();
    cluster.min_xyz = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    cluster.max_xyz = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
    cluster.angle_min = std::numeric_limits<double>::infinity();
    cluster.angle_max = -std::numeric_limits<double>::infinity();
    cluster.min_margin = std::numeric_limits<double>::infinity();
    cluster.yaw_min = std::numeric_limits<double>::infinity();
    cluster.yaw_max = -std::numeric_limits<double>::infinity();
    std::vector<double> angles;
    const Candidate* representative = nullptr;
    for (const Candidate* c : kv.second)
    {
      cluster.center += c->xyz;
      cluster.min_xyz = cluster.min_xyz.cwiseMin(c->xyz);
      cluster.max_xyz = cluster.max_xyz.cwiseMax(c->xyz);
      cluster.angle_min = std::min(cluster.angle_min, c->approach_angle_deg);
      cluster.angle_max = std::max(cluster.angle_max, c->approach_angle_deg);
      cluster.min_margin = std::min(cluster.min_margin, c->min_margin);
      const double yaw = std::atan2(c->closing.y(), c->closing.x()) * 180.0 / M_PI;
      cluster.yaw_min = std::min(cluster.yaw_min, yaw);
      cluster.yaw_max = std::max(cluster.yaw_max, yaw);
      angles.push_back(c->approach_angle_deg);
      if (!representative || std::make_tuple(c->approach_angle_deg, -c->min_margin, c->joint_distance) <
                                 std::make_tuple(representative->approach_angle_deg, -representative->min_margin,
                                                 representative->joint_distance))
        representative = c;
    }
    cluster.center /= static_cast<double>(kv.second.size());
    std::sort(angles.begin(), angles.end());
    cluster.angle_median = angles[angles.size() / 2];
    cluster.representative = representative;
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

std::vector<double> solutionJoints(const moveit_msgs::RobotState& state, const std::vector<std::string>& joint_names)
{
  std::map<std::string, double> values;
  for (std::size_t i = 0; i < state.joint_state.name.size() && i < state.joint_state.position.size(); ++i)
    values[state.joint_state.name[i]] = state.joint_state.position[i];
  std::vector<double> out;
  for (const auto& name : joint_names)
  {
    auto it = values.find(name);
    if (it != values.end())
      out.push_back(it->second);
  }
  return out;
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "piper_fk_workspace_mapper");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle nh("~");

  int halton_samples = 100000;
  nh.param("halton_samples", halton_samples, halton_samples);
  if (halton_samples < 100000)
  {
    ROS_ERROR_STREAM("halton_samples must be at least 100000");
    return 2;
  }

  const auto start_wall = ros::WallTime::now();
  robot_model_loader::RobotModelLoader loader("robot_description");
  const moveit::core::RobotModelPtr model = loader.getModel();
  if (!model)
  {
    ROS_ERROR_STREAM("Failed to load robot model");
    return 1;
  }
  const moveit::core::JointModelGroup* jmg = model->getJointModelGroup(kGroupName);
  if (!jmg)
  {
    ROS_ERROR_STREAM("Missing JointModelGroup " << kGroupName);
    return 1;
  }
  const moveit::core::LinkModel* tcp_link = model->getLinkModel(kTcpLink);
  if (!tcp_link)
  {
    ROS_ERROR_STREAM("Missing TCP link " << kTcpLink);
    return 1;
  }

  planning_scene_monitor::PlanningSceneMonitorPtr psm =
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  if (!psm->getPlanningScene())
  {
    ROS_ERROR_STREAM("PlanningSceneMonitor has no planning scene");
    return 1;
  }
  psm->startSceneMonitor();
  psm->startWorldGeometryMonitor();
  psm->startStateMonitor();
  psm->requestPlanningSceneState();
  ros::Duration(1.0).sleep();

  planning_scene_monitor::LockedPlanningSceneRO scene(psm);
  moveit::core::RobotState current_state = scene->getCurrentState();
  current_state.update();
  std::vector<std::string> joint_names = jmg->getVariableNames();
  std::vector<double> current_joints;
  current_state.copyJointGroupPositions(jmg, current_joints);

  std::vector<Bounds> bounds;
  for (const auto& name : joint_names)
  {
    const moveit::core::VariableBounds& b = model->getVariableBounds(name);
    bounds.push_back({ b.min_position_, b.max_position_ });
  }

  std::ostringstream context;
  context << "{";
  context << q("halton_samples") << ":" << halton_samples << ",";
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
  context << q("current_joints") << ":" << vecJson(current_joints);
  context << "}";
  printJson("PIPER_FK_WORKSPACE_CONTEXT_JSON", context.str());

  std::vector<std::vector<double>> samples = structuredSamples(current_joints, bounds);
  samples.reserve(samples.size() + halton_samples);
  for (int i = 1; i <= halton_samples; ++i)
    samples.push_back(haltonSample(i, bounds));

  std::vector<Candidate> collision_free_candidates;
  collision_free_candidates.reserve(samples.size() / 20);
  std::size_t filter_candidate_count = 0;
  std::size_t collision_checked_count = 0;
  std::size_t collision_free_count = 0;
  std::size_t safe20_count = 0;
  std::size_t safe25_count = 0;
  std::size_t exploratory30_count = 0;
  bool exact5_tabletop_exists = false;
  double min_tabletop_angle = std::numeric_limits<double>::infinity();

  moveit::core::RobotState sample_state(current_state);
  collision_detection::CollisionRequest collision_request;
  collision_request.group_name = kGroupName;
  collision_request.contacts = false;

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
    const bool tabletop = inTabletopWorkspace(xyz);
    if (tabletop)
    {
      min_tabletop_angle = std::min(min_tabletop_angle, angle);
      if (angle <= 5.0)
        exact5_tabletop_exists = true;
    }

    if (inCollisionFilterWorkspace(xyz) && angle <= 35.0)
    {
      ++filter_candidate_count;
      collision_detection::CollisionResult collision_result;
      scene->checkCollision(collision_request, collision_result, sample_state);
      ++collision_checked_count;
      if (!collision_result.collision)
      {
        ++collision_free_count;
        Candidate c;
        c.joints = samples[i];
        c.xyz = xyz;
        c.quat = Eigen::Quaterniond(rot).normalized();
        c.approach = approach;
        c.closing = closing;
        c.approach_angle_deg = angle;
        c.min_margin = margin;
        c.joint_distance = jointDistance(samples[i], current_joints);
        c.workspace = tabletop;
        c.collision_free = true;
        collision_free_candidates.push_back(c);
        if (margin >= kPreferredMargin && angle <= 20.0)
          ++safe20_count;
        if (margin >= kPreferredMargin && angle <= 25.0)
          ++safe25_count;
        if (angle <= 30.0)
          ++exploratory30_count;
      }
    }
  }

  std::vector<Cluster> clusters = clusterCandidates(collision_free_candidates, current_joints);

  const double elapsed = (ros::WallTime::now() - start_wall).toSec();
  std::ostringstream summary;
  summary << "{";
  summary << q("sample_count") << ":" << samples.size() << ",";
  summary << q("collision_filter_candidate_count") << ":" << filter_candidate_count << ",";
  summary << q("collision_checked_count") << ":" << collision_checked_count << ",";
  summary << q("collision_free_candidate_count") << ":" << collision_free_count << ",";
  summary << q("safe_20_deg_margin_0_15_count") << ":" << safe20_count << ",";
  summary << q("safe_25_deg_margin_0_15_count") << ":" << safe25_count << ",";
  summary << q("exploratory_30_deg_count") << ":" << exploratory30_count << ",";
  summary << q("exact_5_deg_tabletop_exists") << ":" << boolJson(exact5_tabletop_exists) << ",";
  summary << q("minimum_tabletop_approach_angle_deg") << ":" << num(min_tabletop_angle) << ",";
  summary << q("dense_cluster_count") << ":" << clusters.size() << ",";
  summary << q("elapsed_s") << ":" << num(elapsed) << ",";
  summary << q("samples_per_second") << ":" << num(samples.size() / std::max(1e-9, elapsed));
  summary << "}";
  printJson("PIPER_FK_WORKSPACE_SUMMARY_JSON", summary.str());

  for (std::size_t i = 0; i < clusters.size(); ++i)
    printJson("PIPER_FK_WORKSPACE_CLUSTER_JSON", clusterJson(clusters[i], static_cast<int>(i + 1)));

  std::vector<const Candidate*> ranked_candidates;
  for (const auto& candidate : collision_free_candidates)
  {
    if (candidate.workspace && candidate.approach_angle_deg <= 30.0 && candidate.min_margin >= kMinRecommendMargin)
      ranked_candidates.push_back(&candidate);
  }
  std::sort(ranked_candidates.begin(), ranked_candidates.end(), [](const Candidate* a, const Candidate* b) {
    return std::make_tuple(a->approach_angle_deg, -a->min_margin, a->joint_distance) <
           std::make_tuple(b->approach_angle_deg, -b->min_margin, b->joint_distance);
  });
  for (std::size_t i = 0; i < std::min<std::size_t>(20, ranked_candidates.size()); ++i)
  {
    std::ostringstream row;
    row << "{";
    row << q("rank") << ":" << (i + 1) << ",";
    row << q("recommendation_status") << ":" << q("exploratory_only_not_dense_cluster") << ",";
    row << q("candidate") << ":" << candidateJson(*ranked_candidates[i]);
    row << "}";
    printJson("PIPER_EXPLORATORY_TCP_CANDIDATE_JSON", row.str());
  }

  ros::ServiceClient ik_client = nh.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
  ik_client.waitForExistence(ros::Duration(5.0));
  moveit::planning_interface::MoveGroupInterface move_group(kGroupName);
  move_group.setEndEffectorLink(kTcpLink);
  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(5);
  move_group.setGoalPositionTolerance(0.005);
  move_group.setGoalOrientationTolerance(5.0 * M_PI / 180.0);

  std::vector<const Candidate*> validation_candidates;
  for (const auto& cluster : clusters)
    validation_candidates.push_back(cluster.representative);
  if (validation_candidates.empty())
    validation_candidates = ranked_candidates;

  std::vector<std::string> validation_json;
  const std::size_t validation_count = std::min<std::size_t>(5, validation_candidates.size());
  for (std::size_t i = 0; i < validation_count; ++i)
  {
    const Candidate& rep = *validation_candidates[i];
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
    const bool ik_success = ik_call_ok && srv.response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS;

    bool solution_valid = false;
    if (ik_success)
    {
      moveit::core::RobotState solution_state(current_state);
      moveit::core::robotStateMsgToRobotState(srv.response.solution, solution_state);
      solution_state.update();
      collision_detection::CollisionResult validity_result;
      scene->checkCollision(collision_request, validity_result, solution_state);
      solution_valid = !validity_result.collision;
    }

    move_group.setStartStateToCurrentState();
    move_group.clearPoseTargets();
    move_group.setPoseTarget(eigenPoseToMsg(rep.xyz, rep.quat), kTcpLink);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const moveit::core::MoveItErrorCode plan_code = move_group.plan(plan);
    const std::size_t point_count = plan.trajectory_.joint_trajectory.points.size();

    std::ostringstream row;
    row << "{";
    row << q("rank") << ":" << (i + 1) << ",";
    row << q("validation_source") << ":" << q(clusters.empty() ? "exploratory_candidate_no_dense_cluster" : "dense_cluster")
        << ",";
    row << q("representative_tcp_pose") << ":" << candidateJson(rep) << ",";
    row << q("ik_validation") << ":{";
    row << q("success") << ":" << boolJson(ik_success) << ",";
    row << q("error") << ":" << q(ik_call_ok ? moveItMsgErrorName(srv.response.error_code.val) : "IK_SERVICE_CALL_FAILED")
        << ",";
    row << q("state_validity") << ":" << boolJson(solution_valid);
    row << "},";
    row << q("moveit_plan_validation") << ":{";
    row << q("success") << ":" << boolJson(plan_code == moveit::core::MoveItErrorCode::SUCCESS && point_count > 0)
        << ",";
    row << q("error") << ":" << q(moveItErrorName(plan_code)) << ",";
    row << q("trajectory_point_count") << ":" << point_count;
    row << "}";
    row << "}";
    validation_json.push_back(row.str());
    printJson("PIPER_REPRESENTATIVE_VALIDATION_JSON", row.str());
  }

  if (!clusters.empty())
  {
    const Cluster& best = clusters.front();
    const Candidate& rep = *best.representative;
    std::ostringstream safe;
    safe << "{";
    safe << q("recommended_tcp_hover_region") << ":{";
    safe << q("center") << ":" << vecJson(best.center) << ",";
    safe << q("min") << ":" << vecJson(best.min_xyz) << ",";
    safe << q("max") << ":" << vecJson(best.max_xyz);
    safe << "},";
    safe << q("corresponding_cup_surface_regions") << ":{";
    bool first_hover = true;
    for (double hover : { 0.05, 0.07, 0.10 })
    {
      if (!first_hover)
        safe << ",";
      first_hover = false;
      Eigen::Vector3d center = best.center;
      Eigen::Vector3d minv = best.min_xyz;
      Eigen::Vector3d maxv = best.max_xyz;
      center.z() -= hover;
      minv.z() -= hover;
      maxv.z() -= hover;
      safe << q(num(hover)) << ":{";
      safe << q("center") << ":" << vecJson(center) << ",";
      safe << q("min") << ":" << vecJson(minv) << ",";
      safe << q("max") << ":" << vecJson(maxv);
      safe << "}";
    }
    safe << "},";
    safe << q("allowed_approach_angle_range_deg") << ":[" << num(best.angle_min) << "," << num(best.angle_max)
         << "],";
    safe << q("representative_tcp_quaternion") << ":" << quatJson(rep.quat) << ",";
    safe << q("representative_joint_state") << ":" << vecJson(rep.joints) << ",";
    safe << q("minimum_joint_limit_margin_rad") << ":" << num(best.min_margin) << ",";
    safe << q("cluster_size") << ":" << best.members.size() << ",";
    safe << q("collision_status") << ":" << q("collision_free") << ",";
    safe << q("ik_validation") << ":"
         << (validation_json.empty() ? "{}" : validation_json.front()) << ",";
    safe << q("exact_5_deg_tabletop_exists") << ":" << boolJson(exact5_tabletop_exists);
    safe << "}";
    printJson("PIPER_SAFE_GRASP_REGION_JSON", safe.str());
  }
  else
  {
    printJson("PIPER_SAFE_GRASP_REGION_JSON",
              std::string("{") + q("recommended_tcp_hover_region") + ":null," + q("reason") +
                  ":" + q("no dense collision-free cluster with margin >= 0.10 rad and approach <= 30 deg") + "," +
                  q("exact_5_deg_tabletop_exists") + ":" + boolJson(exact5_tabletop_exists) + "}");
  }

  std::cout << "NO_HARDWARE_COMMANDS_ISSUED" << std::endl;
  return 0;
}
