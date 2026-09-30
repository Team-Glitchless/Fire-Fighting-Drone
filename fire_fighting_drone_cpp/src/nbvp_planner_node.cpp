// ROS 2 (Jazzy) node running the RRT-based Next-Best-View exploration
// planner. Replaces the ROS 1 `treestuff.cpp` 10-ray greedy heuristic with a
// proper RRT tree grown over the live OctoMap, selecting the branch that
// maximizes distance-discounted information gain (see rrt_nbvp.hpp).

#include <chrono>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/time.h>

#include "fire_fighting_drone_cpp/rrt_nbvp.hpp"

using namespace std::chrono_literals;

namespace fire_fighting_drone_cpp
{

class NbvpPlannerNode : public rclcpp::Node
{
public:
  NbvpPlannerNode()
  : Node("nbvp_planner")
  {
    RrtNbvpParams params;
    params.bound_min = octomap::point3d(
      static_cast<float>(declare_parameter("bound_min_x", -5.0)),
      static_cast<float>(declare_parameter("bound_min_y", -15.0)),
      static_cast<float>(declare_parameter("bound_min_z", 0.3)));
    params.bound_max = octomap::point3d(
      static_cast<float>(declare_parameter("bound_max_x", 5.0)),
      static_cast<float>(declare_parameter("bound_max_y", 15.0)),
      static_cast<float>(declare_parameter("bound_max_z", 5.0)));
    params.max_iterations = declare_parameter("max_iterations", 300);
    params.extension_range = declare_parameter("extension_range", 1.5);
    params.collision_check_resolution = declare_parameter("collision_check_resolution", 0.1);
    params.sensor_range = declare_parameter("sensor_range", 5.0);
    params.horizontal_fov = declare_parameter("horizontal_fov", 1.2217);
    params.vertical_fov = declare_parameter("vertical_fov", 0.7854);
    params.horizontal_rays = declare_parameter("horizontal_rays", 5);
    params.vertical_rays = declare_parameter("vertical_rays", 3);
    params.yaw_samples = declare_parameter("yaw_samples", 12);
    params.lambda = declare_parameter("lambda", 0.5);
    params.gain_threshold = declare_parameter("gain_threshold", 5.0);
    params.min_node_separation = declare_parameter("min_node_separation", 0.3);
    replan_period_ = declare_parameter("replan_period_sec", 2.0);

    planner_ = std::make_unique<RrtNbvp>(params);

    octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      "/octomap_binary", rclcpp::QoS(1),
      std::bind(&NbvpPlannerNode::octomapCallback, this, std::placeholders::_1));
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
      std::bind(&NbvpPlannerNode::poseCallback, this, std::placeholders::_1));

    next_goal_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/next_goal", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/nbvp_best_branch", 10);
    complete_pub_ = create_publisher<std_msgs::msg::Bool>("/exploration_complete", 10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(replan_period_),
      std::bind(&NbvpPlannerNode::plan, this));

    RCLCPP_INFO(get_logger(), "NBVP planner initialized");
  }

private:
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::unique_ptr<octomap::AbstractOcTree> tree(octomap_msgs::binaryMsgToMap(*msg));
    if (!tree) {
      RCLCPP_WARN(get_logger(), "Failed to deserialize incoming OctoMap message");
      return;
    }
    octree_ = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree *>(tree.release()));
    have_octomap_ = (octree_ != nullptr);
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_position_ = octomap::point3d(
      static_cast<float>(msg->pose.position.x),
      static_cast<float>(msg->pose.position.y),
      static_cast<float>(msg->pose.position.z));

    tf2::Quaternion q(
      msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    current_yaw_ = yaw;
    have_pose_ = true;
  }

  void plan()
  {
    std::shared_ptr<octomap::OcTree> octree_copy;
    octomap::point3d root_position;
    double root_yaw;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_octomap_ || !have_pose_ || exploration_complete_) {
        return;
      }
      octree_copy = octree_;
      root_position = current_position_;
      root_yaw = current_yaw_;
    }

    const RrtNbvpResult result = planner_->plan(octree_copy, root_position, root_yaw);

    RCLCPP_INFO(
      get_logger(), "NBVP: best utility %.3f, best gain %.3f, branch length %zu",
      result.best_utility, result.best_gain, result.path.size());

    publishPath(result.path);

    if (result.exploration_complete) {
      std_msgs::msg::Bool done_msg;
      done_msg.data = true;
      complete_pub_->publish(done_msg);
      exploration_complete_ = true;
      RCLCPP_INFO(get_logger(), "Exploration complete: no further information gain available");
      return;
    }

    if (result.path.size() < 2) {
      return; // No viable extension found this cycle; try again next timer tick.
    }

    // First edge of the best branch is the receding-horizon next viewpoint.
    const RrtNode & next = result.path[1];
    geometry_msgs::msg::PointStamped goal;
    goal.header.stamp = now();
    goal.header.frame_id = "world";
    goal.point.x = next.position.x();
    goal.point.y = next.position.y();
    goal.point.z = next.position.z();
    next_goal_pub_->publish(goal);
  }

  void publishPath(const std::vector<RrtNode> & branch)
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now();
    path_msg.header.frame_id = "world";
    for (const auto & node : branch) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = node.position.x();
      pose.pose.position.y = node.position.y();
      pose.pose.position.z = node.position.z();
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, node.yaw);
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();
      path_msg.poses.push_back(pose);
    }
    path_pub_->publish(path_msg);
  }

  std::unique_ptr<RrtNbvp> planner_;
  std::mutex mutex_;
  std::shared_ptr<octomap::OcTree> octree_;
  bool have_octomap_{false};
  bool have_pose_{false};
  bool exploration_complete_{false};
  octomap::point3d current_position_{0.0F, 0.0F, 0.0F};
  double current_yaw_{0.0};
  double replan_period_{2.0};

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr next_goal_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr complete_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace fire_fighting_drone_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fire_fighting_drone_cpp::NbvpPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
