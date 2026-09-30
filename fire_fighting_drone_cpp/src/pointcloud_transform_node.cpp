// ROS 2 (Jazzy) port of the ROS 1 `transform.cpp` node. Rotates the incoming
// ORB-SLAM2 map-point cloud from the camera optical frame into a body/NEU
// convention and broadcasts the composed world->camera_transformed tf, so
// downstream mapping (octomap_server) can consume points already expressed
// in a consistent world frame.

#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace fire_fighting_drone_cpp
{

class PointCloudTransformNode : public rclcpp::Node
{
public:
  PointCloudTransformNode()
  : Node("pcl_transform"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    camera_frame_ = declare_parameter<std::string>("camera_frame", "camera");
    output_frame_ = declare_parameter<std::string>("output_frame", "camera_transformed");

    // Fixed rotation: camera optical frame (X-right, Y-down, Z-forward) into
    // a body/NEU-style convention, matching the original ROS 1 node.
    camera_to_body_.setBasis(tf2::Matrix3x3(0, 0, 1, -1, 0, 0, 0, -1, 0));
    camera_to_body_.setOrigin(tf2::Vector3(0, 0, 0));

    pcl_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("pcl_out", rclcpp::QoS(100));
    pcl_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/orb_slam2_rgbd/map_points", rclcpp::QoS(100),
      std::bind(&PointCloudTransformNode::pclCallback, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

private:
  void pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped world_to_camera;
    try {
      world_to_camera = tf_buffer_.lookupTransform(
        world_frame_, camera_frame_, tf2::TimePointZero, tf2::durationFromSec(1.0));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(), "%s", ex.what());
      return;
    }

    tf2::Transform world_to_camera_tf;
    tf2::fromMsg(world_to_camera.transform, world_to_camera_tf);

    // Mirrors the original sandwich transform: camera_to_body * T * camera_to_body^-1.
    const tf2::Transform camera_transformed_tf =
      camera_to_body_ * world_to_camera_tf * camera_to_body_.inverse();

    const tf2::Vector3 & origin = camera_transformed_tf.getOrigin();
    if (std::isnan(origin.x()) || std::isnan(origin.y()) || std::isnan(origin.z())) {
      RCLCPP_ERROR(get_logger(), "Computed camera_transformed transform is NaN, dropping cloud");
      return;
    }

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = now();
    transform_stamped.header.frame_id = world_frame_;
    transform_stamped.child_frame_id = output_frame_;
    transform_stamped.transform = tf2::toMsg(camera_transformed_tf);
    tf_broadcaster_->sendTransform(transform_stamped);

    // Rotate the incoming cloud from the optical frame into the body/NEU frame.
    geometry_msgs::msg::TransformStamped cloud_transform;
    cloud_transform.transform = tf2::toMsg(camera_to_body_);
    sensor_msgs::msg::PointCloud2 out;
    tf2::doTransform(*msg, out, cloud_transform);
    out.header.frame_id = output_frame_;
    out.header.stamp = msg->header.stamp;
    pcl_pub_->publish(out);
  }

  tf2::Transform camera_to_body_;
  std::string world_frame_;
  std::string camera_frame_;
  std::string output_frame_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
};

}  // namespace fire_fighting_drone_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fire_fighting_drone_cpp::PointCloudTransformNode>());
  rclcpp::shutdown();
  return 0;
}
