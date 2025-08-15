#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <iostream>
#include <functional>

// forward declare your GraphSearch and TrajectoryVector types
// #include <adore/apps/graph_search.hpp>  // adjust include to match your project
// using adore::apps::GraphSearch; etc.

namespace adore
{
namespace if_ROS
{

class GraphSearchNode : public rclcpp::Node
{
public:
  GraphSearchNode()
  : Node("graph_search_node")
  {
    RCLCPP_INFO(this->get_logger(), "GraphSearchNode constructor called");

    // QoS 10 (history keep last 10)
    rclcpp::QoS qos(10);

    // map subscription
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", qos,
      std::bind(&GraphSearchNode::receive_map_data, this, std::placeholders::_1));

    // pose topic (PoseWithCovarianceStamped)
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "pose", qos,
      std::bind(&GraphSearchNode::receiveStartPose, this, std::placeholders::_1));

    // goalpose topic (PoseStamped)
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "goalpose", qos,
      std::bind(&GraphSearchNode::receiveEndPose, this, std::placeholders::_1));

    // path publisher (nav_msgs/Path)
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("Path", qos);

    first_set_ = false;
    validStart = validEnd = false;

    RCLCPP_INFO(this->get_logger(), "init graph search node test");
  }

  ~GraphSearchNode()
  {
    if (gs_) {
      delete gs_;
      gs_ = nullptr;
    }
  }

private:
  // subscriptions / publisher
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

  // Graph search pointer and path container (keep your types)
  adore::apps::GraphSearch* gs_ = nullptr;
  TrajectoryVector path;

  // flags
  bool first_set_;
  bool validStart;
  bool validEnd;

  // Map callback (ROS2 message type)
  void receive_map_data(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "receive map data");
    if (!first_set_)
    {
      RCLCPP_INFO(this->get_logger(), "receive map data first time test");
      first_set_ = true;

      int test = 1;
      // Construct your GraphSearch (assumes it accepts this msg type)
      gs_ = new adore::apps::GraphSearch(msg, test, static_cast<uint32_t>(msg->info.height),
                                         static_cast<uint32_t>(msg->info.width));

      // If GraphSearch::update is a method to be periodically called, you can call it directly here:
      gs_->update();

      path = gs_->path;

      int i = 0;
      RCLCPP_INFO(this->get_logger(), "before publish path");
      for (const auto &point : path)
      {
        RCLCPP_INFO(this->get_logger(), "a star Point : %f %f", point.x, point.y);
        if (++i > 10) break;
      }

      publishPath();
    }
  }

  // Start pose callback (PoseWithCovarianceStamped)
  void receiveStartPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    // Extract yaw if needed from msg->pose.pose.orientation, or use msg->pose.pose.position
    // Here we simply mark validStart and could store the pose for use in graph search
    RCLCPP_INFO(this->get_logger(), "received start pose");
    validStart = true;
    // store start pose if you need: start_pose_ = msg->pose.pose;
  }

  // Goal pose callback (PoseStamped)
  void receiveEndPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "received goal pose");
    validEnd = true;
    // store goal pose if you need: goal_pose_ = msg->pose;
  }

  void publishPath()
  {
    nav_msgs::msg::Path msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map"; // or your desired frame

    RCLCPP_INFO(this->get_logger(), "publish path");

    for (const auto &point : path)
    {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.stamp = this->now();
      pose_stamped.header.frame_id = "map";
      pose_stamped.pose.position.x = point.x;
      pose_stamped.pose.position.y = point.y;
      pose_stamped.pose.position.z = 0.0;

      // Convert yaw (psi) into quaternion
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, point.psi);
      pose_stamped.pose.orientation.x = q.x();
      pose_stamped.pose.orientation.y = q.y();
      pose_stamped.pose.orientation.z = q.z();
      pose_stamped.pose.orientation.w = q.w();

      RCLCPP_DEBUG(this->get_logger(), "GraphSearch Test Point : %f %f", point.x, point.y);

      msg.poses.push_back(pose_stamped);
    }

    path_publisher_->publish(msg);
  }

  // you may want to store start/goal poses
  // geometry_msgs::msg::Pose start_pose_;
  // geometry_msgs::msg::Pose goal_pose_;
};

}  // namespace if_ROS
}  // namespace adore

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<adore::if_ROS::GraphSearchNode>());
  rclcpp::shutdown();
  return 0;
}
