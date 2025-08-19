// graph_search_node.cpp
//
// Merged GraphSearch + GraphSearchNode into a single ROS2 node file.
// Keeps your original project structure and types (OccupanyGrid, GRID, Hybrid_A_Star, TrajectorySmoothing).
//
// NOTE: This file assumes your other headers/classes (TrajectorySmoothing, Hybrid_A_Star,
//       OccupanyGrid, Node, GRID, CollisionCheckOffline, TrajectoryVector, etc.) exist
//       and are already adapted (e.g. csaps fixes applied inside TrajectorySmoothing).

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <memory>
#include <mutex>

// project headers (adjust include paths if needed)
#include "env/occupancy_grid.h"
#include "env/node.h"
#include "env/search_grid.h"
#include "algo/hybrid_A_star.h"
#include "algo/collision_check_offline.h"
#include "env/trajectory_smoothing.h"
#include "env/path.h"

namespace adore {
namespace if_ROS {

class GraphSearchNode : public rclcpp::Node
{
public:
  GraphSearchNode()
  : Node("graph_search_node"),
    grid_height_(0),
    grid_width_(0),
    vehicleLength(3.2f),
    vehicleWidth(1.0f),
    HeadingResolution(45),
    Depth(0),
    h_A_star(nullptr),
    cco(nullptr),
    smoothing(nullptr),
    first_map_received_(false),
    validStart(false),
    validEnd(false),
    iteration(1),
    avg_time_us(0)
  {
    RCLCPP_INFO(this->get_logger(), "GraphSearchNode constructor called");

    // QoS 10 (history keep last 10)
    rclcpp::QoS qos(10);

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", qos, std::bind(&GraphSearchNode::receive_map_data, this, std::placeholders::_1));

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "pose", qos, std::bind(&GraphSearchNode::receiveStartPose, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "goalpose", qos, std::bind(&GraphSearchNode::receiveEndPose, this, std::placeholders::_1));

    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("Path", qos);

    RCLCPP_INFO(this->get_logger(), "GraphSearchNode initialized");
  }

  ~GraphSearchNode()
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (h_A_star) { delete h_A_star; h_A_star = nullptr; }
    if (cco) { delete cco; cco = nullptr; }
    if (smoothing) { delete smoothing; smoothing = nullptr; }
  }

private:
  // ---------- ROS pubs / subs ----------
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

  // ---------- Graph search components ----------
  static constexpr int HeadingResolutionConst = 45; // kept for reference
  const int HeadingResolution;
  int Depth;

  uint32_t grid_height_;
  uint32_t grid_width_;

  adore::env::OccupanyGrid OG;
  adore::fun::GRID<adore::fun::Node<3, double>> NH_GRID;

  adore::fun::Hybrid_A_Star* h_A_star;
  adore::fun::CollisionCheckOffline* cco;
  adore::fun::TrajectorySmoothing* smoothing;

  // start/goal nodes
  adore::fun::Node<3, double> Start;
  adore::fun::Node<3, double> End;

  // results
  TrajectoryVector path;

  // flags / runtime
  bool first_map_received_;
  bool validStart;
  bool validEnd;

  // bookkeeping
  double avg_time_us;
  int iteration;
  double vehicleLength;
  double vehicleWidth;
  int time_us;

  // mutex to protect members modified in callbacks
  std::mutex mutex_;

  // store last poses for republishing or planning
  geometry_msgs::msg::Pose start_pose_;
  geometry_msgs::msg::Pose goal_pose_;

  // ---------- Callbacks and helpers ----------
  void receive_map_data(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);

    RCLCPP_INFO(this->get_logger(), "receive map data");

    // Initialize occupancy grid and NH_GRID once
    if (!first_map_received_)
    {
      first_map_received_ = true;
      grid_height_ = msg->info.height;
      grid_width_ = msg->info.width;

      // Initialize OG and NH_GRID using your project's API
      OG.init(msg, grid_height_, grid_width_);
      OG.resize(grid_height_, grid_width_);

      Depth = 360 / HeadingResolution;
      NH_GRID.resize(grid_height_, grid_width_, Depth);

      // Create helper objects now that we have grid sizes
      smoothing = new adore::fun::TrajectorySmoothing();
      h_A_star = new adore::fun::Hybrid_A_Star(smoothing);
      h_A_star->setSize(grid_height_, grid_width_);

      // collision checker: adjust parameters to your needs
      cco = new adore::fun::CollisionCheckOffline(2, 2, HeadingResolution, 10);

      RCLCPP_INFO(this->get_logger(), "Map initialized: width=%u height=%u Depth=%d", grid_width_, grid_height_, Depth);
    }

    // Optionally plan immediately if both start and goal already known
    if (validStart && validEnd)
    {
      planAndPublish();
    }
  }

  void receiveStartPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    double r, p, y;
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(r, p, y);

    if (OG.check_valid_position(msg->pose.pose.position.y, msg->pose.pose.position.x))
    {
      // setPosition(x, y, yaw, grid_h, grid_w, depth, heading_resolution_in_radians)
      validStart = Start.setPosition(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        y,
        grid_height_,
        grid_width_,
        Depth,
        adore::mad::CoordinateConversion::DegToRad(HeadingResolution)
      );
      start_pose_ = msg->pose.pose;

      RCLCPP_INFO(this->get_logger(), "Received valid start pose (x=%f y=%f yaw=%f) validStart=%d",
                  start_pose_.position.x, start_pose_.position.y, y, validStart);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Invalid Start Pose received");
      validStart = false;
    }

    if (first_map_received_ && validStart && validEnd) {
      planAndPublish();
    }
  }

  void receiveEndPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    double r, p, y;
    tf2::Quaternion q(
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(r, p, y);

    if (OG.check_valid_position(msg->pose.position.y, msg->pose.position.x))
    {
      validEnd = End.setPosition(
        msg->pose.position.x,
        msg->pose.position.y,
        y,
        grid_height_,
        grid_width_,
        Depth,
        adore::mad::CoordinateConversion::DegToRad(HeadingResolution)
      );
      goal_pose_ = msg->pose;
      RCLCPP_INFO(this->get_logger(), "Received valid goal pose (x=%f y=%f yaw=%f) validEnd=%d",
                  goal_pose_.position.x, goal_pose_.position.y, y, validEnd);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Invalid End Pose received");
      validEnd = false;
    }

    if (first_map_received_ && validStart && validEnd) {
      planAndPublish();
    }
  }

  void planAndPublish()
  {
    // Must hold mutex when calling into h_A_star because other callbacks can race.
    // If h_A_star is slow you may wish to release the mutex during plan call and
    // guard shared state separately; here we keep it simple and blocking.
    auto t0 = std::chrono::high_resolution_clock::now();

    // Plan: use Hybrid_A_Star::plan(...) signature you had earlier.
    // path = h_A_star->plan(&NH_GRID, &OG, cco, &Start, &End, HeadingResolution, 1000, vehicleWidth, vehicleLength);
    // Note: exact function signature may differ in your repo — adapt args if necessary.
    path = h_A_star->plan(&NH_GRID, &OG, cco, &Start, &End, HeadingResolution, 1000, vehicleWidth, vehicleLength);

    auto t1 = std::chrono::high_resolution_clock::now();
    auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
    avg_time_us += elapsed_us;
    iteration++;

    RCLCPP_INFO(this->get_logger(), "Plan finished in %lld us (avg %f us over %d iters). Path size: %zu",
                static_cast<long long>(elapsed_us), avg_time_us / std::max(1, iteration), iteration, path.size());

    if (!path.empty())
    {
      publishPath();
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Path is empty");
    }
  }

  void publishPath()
  {
    nav_msgs::msg::Path msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";

    for (const auto &point : path)
    {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.stamp = this->now();
      pose_stamped.header.frame_id = "map";
      pose_stamped.pose.position.x = point.x;
      pose_stamped.pose.position.y = point.y;
      pose_stamped.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, point.psi);
      pose_stamped.pose.orientation.x = q.x();
      pose_stamped.pose.orientation.y = q.y();
      pose_stamped.pose.orientation.z = q.z();
      pose_stamped.pose.orientation.w = q.w();

      msg.poses.push_back(pose_stamped);
    }

    path_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published path with %zu poses", msg.poses.size());
  }
};

} // namespace if_ROS
} // namespace adore

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<adore::if_ROS::GraphSearchNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
