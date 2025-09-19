// graph_search_node.cpp
//
// GraphSearchNode: ROS2 node that wraps ADORe Hybrid A* planner.
//
// This file is an updated drop-in replacement for your previous GraphSearchNode.
// Key changes:
//  - store map origin + resolution on map receive
//  - convert world->grid (floating cell coords) before calling Node::setPosition
//  - validate using OG.worldToGrid() and OG.check_valid_position(row,col)
//  - convert planner path points (grid units) back to world meters on publish
//  - improved logging
//
// Notes:
//  - Node::setPosition expects Width=rows, Length=cols, Depth, HeadingResolutionRad
//  - map cell coordinate convention: cell_x = (world_x - origin_x) / resolution (column index)
//                        cell_y = (world_y - origin_y) / resolution (row index)
//  - If your map origin has rotation (non-identity quaternion), OG.worldToGrid is used for validation.
//    If OG.worldToGrid already rotates the point internally, the simple cell_x/cell_y conversion is still valid
//    using the stored origin/resolution. If you observe offset/rotation issues, we may need to transform world->map
//    using the origin orientation explicitly (I can provide that patch if needed).

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <memory>
#include <mutex>

#include "env/occupancy_grid.h"
#include "env/node.h"
#include "env/search_grid.h"
#include "algo/hybrid_A_star.h"
#include "algo/collision_check_offline.h"
#include "env/trajectory_smoothing.h"
#include "env/path.h"

using namespace std::chrono_literals;

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
    avg_time_us(0.0)
  {
    RCLCPP_INFO(this->get_logger(), "GraphSearchNode constructor called");

    rclcpp::QoS qos(10);

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", qos, std::bind(&GraphSearchNode::receive_map_data, this, std::placeholders::_1));

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "pose", qos, std::bind(&GraphSearchNode::receiveStartPose, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", qos, std::bind(&GraphSearchNode::receiveEndPose, this, std::placeholders::_1));

    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("external_path_from_ros1", qos);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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
  const int HeadingResolution;
  int Depth;

  uint32_t grid_height_;
  uint32_t grid_width_;

  adore::env::OccupanyGrid OG;
  adore::fun::GRID<adore::fun::Node<3, double>> NH_GRID;

  adore::fun::Hybrid_A_Star* h_A_star;
  adore::fun::CollisionCheckOffline* cco;
  adore::fun::TrajectorySmoothing* smoothing;

  // start/goal nodes (in grid units after setPosition)
  adore::fun::Node<3, double> Start;
  adore::fun::Node<3, double> End;

  // results
  TrajectoryVector path;

  // flags / runtime
  bool first_map_received_;
  bool validStart;
  bool validEnd;

  // map origin + resolution for conversions
  double map_origin_x_ = 0.0;
  double map_origin_y_ = 0.0;
  double map_resolution_ = 0.0;

  // bookkeeping
  double avg_time_us;
  int iteration;
  double vehicleLength;
  double vehicleWidth;
  int time_us;

  std::mutex mutex_;

  geometry_msgs::msg::Pose start_pose_;
  geometry_msgs::msg::Pose goal_pose_;

  // tf2
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  
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

      // collision checker: parameters as before
      cco = new adore::fun::CollisionCheckOffline(2, 2, HeadingResolution, 10);

      // store map origin & resolution (used to convert world <-> grid)
      map_origin_x_ = msg->info.origin.position.x;
      map_origin_y_ = msg->info.origin.position.y;
      map_resolution_ = msg->info.resolution;

      RCLCPP_INFO(this->get_logger(), "Map initialized: width=%u height=%u Depth=%d", grid_width_, grid_height_, Depth);
      RCLCPP_INFO(this->get_logger(), "Map origin (m)=(%.6f, %.6f) resolution=%.6f", map_origin_x_, map_origin_y_, map_resolution_);
    }

    // Optionally plan immediately if both start and goal already known
    if (validStart && validEnd)
    {
      planAndPublish();
    }
  }

  void receiveStartPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "receive start pose");
    std::lock_guard<std::mutex> lk(mutex_);

    std::string target_frame = "map";
    geometry_msgs::msg::PoseWithCovarianceStamped pose_in_map = *msg;

    if (msg->header.frame_id != target_frame) {
      try {
        auto tf = tf_buffer_->lookupTransform(
          target_frame, msg->header.frame_id, tf2::TimePointZero, std::chrono::milliseconds(100));
        tf2::doTransform(*msg, pose_in_map, tf);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform start pose: %s", ex.what());
        validStart = false;
        return;
      }
    }

    double r, p, y;
    tf2::Quaternion q(
      pose_in_map.pose.pose.orientation.x,
      pose_in_map.pose.pose.orientation.y,
      pose_in_map.pose.pose.orientation.z,
      pose_in_map.pose.pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(r, p, y);

    double wx = pose_in_map.pose.pose.position.x;
    double wy = pose_in_map.pose.pose.position.y;

    RCLCPP_INFO(this->get_logger(), "Received start pose: world x=%.6f, y=%.6f, yaw=%.6f", wx, wy, y);

    // Convert world -> integer grid indices for validation
    int row = -1, col = -1;
    if (!OG.worldToGrid(wx, wy, row, col)) {
      RCLCPP_WARN(this->get_logger(), "Start pose outside map bounds (world x=%.6f y=%.6f)", wx, wy);
      validStart = false;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Start maps to grid (row=%d, col=%d)", row, col);

    if (!OG.check_valid_position(row, col)) {
      RCLCPP_WARN(this->get_logger(), "Start pose maps to an occupied/unknown grid cell (r=%d c=%d)", row, col);
      validStart = false;
      return;
    }

    // Convert world -> floating cell coordinates (column, row)
    // cell_x = (world_x - origin_x) / resolution
    double cell_x = (wx - map_origin_x_) / map_resolution_;
    double cell_y = (wy - map_origin_y_) / map_resolution_;

    double heading_rad = adore::mad::CoordinateConversion::DegToRad(HeadingResolution);

    // Node::setPosition expects Width=rows, Length=cols, Depth, HeadingResolutionRad
    validStart = Start.setPosition(cell_x, cell_y, y, grid_height_, grid_width_, Depth, heading_rad);
    if (!validStart) {
      RCLCPP_WARN(this->get_logger(), "Start.setPosition rejected (cell_x=%.6f, cell_y=%.6f)", cell_x, cell_y);
      return;
    }

    start_pose_ = pose_in_map.pose.pose;
    RCLCPP_INFO(this->get_logger(), "Received valid start pose (cell x=%.6f y=%.6f) validStart=%d", cell_x, cell_y, validStart);

    if (first_map_received_ && validStart && validEnd) {
      planAndPublish();
    }
  }

  void receiveEndPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);

    std::string target_frame = "map";
    geometry_msgs::msg::PoseStamped pose_in_map = *msg;

    if (msg->header.frame_id != target_frame) {
      try {
        auto tf = tf_buffer_->lookupTransform(
          target_frame, msg->header.frame_id, tf2::TimePointZero, std::chrono::milliseconds(100));
        tf2::doTransform(*msg, pose_in_map, tf);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform goal pose: %s", ex.what());
        validEnd = false;
        return;
      }
    }

    double r, p, y;
    tf2::Quaternion q(
      pose_in_map.pose.orientation.x,
      pose_in_map.pose.orientation.y,
      pose_in_map.pose.orientation.z,
      pose_in_map.pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(r, p, y);

    double wx = pose_in_map.pose.position.x;
    double wy = pose_in_map.pose.position.y;

    RCLCPP_INFO(this->get_logger(), "Received goal pose (world): x=%.6f, y=%.6f, yaw=%.6f", wx, wy, y);

    // Convert world -> grid indices
    int row = -1, col = -1;
    if (!OG.worldToGrid(wx, wy, row, col)) {
      RCLCPP_WARN(this->get_logger(), "Goal pose outside map bounds (world x=%.6f y=%.6f)", wx, wy);
      validEnd = false;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Goal maps to grid (row=%d, col=%d)", row, col);

    if (!OG.check_valid_position(row, col)) {
      RCLCPP_WARN(this->get_logger(), "Goal pose maps to an occupied/unknown grid cell (r=%d c=%d)", row, col);
      validEnd = false;
      return;
    }

    // Convert to floating cell coords
    double cell_x = (wx - map_origin_x_) / map_resolution_;
    double cell_y = (wy - map_origin_y_) / map_resolution_;

    double heading_rad = adore::mad::CoordinateConversion::DegToRad(HeadingResolution);

    validEnd = End.setPosition(cell_x, cell_y, y, grid_height_, grid_width_, Depth, heading_rad);
    if (!validEnd) {
      RCLCPP_WARN(this->get_logger(), "End.setPosition rejected (cell_x=%.6f, cell_y=%.6f)", cell_x, cell_y);
      return;
    }

    goal_pose_ = pose_in_map.pose;
    RCLCPP_INFO(this->get_logger(), "Received valid goal pose (cell x=%.6f y=%.6f) validEnd=%d", cell_x, cell_y, validEnd);

    if (first_map_received_ && validStart && validEnd) {
      planAndPublish();
    }
  }

  void planAndPublish()
  {
    // plan synchronously while holding mutex - simple and deterministic
    auto t0 = std::chrono::high_resolution_clock::now();

    path = h_A_star->plan(&NH_GRID, &OG, cco, &Start, &End, HeadingResolution, 1000, vehicleWidth, vehicleLength);

    auto t1 = std::chrono::high_resolution_clock::now();
    auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
    avg_time_us += elapsed_us;
    iteration++;

    RCLCPP_INFO(this->get_logger(), "Plan finished in %lld us (avg %.3f us over %d iters). Path size: %zu",
                static_cast<long long>(elapsed_us), avg_time_us / std::max(1, iteration), iteration, path.size());
    
    validStart = false;
    validEnd = false;
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
      // Convert planner grid units (cell coordinates) back to world meters
      double world_x = map_origin_x_ + point.x * map_resolution_;
      double world_y = map_origin_y_ + point.y * map_resolution_;

      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.stamp = this->now();
      pose_stamped.header.frame_id = "map";
      pose_stamped.pose.position.x = world_x;
      pose_stamped.pose.position.y = world_y;
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
