// graph_search_node.cpp
//
// Hybrid A* wrapper with robust collision handling + anti-flap + goal hysteresis.
//
// - Uses TF polling for start pose
// - Orientation-aware world<->grid conversions
// - **Planner-only** inflation on a cloned grid (final check uses pristine grid)
// - Planner-only clearance (keeps published check strict-ish but configurable shrink)
// - FINAL hard collision check with toggle-able unknown policy + footprint shrink
// - Path “stickiness” (keeps old path unless the new one is clearly better)
// - Goal hysteresis (stop replanning when inside goal tolerances)
// - Warmup/keepalive path re-publishing to avoid "first plan missed" races
//
// Assumes OccupanyGrid exposes: Eigen::MatrixXi Grid, resolution, origin yaw,
// check_valid_position(row,col), inflate(int), unknown_as_occupied(), set_unknown_as_occupied(bool).

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <tf2/time.h>   // tf2::durationFromSec

#include <chrono>
#include <memory>
#include <mutex>
#include <cmath>
#include <algorithm>

#include "env/occupancy_grid.h"
#include "env/node.h"
#include "env/search_grid.h"
#include "algo/hybrid_A_star.h"
#include "algo/collision_check_offline.h"
#include "env/trajectory_smoothing.h"
#include "env/path.h"   // TrajectoryVector

using namespace std::chrono_literals;

namespace {
inline double deg2rad(double d){ return d * M_PI / 180.0; }
inline double rad2deg(double r){ return r * 180.0 / M_PI; }

inline double quatYaw(const geometry_msgs::msg::Quaternion& q)
{
  tf2::Quaternion tq(q.x,q.y,q.z,q.w);
  double r,p,y; tf2::Matrix3x3(tq).getRPY(r,p,y);
  return y;
}
}

namespace adore {
namespace if_ROS {

class GraphSearchNode : public rclcpp::Node
{
public:
  GraphSearchNode()
  : Node("graph_search_node"),
    grid_height_(0),
    grid_width_(0),
    HeadingResolution(20),
    Depth(0),
    h_A_star(nullptr),
    cco(nullptr),
    smoothing(nullptr),
    first_map_received_(false),
    validStart(false),
    validEnd(false),
    plan_count_(0),
    sum_plan_time_us_(0.0),
    map_origin_x_(0.0),
    map_origin_y_(0.0),
    map_resolution_(0.0),
    map_origin_yaw_(0.0),
    world_frame_("map"),
    base_frame_("base_link"),
    // robot dims (meters)
    vehicleWidth_m_(0.40),
    vehicleLength_m_(0.40),
    min_replan_translation_(0.25),
    min_replan_rotation_deg_(10.0),
    tf_poll_hz_(30.0),
    inflate_cells_(2),
    // planner tunables
    max_iterations_(500000),
    planning_clearance_m_(0.05),
    final_allow_unknown_(true),
    publish_truncated_path_(false),
    min_publish_length_m_(0.5),
    min_publish_points_(10),
    min_publish_ratio_(0.8),
    tf_lookup_timeout_s_(0.2),
    tf_buffer_cache_s_(120.0),
    occ_threshold_(50),
    // anti-flap (path stickiness)
    keep_old_if_valid_prefix_(true),
    lock_prefix_m_(1.5),
    max_prefix_deviation_m_(0.35),
    require_improvement_m_(0.25),
    // goal hysteresis
    goal_pos_tol_m_(0.25),
    goal_yaw_tol_deg_(20.0),
    stop_replanning_within_goal_radius_m_(0.5),
    post_goal_freeze_s_(3.0),
    publish_when_goal_reached_(false),
    // republish/warmup
    warmup_republish_s_(0.6),
    keepalive_rate_hz_(1.0),
    enable_keepalive_(true),
    // anchoring
    anchor_start_to_robot_(true),
    anchor_start_gap_m_(0.30),
    anchor_goal_to_target_(false),
    anchor_goal_gap_m_(0.35),
    have_last_published_goal_(false),
    // NEW: final check tunables
    final_shrink_cells_(1.0),
    final_strip_prefix_m_(0.45)
  {
    // QoS:
    auto qos_map   = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    auto qos_goal  = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();   // volatile
    auto qos_pathp = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    map_sub_  = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", qos_map, std::bind(&GraphSearchNode::receive_map_data, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", qos_goal, std::bind(&GraphSearchNode::receiveEndPose, this, std::placeholders::_1));

    rclcpp::QoS latched_qos(rclcpp::KeepLast(1));
    latched_qos.transient_local().reliable();
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/external_path_from_ros1", latched_qos);

    // ---------------- Parameters ----------------
    this->declare_parameter<std::string>("world_frame",        world_frame_);
    this->declare_parameter<std::string>("base_frame",         base_frame_);
    this->declare_parameter<double>("heading_resolution_deg",  HeadingResolution);
    this->declare_parameter<double>("vehicle_width_m",         vehicleWidth_m_);
    this->declare_parameter<double>("vehicle_length_m",        vehicleLength_m_);
    this->declare_parameter<double>("min_replan_translation",  min_replan_translation_);
    this->declare_parameter<double>("min_replan_rotation_deg", min_replan_rotation_deg_);
    this->declare_parameter<double>("tf_poll_hz",              tf_poll_hz_);
    this->declare_parameter<int>("inflate_cells",              inflate_cells_);

    // planner tunables
    this->declare_parameter<int>("max_iterations",             max_iterations_);
    this->declare_parameter<double>("planning_clearance_m",    planning_clearance_m_);
    this->declare_parameter<bool>("final_allow_unknown",       final_allow_unknown_);
    this->declare_parameter<bool>("publish_truncated_path",    publish_truncated_path_);
    this->declare_parameter<double>("min_publish_length_m",    min_publish_length_m_);
    this->declare_parameter<int>("min_publish_points",         min_publish_points_);
    this->declare_parameter<double>("min_publish_ratio",       min_publish_ratio_);

    // NEW: final check tunables
    this->declare_parameter<double>("final_shrink_cells",      final_shrink_cells_);
    this->declare_parameter<double>("final_strip_prefix_m",    final_strip_prefix_m_);

    // TF robustness params
    this->declare_parameter<double>("tf_lookup_timeout_s",     tf_lookup_timeout_s_);
    this->declare_parameter<double>("tf_buffer_cache_s",       tf_buffer_cache_s_);

    // map decoding
    this->declare_parameter<int>("occ_threshold",              occ_threshold_);

    // anti-flap
    this->declare_parameter<bool>("keep_old_if_valid_prefix",  keep_old_if_valid_prefix_);
    this->declare_parameter<double>("lock_prefix_m",           lock_prefix_m_);
    this->declare_parameter<double>("max_prefix_deviation_m",  max_prefix_deviation_m_);
    this->declare_parameter<double>("require_improvement_m",   require_improvement_m_);

    // goal hysteresis
    this->declare_parameter<double>("goal_pos_tol_m",          goal_pos_tol_m_);
    this->declare_parameter<double>("goal_yaw_tol_deg",        goal_yaw_tol_deg_);
    this->declare_parameter<double>("stop_replanning_within_goal_radius_m", stop_replanning_within_goal_radius_m_);
    this->declare_parameter<double>("post_goal_freeze_s",      post_goal_freeze_s_);
    this->declare_parameter<bool>("publish_when_goal_reached", publish_when_goal_reached_);

    // republish/anchoring params
    this->declare_parameter<double>("warmup_republish_s",      warmup_republish_s_);
    this->declare_parameter<double>("keepalive_rate_hz",       keepalive_rate_hz_);
    this->declare_parameter<bool>("enable_keepalive",          enable_keepalive_);
    this->declare_parameter<bool>("anchor_start_to_robot",     anchor_start_to_robot_);
    this->declare_parameter<double>("anchor_start_gap_m",      anchor_start_gap_m_);
    this->declare_parameter<bool>("anchor_goal_to_target",     anchor_goal_to_target_);
    this->declare_parameter<double>("anchor_goal_gap_m",       anchor_goal_gap_m_);

    // ---------------- Read params ----------------
    world_frame_             = this->get_parameter("world_frame").as_string();
    base_frame_              = this->get_parameter("base_frame").as_string();
    HeadingResolution        = static_cast<int>(this->get_parameter("heading_resolution_deg").as_double());
    vehicleWidth_m_          = this->get_parameter("vehicle_width_m").as_double();
    vehicleLength_m_         = this->get_parameter("vehicle_length_m").as_double();
    min_replan_translation_  = this->get_parameter("min_replan_translation").as_double();
    min_replan_rotation_deg_ = this->get_parameter("min_replan_rotation_deg").as_double();
    tf_poll_hz_              = this->get_parameter("tf_poll_hz").as_double();
    inflate_cells_           = this->get_parameter("inflate_cells").as_int();

    max_iterations_          = this->get_parameter("max_iterations").as_int();
    planning_clearance_m_    = this->get_parameter("planning_clearance_m").as_double();
    final_allow_unknown_     = this->get_parameter("final_allow_unknown").as_bool();
    publish_truncated_path_  = this->get_parameter("publish_truncated_path").as_bool();
    min_publish_length_m_    = this->get_parameter("min_publish_length_m").as_double();
    min_publish_points_      = this->get_parameter("min_publish_points").as_int();
    min_publish_ratio_       = this->get_parameter("min_publish_ratio").as_double();

    // NEW: final check tunables
    final_shrink_cells_      = this->get_parameter("final_shrink_cells").as_double();
    final_strip_prefix_m_    = this->get_parameter("final_strip_prefix_m").as_double();

    tf_lookup_timeout_s_     = this->get_parameter("tf_lookup_timeout_s").as_double();
    tf_buffer_cache_s_       = this->get_parameter("tf_buffer_cache_s").as_double();

    occ_threshold_           = this->get_parameter("occ_threshold").as_int();

    keep_old_if_valid_prefix_= this->get_parameter("keep_old_if_valid_prefix").as_bool();
    lock_prefix_m_           = this->get_parameter("lock_prefix_m").as_double();
    max_prefix_deviation_m_  = this->get_parameter("max_prefix_deviation_m").as_double();
    require_improvement_m_   = this->get_parameter("require_improvement_m").as_double();

    goal_pos_tol_m_          = this->get_parameter("goal_pos_tol_m").as_double();
    goal_yaw_tol_deg_        = this->get_parameter("goal_yaw_tol_deg").as_double();
    stop_replanning_within_goal_radius_m_ = this->get_parameter("stop_replanning_within_goal_radius_m").as_double();
    post_goal_freeze_s_      = this->get_parameter("post_goal_freeze_s").as_double();
    publish_when_goal_reached_ = this->get_parameter("publish_when_goal_reached").as_bool();

    warmup_republish_s_      = this->get_parameter("warmup_republish_s").as_double();
    keepalive_rate_hz_       = this->get_parameter("keepalive_rate_hz").as_double();
    enable_keepalive_        = this->get_parameter("enable_keepalive").as_bool();
    anchor_start_to_robot_   = this->get_parameter("anchor_start_to_robot").as_bool();
    anchor_start_gap_m_      = this->get_parameter("anchor_start_gap_m").as_double();
    anchor_goal_to_target_   = this->get_parameter("anchor_goal_to_target").as_bool();
    anchor_goal_gap_m_       = this->get_parameter("anchor_goal_gap_m").as_double();

    if (tf_poll_hz_ < 1.0) tf_poll_hz_ = 1.0;
    if (keepalive_rate_hz_ < 0.1) keepalive_rate_hz_ = 0.1;

    // ---------------- TF Buffer & Listener (with cache) ----------------
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(
      this->get_clock(),
      tf2::durationFromSec(std::max(1.0, tf_buffer_cache_s_))
    );
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ---------------- TF polling timer ----------------
    tf_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / tf_poll_hz_)),
      [this]() {
        try {
          const auto timeout = tf2::durationFromSec(std::max(0.01, tf_lookup_timeout_s_));
          geometry_msgs::msg::TransformStamped tf =
            tf_buffer_->lookupTransform(world_frame_, base_frame_, tf2::TimePointZero, timeout);

          auto pose_in_map = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
          pose_in_map->header.stamp = tf.header.stamp;
          pose_in_map->header.frame_id = tf.header.frame_id;
          pose_in_map->pose.pose.position.x = tf.transform.translation.x;
          pose_in_map->pose.pose.position.y = tf.transform.translation.y;
          pose_in_map->pose.pose.position.z = tf.transform.translation.z;
          pose_in_map->pose.pose.orientation = tf.transform.rotation;

          receiveStartPose(pose_in_map);
        }
        catch (const tf2::TransformException &ex) {
          RCLCPP_DEBUG(this->get_logger(), "TF exception polling start pose: %s", ex.what());
        }
      }
    );

    // --------------- Republish/keepalive timer ---------------
    repub_timer_ = this->create_wall_timer(
      20ms, [this]() {
        std::lock_guard<std::mutex> lk(mtx_);
        if (active_path_.empty()) return;

        const rclcpp::Time now = this->now();

        // Warmup burst after a newly accepted path
        if (now < warmup_until_) {
          publishPath(active_path_);
          last_publish_time_ = now;
          return;
        }

        // Keepalive at low rate so late subscribers/controllers still catch a path
        if (enable_keepalive_) {
          const double since = (now - last_publish_time_).seconds();
          const double period = 1.0 / keepalive_rate_hz_;
          if (since >= period) {
            publishPath(active_path_);
            last_publish_time_ = now;
          }
        }
      }
    );

    goal_freeze_until_ = this->now();
  }

  ~GraphSearchNode()
  {
    try {
      if (tf_timer_) { tf_timer_->cancel(); tf_timer_.reset(); }
      if (repub_timer_) { repub_timer_->cancel(); repub_timer_.reset(); }
    } catch (...) { /* ignore exceptions during shutdown */ }
    if (h_A_star) { delete h_A_star; h_A_star = nullptr; }
    if (cco)      { delete cco;      cco = nullptr; }
    if (smoothing){ delete smoothing; smoothing = nullptr; }
  }

private:
  // ---------- ROS wiring ----------
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::TimerBase::SharedPtr repub_timer_;

  // ---------- Planner config ----------
  int HeadingResolution; // deg
  int Depth;

  // ---------- Map/grid ----------
  uint32_t grid_height_;
  uint32_t grid_width_;

  adore::env::OccupanyGrid OG;
  adore::fun::GRID<adore::fun::Node<3, double>> NH_GRID;

  // ---------- Planner components ----------
  adore::fun::Hybrid_A_Star*         h_A_star{nullptr};
  adore::fun::CollisionCheckOffline* cco{nullptr};
  adore::fun::TrajectorySmoothing*   smoothing{nullptr};

  // ---------- Nodes ----------
  adore::fun::Node<3, double> Start;
  adore::fun::Node<3, double> End;

  // Paths
  adore::TrajectoryVector path_;         // last planned (candidate)
  adore::TrajectoryVector active_path_;  // last published (sticky)

  // ---------- State ----------
  bool first_map_received_;
  bool validStart;
  bool validEnd;

  uint64_t plan_count_{0};
  long double sum_plan_time_us_{0.0};

  // Map origin + conversion
  double map_origin_x_;
  double map_origin_y_;
  double map_resolution_;
  double map_origin_yaw_; // radians

  // Frames + gating
  std::string world_frame_;
  std::string base_frame_;
  double vehicleWidth_m_;
  double vehicleLength_m_;
  double min_replan_translation_;
  double min_replan_rotation_deg_;
  double tf_poll_hz_;
  int    inflate_cells_;

  // planner tunables
  int    max_iterations_;
  double planning_clearance_m_;
  bool   final_allow_unknown_;
  bool   publish_truncated_path_;
  double min_publish_length_m_;
  int    min_publish_points_;
  double min_publish_ratio_;

  // TF robustness
  double tf_lookup_timeout_s_;
  double tf_buffer_cache_s_;

  // Map decoding
  int occ_threshold_;

  // anti-flap
  bool   keep_old_if_valid_prefix_;
  double lock_prefix_m_;
  double max_prefix_deviation_m_;
  double require_improvement_m_;

  // goal hysteresis
  double goal_pos_tol_m_;
  double goal_yaw_tol_deg_;
  double stop_replanning_within_goal_radius_m_;
  double post_goal_freeze_s_;
  bool   publish_when_goal_reached_;
  rclcpp::Time goal_freeze_until_;

  geometry_msgs::msg::Pose goal_pose_;
  geometry_msgs::msg::PoseWithCovarianceStamped last_planned_start_;

  // Track the goal used for the currently active_path_
  geometry_msgs::msg::Pose last_published_goal_pose_;
  bool have_last_published_goal_;

  // Republish/keepalive state
  rclcpp::Time  last_publish_time_;
  rclcpp::Time  warmup_until_;
  double        warmup_republish_s_;
  double        keepalive_rate_hz_;
  bool          enable_keepalive_;

  // Anchoring
  bool  anchor_start_to_robot_;
  double anchor_start_gap_m_;
  bool  anchor_goal_to_target_;
  double anchor_goal_gap_m_;

  // NEW: final check tunables
  double final_shrink_cells_;
  double final_strip_prefix_m_;

  std::mutex mtx_;
  geometry_msgs::msg::Pose start_pose_; // last valid start (world)

private:
  // ==== Orientation-aware conversions ====
  inline void worldToCell(double wx, double wy, double& cx, double& cy) const {
    const double dx = wx - map_origin_x_;
    const double dy = wy - map_origin_y_;
    const double c = std::cos(-map_origin_yaw_);
    const double s = std::sin(-map_origin_yaw_);
    const double gx = c*dx - s*dy;     // rotate by -yaw
    const double gy = s*dx + c*dy;
    cx = gx / map_resolution_;
    cy = gy / map_resolution_;
  }
  inline void cellToWorld(double cx, double cy, double& wx, double& wy) const {
    const double gx = cx * map_resolution_;
    const double gy = cy * map_resolution_;
    const double c = std::cos(map_origin_yaw_);
    const double s = std::sin(map_origin_yaw_);
    const double dx = c*gx - s*gy;     // rotate by +yaw
    const double dy = s*gx + c*gy;
    wx = map_origin_x_ + dx;
    wy = map_origin_y_ + dy;
  }

  // ---- Path length helper (meters) for first N points
  double pathLengthMetersN(const adore::TrajectoryVector& v, size_t n) const {
    if (n < 2) return 0.0;
    double L = 0.0;
    const size_t M = std::min(n, v.size());
    for (size_t i = 1; i < M; ++i) {
      const double dx = (v[i].x - v[i-1].x) * map_resolution_;
      const double dy = (v[i].y - v[i-1].y) * map_resolution_;
      L += std::hypot(dx, dy);
    }
    return L;
  }
  double pathLengthMeters(const adore::TrajectoryVector& v) const {
    return pathLengthMetersN(v, v.size());
  }

  // ---- Convert meters of prefix to point count (approx via cumulative length)
  size_t pointsForMeters(const adore::TrajectoryVector& v, double meters) const {
    if (v.size() < 2 || meters <= 0.0) return std::min<size_t>(v.size(), 1);
    double L = 0.0;
    for (size_t i = 1; i < v.size(); ++i) {
      L += std::hypot((v[i].x - v[i-1].x) * map_resolution_,
                      (v[i].y - v[i-1].y) * map_resolution_);
      if (L >= meters) return i+1;
    }
    return v.size();
  }

  // ---- RMS distance between prefixes (meters)
  double prefixRMSDistanceMeters(const adore::TrajectoryVector& A,
                                 const adore::TrajectoryVector& B,
                                 size_t n) const
  {
    if (A.empty() || B.empty() || n == 0) return 1e9;
    const size_t N = std::min({n, A.size(), B.size()});
    double acc = 0.0;
    for (size_t i = 0; i < N; ++i) {
      const double dx = (A[i].x - B[i].x) * map_resolution_;
      const double dy = (A[i].y - B[i].y) * map_resolution_;
      acc += dx*dx + dy*dy;
    }
    return std::sqrt(acc / static_cast<double>(N));
  }

  // ---- Remaining length along path from current start pose (meters)
  double remainingLengthMeters(const adore::TrajectoryVector& P) const
  {
    if (P.size() < 2) return 0.0;
    double cell_x, cell_y;
    worldToCell(start_pose_.position.x, start_pose_.position.y, cell_x, cell_y);
    size_t idx = 0;
    const double tol_cells_sq = 1.0 * 1.0;
    bool found = false;
    for (size_t i = 0; i < P.size(); ++i) {
      const double dx = P[i].x - cell_x;
      const double dy = P[i].y - cell_y;
      if (dx*dx + dy*dy <= tol_cells_sq) {
        idx = i; found = true; break;
      }
    }
    if (!found) return pathLengthMeters(P);
    const double tot = pathLengthMeters(P);
    double prefix = 0.0;
    if (idx >= 1) prefix = pathLengthMetersN(P, idx);
    return std::max(0.0, tot - prefix);
  }

  // ---- Validate prefix of path against current grid using robot footprint
  size_t validPrefixCount(const adore::TrajectoryVector& P,
                          double w_cells, double l_cells) const
  {
    size_t safe_n = 0;
    for (const auto& pt : P) {
      if (!poseFootprintIsFree(pt.x, pt.y, pt.psi, w_cells, l_cells)) break;
      ++safe_n;
    }
    return safe_n;
  }

  // ==== Map reception ====
  void receive_map_data(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);

    grid_height_    = msg->info.height;
    grid_width_     = msg->info.width;
    map_origin_x_   = msg->info.origin.position.x;
    map_origin_y_   = msg->info.origin.position.y;
    map_resolution_ = msg->info.resolution;

    // origin yaw
    {
      const auto& q = msg->info.origin.orientation;
      tf2::Quaternion tq(q.x, q.y, q.z, q.w);
      double r, p, y; tf2::Matrix3x3(tq).getRPY(r, p, y);
      map_origin_yaw_ = y;
    }

    // ---- Directly populate OG (PRISTINE: no inflation here) ----
    OG.width       = grid_width_;
    OG.height      = grid_height_;
    OG.resolution  = map_resolution_;
    OG.origin_x    = map_origin_x_;
    OG.origin_y    = map_origin_y_;
    OG.origin_yaw  = map_origin_yaw_;
    OG.Grid        = Eigen::MatrixXi::Zero(OG.height, OG.width);

    for (uint32_t y = 0; y < OG.height; ++y) {
      for (uint32_t x = 0; x < OG.width; ++x) {
        const size_t idx = static_cast<size_t>(x + y * OG.width);
        const int v = static_cast<int>(msg->data[idx]);
        if (v < 0)                     OG.Grid(y, x) = -1;  // unknown
        else if (v >= occ_threshold_)  OG.Grid(y, x) =  1;  // occupied
        else                           OG.Grid(y, x) =  0;  // free
      }
    }

    // --- Ensure search structures follow map size changes ---
    static uint32_t prev_w = 0, prev_h = 0;
    const bool size_changed = (OG.width != prev_w) || (OG.height != prev_h);

    if (!first_map_received_) {
      first_map_received_ = true;

      Depth = static_cast<int>(std::round(360.0 / static_cast<double>(HeadingResolution)));
      NH_GRID.resize(grid_height_, grid_width_, Depth);
      NH_GRID.set_owns_pointers(false);

      smoothing = new adore::fun::TrajectorySmoothing();
      h_A_star  = new adore::fun::Hybrid_A_Star(false);

      h_A_star->setSize(grid_height_, grid_width_);

      const double vehicleWidth_cells  = vehicleWidth_m_  / map_resolution_;
      const double vehicleLength_cells = vehicleLength_m_ / map_resolution_;
      cco = new adore::fun::CollisionCheckOffline(
              vehicleWidth_cells, vehicleLength_cells,
              HeadingResolution, 10 /*subcell sampling per axis*/);

      RCLCPP_INFO(this->get_logger(),
        "Map init: WxH=%ux%u, res=%.3f m/cell, origin yaw=%.2f deg, Depth=%d, inflate(plan)=%d, occ>=%d",
        grid_width_, grid_height_, map_resolution_, rad2deg(map_origin_yaw_), Depth,
        inflate_cells_, occ_threshold_);
    }
    else if (size_changed) {
      // Reconfigure on resize
      Depth = static_cast<int>(std::round(360.0 / static_cast<double>(HeadingResolution)));
      NH_GRID.resize(grid_height_, grid_width_, Depth);
      h_A_star->setSize(grid_height_, grid_width_);

      RCLCPP_INFO(this->get_logger(),
        "Map resized → WxH=%ux%u, res=%.3f m/cell, origin yaw=%.2f deg, Depth=%d",
        grid_width_, grid_height_, map_resolution_, rad2deg(map_origin_yaw_), Depth);
    }

    prev_w = OG.width;
    prev_h = OG.height;
  }

  // ==== Start pose (from TF) ====
  void receiveStartPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_in_map)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    validStart = false;

    const double y = quatYaw(pose_in_map->pose.pose.orientation);

    const double wx = pose_in_map->pose.pose.position.x;
    const double wy = pose_in_map->pose.pose.position.y;

    double cell_x, cell_y;
    worldToCell(wx, wy, cell_x, cell_y);
    const int row = static_cast<int>(std::floor(cell_y));
    const int col = static_cast<int>(std::floor(cell_x));

    // Allow unknown start, only block occupied/out of bounds
    if (row < 0 || col < 0 || row >= (int)OG.height || col >= (int)OG.width) return;
    if (OG.Grid(row, col) == 1) return;

    Start.setPosition(cell_x, cell_y, y,
                      static_cast<int>(grid_height_),
                      static_cast<int>(grid_width_),
                      Depth, deg2rad(static_cast<double>(HeadingResolution)));

    start_pose_ = pose_in_map->pose.pose;
    validStart = true;

    if (first_map_received_ && validStart && validEnd) { maybePlan(); }
  }

  // ==== Goal pose (topic) ====
  void receiveEndPose(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    validEnd = false;

    // --- 1) Transform goal into world_frame_ if needed ---
    geometry_msgs::msg::PoseStamped goal_world;
    try {
      geometry_msgs::msg::PoseStamped in = *pose_msg;
      if (in.header.frame_id.empty()) {
        in.header.frame_id = world_frame_;
      }

      if (in.header.frame_id != world_frame_) {
        const auto timeout = tf2::durationFromSec(std::max(0.01, tf_lookup_timeout_s_));
        goal_world = tf_buffer_->transform(in, world_frame_, timeout);
        RCLCPP_INFO(this->get_logger(),
                    "Transformed goal from '%s' to '%s'.",
                    in.header.frame_id.c_str(), world_frame_.c_str());
      } else {
        goal_world = in;
      }
    }
    catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to transform goal from '%s' to '%s': %s",
                  pose_msg->header.frame_id.c_str(), world_frame_.c_str(), ex.what());
      return;
    }

    // --- 2) Convert to grid coords & sanity-check bounds ---
    const double gx = goal_world.pose.position.x;
    const double gy = goal_world.pose.position.y;

    double cell_x, cell_y;
    worldToCell(gx, gy, cell_x, cell_y);

    if (cell_x < 0.0 || cell_y < 0.0 ||
        cell_x >= static_cast<double>(OG.width) ||
        cell_y >= static_cast<double>(OG.height))
    {
      RCLCPP_WARN(this->get_logger(),
                  "Goal out of grid bounds (cx=%.2f, cy=%.2f, W=%u, H=%u).",
                  cell_x, cell_y, OG.width, OG.height);
      return;
    }

    // Use nearest cell to probe occupancy; keep doubles for End.setPosition
    int col = static_cast<int>(std::lround(cell_x));
    int row = static_cast<int>(std::lround(cell_y));
    col = std::max(0, std::min(col, static_cast<int>(OG.width)  - 1));
    row = std::max(0, std::min(row, static_cast<int>(OG.height) - 1));

    const int gv = OG.Grid(row, col);  // -1=unknown, 0=free, 1=occupied (PRISTINE)
    if (gv == 1) {
      RCLCPP_WARN(this->get_logger(),
                  "Goal cell is occupied: row=%d col=%d.", row, col);
      return;
    }
    // Unknown (-1) or free (0) → accept.

    // --- 3) Set End (keep yaw neutral unless you want to enforce heading) ---
    End.setPosition(cell_x, cell_y, 0.0,
                    static_cast<int>(grid_height_),
                    static_cast<int>(grid_width_),
                    Depth, deg2rad(static_cast<double>(HeadingResolution)));

    validEnd   = true;
    goal_pose_ = goal_world.pose;

    // --- 4) Ensure new goal takes effect immediately ---
    goal_freeze_until_ = this->now(); // cancel freeze
    active_path_.clear();

    if (first_map_received_ && validStart) {
      maybePlan(/*force=*/true);
    }
  }

  // ==== Goal checks / gating ====
  bool withinGoalTolerancesWorld() const
  {
    const double sx = start_pose_.position.x;
    const double sy = start_pose_.position.y;
    const double gx = goal_pose_.position.x;
    const double gy = goal_pose_.position.y;
    const double dx = sx - gx;
    const double dy = sy - gy;
    const double dist = std::hypot(dx, dy);
    const double yaw_s = quatYaw(start_pose_.orientation);
    const double yaw_g = quatYaw(goal_pose_.orientation);
    double dyaw = std::fabs(yaw_s - yaw_g);
    dyaw = std::fmod(dyaw, 2*M_PI);
    if (dyaw > M_PI) dyaw = 2*M_PI - dyaw;

    const bool pos_ok = dist <= goal_pos_tol_m_;
    const bool yaw_ok = rad2deg(dyaw) <= goal_yaw_tol_deg_;
    return pos_ok && yaw_ok;
  }

  bool shouldReplan(bool force=false) const
  {
    if (force) return true;  // honor force FIRST

    // Freeze near goal (hysteresis)
    if (this->now() < goal_freeze_until_) return false;

    // If we are in a small radius around the goal, just stop replanning to avoid oscillations.
    const double sx = start_pose_.position.x;
    const double sy = start_pose_.position.y;
    const double gx = goal_pose_.position.x;
    const double gy = goal_pose_.position.y;
    const double dist_goal = std::hypot(sx-gx, sy-gy);
    if (dist_goal <= stop_replanning_within_goal_radius_m_) return false;

    if (last_planned_start_.header.stamp.sec == 0) return true;

    const auto &A = last_planned_start_.pose.pose;
    const auto &B = start_pose_;
    const double dx = B.position.x - A.position.x;
    const double dy = B.position.y - A.position.y;
    const double dist = std::hypot(dx, dy);
    const double yawA = quatYaw(A.orientation);
    const double yawB = quatYaw(B.orientation);
    double d_yaw = std::fabs(yawB - yawA) * 180.0 / M_PI;
    d_yaw = std::fmod(d_yaw, 360.0); if (d_yaw > 180.0) d_yaw = 360.0 - d_yaw;
    return dist >= min_replan_translation_ || d_yaw >= min_replan_rotation_deg_;
  }

  void maybePlan(bool force=false)
  {
    if (!shouldReplan(force)) return;
    planAndMaybePublish();
  }

  // ==== Footprint check in CELL coordinates ====
  bool poseFootprintIsFree(double cx, double cy, double psi,
                           double width_cells, double length_cells) const
  {
    const double hw = 0.5 * width_cells;
    const double hl = 0.5 * length_cells;

    const int xmin = static_cast<int>(std::floor(-hl));
    const int xmax = static_cast<int>(std::ceil( hl));
    const int ymin = static_cast<int>(std::floor(-hw));
    const int ymax = static_cast<int>(std::ceil( hw));

    const double c = std::cos(psi);
    const double s = std::sin(psi);

    for (int ix = xmin; ix <= xmax; ++ix) {
      for (int iy = ymin; iy <= ymax; ++iy) {
        const double rx = ix * c - iy * s;
        const double ry = ix * s + iy * c;

        const double gx = cx + rx;
        const double gy = cy + ry;

        const int row = static_cast<int>(std::floor(gy));
        const int col = static_cast<int>(std::floor(gx));
        if (!OG.check_valid_position(row, col)) {
          return false; // unknown/occupied/out-of-bounds
        }
      }
    }
    return true;
  }

  // ==== Planning + anti-flap + goal hysteresis ====
  void planAndMaybePublish()
  {
    auto t0 = std::chrono::high_resolution_clock::now();

    // Planner-only clearance: expand robot for planning
    const double plan_w = std::max(0.05, vehicleWidth_m_  + 2.0*planning_clearance_m_);
    const double plan_l = std::max(0.05, vehicleLength_m_ + 2.0*planning_clearance_m_);

    // Clone pristine OG -> planner grid; optionally inflate for planning safety
    adore::env::OccupanyGrid OG_plan = OG;
    if (inflate_cells_ > 0) {
      OG_plan.inflate(inflate_cells_);
    }

    // Allow unknown during planning (so we can explore toward unknown goals)
    const bool old_unknown_plan = OG_plan.unknown_as_occupied();
    OG_plan.set_unknown_as_occupied(false);
    path_ = h_A_star->plan(&NH_GRID, &OG_plan, cco, &Start, &End,
                           HeadingResolution, max_iterations_,
                           plan_w, plan_l);
    OG_plan.set_unknown_as_occupied(old_unknown_plan);

    auto t1 = std::chrono::high_resolution_clock::now();
    auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
    sum_plan_time_us_ += static_cast<long double>(elapsed_us);
    ++plan_count_;
    const long double avg = sum_plan_time_us_ / std::max<uint64_t>(1, plan_count_);
    (void)avg; // quiet if not logged

    // If planner produced nothing, keep old active path if it’s still valid
    if (path_.empty()) {
      RCLCPP_WARN(this->get_logger(), "Planner returned empty path");
      if (!active_path_.empty()) publishPath(active_path_);
      return;
    }

    // ---- FINAL FAILSAFE: validate NEW path against **pristine** grid OG ----
    // Apply final shrink in **cells** to harmonize with planner's collision model
    const double w_cells_raw = std::max(1.0, vehicleWidth_m_  / map_resolution_);
    const double l_cells_raw = std::max(1.0, vehicleLength_m_ / map_resolution_);
    const double w_cells = std::max(1.0, w_cells_raw - 2.0*std::max(0.0, final_shrink_cells_));
    const double l_cells = std::max(1.0, l_cells_raw - 2.0*std::max(0.0, final_shrink_cells_));

    // Final check: unknown policy per user param
    const bool old_unknown_final = OG.unknown_as_occupied();
    OG.set_unknown_as_occupied(!final_allow_unknown_); // true -> unknown blocks; false -> allowed

    size_t new_safe_n = validPrefixCount(path_, w_cells, l_cells);

    // Fallback: if the first pose(s) collide, strip a prefix (configurable length in meters)
    if (new_safe_n == 0 && final_strip_prefix_m_ > 0.0) {
      const size_t try_pts = pointsForMeters(path_, final_strip_prefix_m_);
      size_t strip = 0;
      while (strip < try_pts && strip < path_.size()) {
        const auto& pt = path_[strip];
        if (poseFootprintIsFree(pt.x, pt.y, pt.psi, w_cells, l_cells)) break;
        ++strip;
      }
      if (strip > 0 && strip < path_.size()) {
        adore::TrajectoryVector tmp(path_.begin() + strip, path_.end());
        size_t tmp_safe = validPrefixCount(tmp, w_cells, l_cells);
        if (tmp_safe > 0) {
          RCLCPP_WARN(this->get_logger(),
            "Start blocked: stripped %zu poses (%.2fm) to pass final footprint check (shrink=%.2f cells).",
            strip, pathLengthMetersN(path_, strip), final_shrink_cells_);
          path_.swap(tmp);
          new_safe_n = tmp_safe;
        }
      }
    }

    OG.set_unknown_as_occupied(old_unknown_final);

    if (new_safe_n == 0) {
      RCLCPP_ERROR(this->get_logger(),
        "New path fails FINAL footprint check at start (unknowns %s, shrink=%.2f cells). Keeping old path.",
        final_allow_unknown_ ? "ALLOWED" : "BLOCKED", final_shrink_cells_);
      if (!active_path_.empty()) publishPath(active_path_);
      return;
    }

    const bool new_full_valid = (new_safe_n == path_.size());

    // --- Compute goal_changed once (for anti-flap and truncation gating)
    bool goal_changed_for_cycle = false;
    if (have_last_published_goal_) {
      double old_cx, old_cy; worldToCell(last_published_goal_pose_.position.x,
                                         last_published_goal_pose_.position.y, old_cx, old_cy);
      double new_cx, new_cy; worldToCell(goal_pose_.position.x,
                                         goal_pose_.position.y, new_cx, new_cy);
      goal_changed_for_cycle = (std::hypot(new_cx - old_cx, new_cy - old_cy) > 0.5); // ~half a cell
    }

    // --- Anti-flap: compare with current active_path_ ---
    bool accept_new = true;
    if (keep_old_if_valid_prefix_ && !active_path_.empty())
    {
      size_t old_safe_n = validPrefixCount(active_path_, w_cells, l_cells);
      const size_t lock_pts_new = pointsForMeters(path_,        lock_prefix_m_);
      const size_t lock_pts_old = pointsForMeters(active_path_, lock_prefix_m_);
      const size_t lock_pts = std::min(lock_pts_new, lock_pts_old);
      const bool old_has_valid_prefix = (old_safe_n >= lock_pts);

      const double rms = prefixRMSDistanceMeters(active_path_, path_, lock_pts);

      const double new_len = pathLengthMeters(path_);
      const double remaining_old = remainingLengthMeters(active_path_);
      const double improvement_m = remaining_old - new_len;

      // When goal changed, bypass improvement/deviation gating
      const bool improves_enough = goal_changed_for_cycle ? true : (improvement_m >= require_improvement_m_);

      if (!goal_changed_for_cycle && old_has_valid_prefix && (rms > max_prefix_deviation_m_) && !improves_enough) {
        accept_new = false;
        RCLCPP_INFO(this->get_logger(),
          "Keeping old path (prefix valid, new deviates %.2fm > %.2fm and improvement %.2fm < %.2fm).",
          rms, max_prefix_deviation_m_, improvement_m, require_improvement_m_);
      } else if (goal_changed_for_cycle) {
        RCLCPP_INFO(this->get_logger(), "Goal changed -> bypass stickiness and accept new path.");
      }
    }

    if (!accept_new) {
      publishPath(active_path_);
      return;
    }

    // If only a prefix of the NEW path is safe, optionally publish truncated prefix
    if (!new_full_valid)
    {
      const double ratio = static_cast<double>(new_safe_n) / static_cast<double>(path_.size());
      const double len_m = pathLengthMetersN(path_, new_safe_n);
      const double clamped_ratio_thr = std::max(0.0, std::min(1.0, min_publish_ratio_));

      // Allow truncated publish if goal changed, even if thresholds aren't met
      const bool allow_truncated = goal_changed_for_cycle;

      if (!allow_truncated &&
          (!publish_truncated_path_
           || new_safe_n < static_cast<size_t>(min_publish_points_)
           || len_m      < min_publish_length_m_
           || ratio      < clamped_ratio_thr))
      {
        RCLCPP_WARN(this->get_logger(),
          "Truncated new path below thresholds (pts %zu, len %.2fm, ratio %.0f%%). Keeping old.",
          new_safe_n, len_m, 100.0*ratio);
        if (!active_path_.empty()) publishPath(active_path_);
        return;
      }

      if (!allow_truncated) {
        RCLCPP_WARN(this->get_logger(),
          "Publishing truncated new path: %zu/%zu (%.2fm).",
          new_safe_n, path_.size(), len_m);
      } else {
        RCLCPP_WARN(this->get_logger(),
          "Goal changed → publishing truncated new path: %zu/%zu (%.2fm).",
          new_safe_n, path_.size(), len_m);
      }
      path_.resize(new_safe_n);
    }

    // Accept and publish the (possibly truncated) new path
    active_path_ = path_;
    publishPath(active_path_);

    // Start warmup re-publish burst so late subscribers can't miss the first plan
    warmup_until_      = this->now() + rclcpp::Duration::from_seconds(std::max(0.1, warmup_republish_s_));
    last_publish_time_ = this->now();

    // Record the goal we just published for anti-flap goal-change detection
    last_published_goal_pose_ = goal_pose_;
    have_last_published_goal_ = true;

    // Update last planned start after ADOPTION (not before)
    last_planned_start_.pose.pose    = start_pose_;
    last_planned_start_.header.stamp = this->now();

    // Goal hysteresis: if we are within goal tolerances now, freeze replanning
    if (withinGoalTolerancesWorld()) {
      goal_freeze_until_ = this->now() + rclcpp::Duration::from_seconds(std::max(0.0, post_goal_freeze_s_));
      if (!publish_when_goal_reached_) {
        RCLCPP_INFO(this->get_logger(),
          "Within goal tolerances (pos<=%.2fm, yaw<=%.1fdeg). Freezing replans for %.1fs and pausing publish.",
          goal_pos_tol_m_, goal_yaw_tol_deg_, post_goal_freeze_s_);
      } else {
        RCLCPP_INFO(this->get_logger(),
          "Within goal tolerances; freezing replans for %.1fs but continuing to publish last path.",
          post_goal_freeze_s_);
      }
    }
  }

  void publishPath(const adore::TrajectoryVector& P)
  {
    if (P.empty()) return;

    // If we decided to not publish when goal reached and still inside freeze window, skip.
    if ((this->now() < goal_freeze_until_) && !publish_when_goal_reached_) {
      return;
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = world_frame_;

    auto push_world_pose = [&](double wx, double wy, double yaw){
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path_msg.header;
      ps.pose.position.x = wx;
      ps.pose.position.y = wy;
      ps.pose.position.z = 0.0;
      tf2::Quaternion q; q.setRPY(0, 0, yaw);
      ps.pose.orientation = tf2::toMsg(q);
      path_msg.poses.push_back(ps);
    };

    // Convert first/last planned points to world for anchoring checks
    double first_wx, first_wy, last_wx, last_wy;
    cellToWorld(P.front().x, P.front().y, first_wx, first_wy);
    cellToWorld(P.back().x,  P.back().y,  last_wx,  last_wy);

    // --- Start anchoring: ensure first pose matches robot pose if needed ---
    const double robot_x = start_pose_.position.x;
    const double robot_y = start_pose_.position.y;
    const double robot_yaw = quatYaw(start_pose_.orientation);
    const double d0 = std::hypot(first_wx - robot_x, first_wy - robot_y);
    if (anchor_start_to_robot_ && d0 > anchor_start_gap_m_) {
      push_world_pose(robot_x, robot_y, robot_yaw);
      RCLCPP_DEBUG(this->get_logger(),
        "Anchored start: first path point was %.2fm away (>%.2fm).", d0, anchor_start_gap_m_);
    }

    // --- Planned poses ---
    for (const auto& pt : P)
    {
      double xw, yw; cellToWorld(pt.x, pt.y, xw, yw);
      push_world_pose(xw, yw, pt.psi);
    }

    // --- Optional goal anchoring ---
    if (anchor_goal_to_target_) {
      const double gx = goal_pose_.position.x, gy = goal_pose_.position.y;
      const double tail = std::hypot(last_wx - gx, last_wy - gy);
      if (tail > anchor_goal_gap_m_) {
        const double gyaw = quatYaw(goal_pose_.orientation);
        push_world_pose(gx, gy, gyaw);
        RCLCPP_DEBUG(this->get_logger(),
          "Anchored goal: last path point was %.2fm from goal (>%.2fm).", tail, anchor_goal_gap_m_);
      }
    }

    if (path_publisher_->get_subscription_count() == 0) {
      RCLCPP_DEBUG(this->get_logger(), "Publishing path but no subscribers yet (warmup/keepalive covers it).");
    }

    path_publisher_->publish(path_msg);
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
