/********************************************************************************
 * occupancy_grid.h -- Relaxed OccupanyGrid with UNKNOWN-as-free default
 *  - Grid values: -1 (unknown), 0 (free), 1 (occupied)
 *  - World<->grid conversion using origin and yaw
 *  - Provides obstacle API used by trajectory_smoothing:
 *      - nested struct _Obstacle with circle decomposition
 *      - get_obstacles()
 *      - get_ellipse_r(beta, a, b)
 *      - static transformation(x,y,theta, dx, dy)
 *  - Adds OG.init(msg, H, W) to match graph_search_node.cpp
 ********************************************************************************/
#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <algorithm>
#include <boost/container/vector.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace adore {
namespace env {

class OccupanyGrid
{
public:
    // --- Nested types for smoothing / obstacles ---
    struct _Circle {
        double x{0.0};
        double y{0.0};
        double r{0.0};
    };

    struct _Obstacle {
        double x{0.0};
        double y{0.0};
        double length{0.0};   // major axis (m)
        double width{0.0};    // minor axis (m)
        double alpha{0.0};    // orientation (rad)
        boost::container::vector<_Circle> circles; // circle decomposition (optional)
    };

    using obstacleList = boost::container::vector<_Obstacle>;

public:
    // Grid values: -1 unknown, 0 free, 1 occupied
    Eigen::MatrixXi Grid;

    // Map metadata
    std::uint32_t width{0};
    std::uint32_t height{0};
    double        resolution{0.05};   // meters / cell
    double        origin_x{0.0};      // meters
    double        origin_y{0.0};      // meters
    double        origin_yaw{0.0};    // radians

    // Obstacles (for smoothing helpers)
    obstacleList obstacles_;

    // Unknown policy (default relaxed → unknown treated as free)
    void set_unknown_as_occupied(bool v) { unknown_is_occupied_ = v; }
    bool unknown_as_occupied() const { return unknown_is_occupied_; }

    OccupanyGrid() = default;

    // Accessors used by trajectory_smoothing
    obstacleList&       get_obstacles()       { return obstacles_; }
    const obstacleList& get_obstacles() const { return obstacles_; }

    // Ellipse radius at angle beta (in obstacle local frame), with semi-axes a=length/2, b=width/2
    static double get_ellipse_r(double beta, double a, double b)
    {
        const double aa = std::max(1e-9, a);
        const double bb = std::max(1e-9, b);
        const double c = std::cos(beta);
        const double s = std::sin(beta);
        const double denom = std::sqrt((bb*bb*c*c) + (aa*aa*s*s));
        return (aa*bb) / denom;
    }

    // Rigid-body transform of a local offset (dx,dy) by pose (x,y,theta)
    static Eigen::Vector2d transformation(double x, double y, double theta, double dx, double dy)
    {
        const double cx =  std::cos(theta) * dx - std::sin(theta) * dy;
        const double cy =  std::sin(theta) * dx + std::cos(theta) * dy;
        return Eigen::Vector2d(x + cx, y + cy);
    }

    // Initialize from a ROS OccupancyGrid message (matches GraphSearchNode::receive_map_data usage)
    void init(const nav_msgs::msg::OccupancyGrid::SharedPtr& msg,
              std::uint32_t /*H*/, std::uint32_t /*W*/,
              int occ_threshold = 50)
    {
        width      = msg->info.width;
        height     = msg->info.height;
        resolution = msg->info.resolution;
        origin_x   = msg->info.origin.position.x;
        origin_y   = msg->info.origin.position.y;

        // yaw from quaternion
        {
            const auto &q = msg->info.origin.orientation;
            tf2::Quaternion tq(q.x, q.y, q.z, q.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
            origin_yaw = yaw;
        }

        Grid = Eigen::MatrixXi::Zero(static_cast<int>(height), static_cast<int>(width));
        for (std::uint32_t y = 0; y < height; ++y) {
            for (std::uint32_t x = 0; x < width; ++x) {
                const std::size_t idx = static_cast<std::size_t>(x + y * width);
                const int v = static_cast<int>(msg->data[idx]);
                if (v < 0)                 Grid(static_cast<int>(y), static_cast<int>(x)) = -1; // unknown
                else if (v >= occ_threshold) Grid(static_cast<int>(y), static_cast<int>(x)) =  1; // occupied
                else                        Grid(static_cast<int>(y), static_cast<int>(x)) =  0; // free
            }
        }
    }

    // Inflate obstacles by N cells (square dilation). Unknowns are NOT inflated.
    void inflate(int radius_cells)
    {
        if (radius_cells <= 0 || width == 0 || height == 0) return;

        Eigen::MatrixXi out = Grid;
        for (int r = 0; r < static_cast<int>(height); ++r) {
            for (int c = 0; c < static_cast<int>(width); ++c) {
                if (Grid(r, c) != 1) continue; // only inflate occupied

                const int r0 = std::max(0, r - radius_cells);
                const int r1 = std::min(static_cast<int>(height) - 1, r + radius_cells);
                const int c0 = std::max(0, c - radius_cells);
                const int c1 = std::min(static_cast<int>(width)  - 1, c + radius_cells);

                for (int rr = r0; rr <= r1; ++rr)
                    for (int cc = c0; cc <= c1; ++cc)
                        out(rr, cc) = 1;
            }
        }
        Grid.swap(out);
    }

    // Convert world [m] -> grid [row, col] (integers). Returns true if inside.
    inline bool world_to_grid(double wx, double wy, int& row, int& col) const
    {
        // transform world -> map frame (rotate by -origin_yaw, then translate)
        const double cx = wx - origin_x;
        const double cy = wy - origin_y;
        const double ca = std::cos(-origin_yaw);
        const double sa = std::sin(-origin_yaw);
        const double mx =  ca * cx - sa * cy;
        const double my =  sa * cx + ca * cy;

        col = static_cast<int>(std::floor(mx / resolution));
        row = static_cast<int>(std::floor(my / resolution));
        return point_inside(row, col);
    }

    // Convert grid [row, col] -> world [m]
    inline void grid_to_world(int row, int col, double& wx, double& wy) const
    {
        const double mx = (static_cast<double>(col) + 0.5) * resolution;
        const double my = (static_cast<double>(row) + 0.5) * resolution;
        const double ca = std::cos(origin_yaw);
        const double sa = std::sin(origin_yaw);
        const double cx =  ca * mx - sa * my;
        const double cy =  sa * mx + ca * my;
        wx = cx + origin_x;
        wy = cy + origin_y;
    }

    // Bounds check
    inline bool point_inside(int row, int col) const
    {
        return (row >= 0 && row < static_cast<int>(height)
             && col >= 0 && col < static_cast<int>(width));
    }

    // Valid cell: inside & not occupied & (unknown free per policy)
    inline bool check_valid_position(int row, int col) const
    {
        if (!point_inside(row, col)) return false;
        const int v = Grid(row, col);
        if (v == 1)  return false;                  // occupied
        if (v == -1) return !unknown_is_occupied_;  // unknown -> free if policy false
        return true;                                // free
    }

private:
    bool unknown_is_occupied_{false}; // RELAXED DEFAULT (planning allows unknown)
};

} // namespace env
} // namespace adore
