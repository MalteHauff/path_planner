#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "../env/occupancy_grid.h"
#include <eigen3/Eigen/Core>
#include "../env/node.h"
#include "../env/search_grid.h"
#include "hybrid_A_star.h"
#include "collision_check_offline.h"
#include <boost/geometry.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include "../env/trajectory_smoothing.h"
#include <chrono>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <boost/container/vector.hpp>
#include "../env/path.h"
#include <iostream>

namespace adore
{
// GraphSearch: robust mapping world -> grid and safer initialization
class GraphSearch
{
public:
    // Constructor: msg is the ROS OccupancyGrid used to initialize OG
    GraphSearch(const nav_msgs::msg::OccupancyGrid::SharedPtr &msg,
                int test,
                uint32_t height,
                uint32_t width)
        : grid_height(height),
          grid_width(width)
    {
        vehicleLength = 3.2f;
        vehicleWidth = 1.0f;

        smoothing = new fun::TrajectorySmoothing;
        h_A_star = new adore::fun::Hybrid_A_Star(smoothing);

        // HeadingResolution is defined as static constexpr below (degrees)
        Depth = 360 / HeadingResolution;

        cco = new adore::fun::CollisionCheckOffline(2, 2, HeadingResolution, 10);

        // Initialize occupancy grid object (your OG type)
        OG.init(msg, grid_height, grid_width);
        OG.resize(grid_height, grid_width);

        // Save map info for conversions
        map_origin_x = msg->info.origin.position.x;
        map_origin_y = msg->info.origin.position.y;
        map_origin_orientation = msg->info.origin.orientation;
        resolution = msg->info.resolution;
        map_cols = msg->info.width;
        map_rows = msg->info.height;

        // debug print map info
        std::cerr << "[GraphSearch] map origin=(" << map_origin_x << "," << map_origin_y
                  << ") resolution=" << resolution
                  << " cols=" << map_cols << " rows=" << map_rows << "\n";

        NH_GRID.resize(height, width, Depth);
        h_A_star->setSize(height, width);

        avg_time = 0.0;
        iteration = 1;
    }

    
    void update(adore::fun::Node<3, double> Start, adore::fun::Node<3, double> End)
    {
        
        if (!validStart && !validEnd) {
            return;
        }

        // Print received Start/End raw values (helpful for debugging)
        std::cerr << "[GraphSearch] Received Start raw: x=" << Start.x << " y=" << Start.y << " psi=" << Start.psi << "\n";
        std::cerr << "[GraphSearch] Received End raw:   x=" << End.x << " y=" << End.y << " psi=" << End.psi << "\n";

        // Convert Start -> cell coords if necessary
        adore::fun::Node<3,double> StartCell;
        bool start_ok = worldOrCellToCell(Start, StartCell);
        if (!start_ok) {
            std::cerr << "[GraphSearch] Starts position invalid after conversion\n";
            validStart = false;
        } else {
            validStart = true;
        }

        // Convert End -> cell coords if necessary
        adore::fun::Node<3,double> EndCell;
        bool end_ok = worldOrCellToCell(End, EndCell);
        if (!end_ok) {
            std::cerr << "[GraphSearch] End position invalid after conversion\n";
            validEnd = false;
        } else {
            validEnd = true;
        }

        if (!validStart || !validEnd) {
            std::cerr << "[GraphSearch] invalidStart=" << validStart << " invalidEnd=" << validEnd << " -> skipping plan\n";
            return;
        }

        // Ready: call planner
        start = std::chrono::system_clock::now();

        path = h_A_star->plan(&NH_GRID, &OG, cco, &StartCell, &EndCell,
                              HeadingResolution, 1000, vehicleWidth, vehicleLength);

        end = std::chrono::system_clock::now();
        time1 = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        avg_time += time1;

        std::cerr << "[GraphSearch] Plan finished in " << time1 << " us. Path size: " << path.size() << "\n";

        iteration++;

        if (!path.empty()) {
            // publishPath();
            std::cerr << "[GraphSearch] Path found length: " << path.size() << "\n";
        } else {
            std::cerr << "[GraphSearch] Path is empty\n";
        }
    }

    // Expose the last computed path
    TrajectoryVector path;

private:
    // map / grid info
    double map_origin_x = 0.0;
    double map_origin_y = 0.0;
    geometry_msgs::msg::Quaternion map_origin_orientation;
    double resolution = 0.0;
    uint32_t map_cols = 0;
    uint32_t map_rows = 0;

    void worldToMapCell(double world_x, double world_y, double &cell_x, double &cell_y)
    {
        // Build origin transform
        tf2::Quaternion q(map_origin_orientation.x,
                          map_origin_orientation.y,
                          map_origin_orientation.z,
                          map_origin_orientation.w);
        tf2::Transform origin_tf(q, tf2::Vector3(map_origin_x, map_origin_y, 0.0));

        tf2::Vector3 p_world(world_x, world_y, 0.0);

        tf2::Transform inv = origin_tf.inverse();
        tf2::Vector3 p_map = inv * p_world;

        cell_x = p_map.x() / resolution; // column
        cell_y = p_map.y() / resolution; // row
    }

    
    bool worldOrCellToCell(const adore::fun::Node<3,double> &inNode, adore::fun::Node<3,double> &outCell)
    {
        double in_x = static_cast<double>(inNode.x);
        double in_y = static_cast<double>(inNode.y);

        bool likely_cell = (in_x >= 0.0 && in_x < static_cast<double>(map_cols)
                            && in_y >= 0.0 && in_y < static_cast<double>(map_rows));

        double cell_x = in_x;
        double cell_y = in_y;

        if (!likely_cell) {
            // Treat as world coords: convert
            worldToMapCell(in_x, in_y, cell_x, cell_y);
            std::cerr << "[GraphSearch] Converted world ("<<in_x<<","<<in_y<<") -> cell ("<<cell_x<<","<<cell_y<<")\n";
        } else {
            std::cerr << "[GraphSearch] Input appears already cell coords ("<<in_x<<","<<in_y<<")\n";
        }

        // Now call setPosition on outCell using Width=rows, Length=cols, Depth and HeadingResolutionRad
        double HeadingResolutionRad = double(HeadingResolution) * M_PI / 180.0;
        bool ok = outCell.setPosition(cell_x, cell_y, inNode.psi, /*Width=*/map_rows, /*Length=*/map_cols, /*Depth=*/Depth, HeadingResolutionRad);

        if (!ok) {
            std::cerr << "[GraphSearch] setPosition rejected cell coords ("<<cell_x<<","<<cell_y<<")\n";
            return false;
        }

        // print resulting (cell) indices & psi
        std::cerr << "[GraphSearch] sestsddPosition succeeded -> node cell (x="<<outCell.x<<", y="<<outCell.y<<", psi="<<outCell.psi<<")\n";
        return true;
    }

    // Members
    int grid_width;
    int grid_height;
    static constexpr int HeadingResolution = 45; // degrees
    static constexpr int nH_Type = 3;
    int Depth;
    adore::env::OccupanyGrid OG;
    adore::fun::GRID<adore::fun::Node<nH_Type, double>> NH_GRID;
    adore::fun::Hybrid_A_Star* h_A_star;
    adore::fun::Node<3, double> Start;
    adore::fun::Node<3, double> End;
    bool validStart = false;
    bool validEnd = false;
    adore::fun::CollisionCheckOffline* cco;
    fun::TrajectorySmoothing* smoothing;

    double avg_time;
    float vehicleLength;
    float vehicleWidth;
    int iteration;
    int time1;
    std::chrono::system_clock::time_point start, end;
};

} // namespace adore
