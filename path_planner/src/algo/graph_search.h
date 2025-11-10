#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <cmath>

#include "../env/occupancy_grid.h"
#include <eigen3/Eigen/Core>
#include "../env/node.h"
#include "../env/search_grid.h"
#include "hybrid_A_star.h"
#include "collision_check_offline.h"
#include <boost/geometry.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <vector>
#include <iostream>
#include <chrono>

namespace adore {

class GraphSearch
{
public:
    GraphSearch()
      : grid_height(0), grid_width(0),
        vehicleLength(3.2f), vehicleWidth(1.0f),
        HeadingResolution(45), Depth(0),
        h_A_star(nullptr), cco(nullptr), smoothing(nullptr),
        avg_time(0.0), iteration(1)
    {}

    ~GraphSearch()
    {
        if (h_A_star) delete h_A_star;
        if (cco) delete cco;
        if (smoothing) delete smoothing;
    }

    void receive_map_data(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        grid_height = msg->info.height;
        grid_width = msg->info.width;

        smoothing = new adore::fun::TrajectorySmoothing();
        h_A_star = new adore::fun::Hybrid_A_Star(smoothing);

        // HeadingResolution is defined as static constexpr below (degrees)
        Depth = static_cast<int>(std::round(360.0 / HeadingResolution));

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
        NH_GRID.set_owns_pointers(false);
        h_A_star->setSize(height, width);

        avg_time = 0.0;
        iteration = 1;
    }

    void setSize(int height_, int width_)
    {
        height = height_;
        width = width_;
    }

    void setStart(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr &start_pose)
    {
        double yaw = poseYaw(start_pose->pose.pose.orientation);
        double wx = start_pose->pose.pose.position.x;
        double wy = start_pose->pose.pose.position.y;

        int row=-1,col=-1;
        if (!OG.worldToGrid(wx, wy, row, col)) {
            std::cerr << "[GraphSearch] Start position outside map bounds\n";
            validStart = false; return;
        }
        if (!OG.check_valid_position(row, col)) {
            std::cerr << "[GraphSearch] Start position invalid after conversion\n";
            validStart = false; return;
        }

        double cell_x = (wx - map_origin_x) / resolution;
        double cell_y = (wy - map_origin_y) / resolution;
        Start.setPosition(cell_x, cell_y, yaw);
        validStart = true;
    }

    void setGoal(const geometry_msgs::msg::PoseStamped::SharedPtr &goal_pose)
    {
        double wx = goal_pose->pose.position.x;
        double wy = goal_pose->pose.position.y;

        int row=-1,col=-1;
        if (!OG.worldToGrid(wx, wy, row, col)) {
            std::cerr << "[GraphSearch] End position outside map bounds\n";
            validEnd = false; return;
        }
        if (!OG.check_valid_position(row, col)) {
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

        path =
            h_A_star->plan(&NH_GRID, &OG, cco, &Start, &End, HeadingResolution, 1000, vehicleWidth, vehicleLength);

        end = std::chrono::system_clock::now();

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        avg_time = (avg_time * iteration + elapsed) / (iteration + 1);
        iteration++;
        std::cerr << "[GraphSearch] plan elapsed(ms)=" << elapsed << " avg(ms)=" << avg_time << "\n";
    }

private:
    static double poseYaw(const geometry_msgs::msg::Quaternion &q)
    {
        tf2::Quaternion tq(q.x, q.y, q.z, q.w);
        double r,p,y; tf2::Matrix3x3(tq).getRPY(r,p,y);
        return y;
    }

private:
    adore::env::OccupanyGrid OG;
    adore::fun::GRID<adore::fun::Node<3,double>> NH_GRID;

    adore::fun::Hybrid_A_Star *h_A_star;
    adore::fun::CollisionCheckOffline *cco;
    adore::fun::TrajectorySmoothing *smoothing;

    adore::fun::Node<3,double> Start, End;
    adore::fun::Path path;

    int height{0}, width{0};
    int grid_height{0}, grid_width{0};
    int map_rows{0}, map_cols{0};

    float vehicleLength;
    float vehicleWidth;

    int HeadingResolution;
    int Depth;

    bool validStart{false}, validEnd{false};

    double map_origin_x{0.0}, map_origin_y{0.0};
    geometry_msgs::msg::Quaternion map_origin_orientation{};
    double resolution{0.0};

    double avg_time;
    int iteration;

    std::chrono::system_clock::time_point start, end;
};

} // namespace adore
