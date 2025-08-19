#pragma once

#include "../env/occupancy_grid.h"
#include <eigen3/Eigen/Core>
#include "../env/node.h"
#include "../env/search_grid.h"
#include "hybrid_A_star.h"
//#include "env/OccupancyGrid.h"
#include "collision_check_offline.h"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include "../env/trajectory_smoothing.h"
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <boost/container/vector.hpp>
#include "../env/path.h"


namespace adore
{
//namespace apps
//{

class GraphSearch
{
public:

    GraphSearch(const nav_msgs::msg::OccupancyGrid::SharedPtr &msg,
                int test,
                uint32_t height,
                uint32_t width,
                )
        : grid_height(height), grid_width(width)
    {
        vehicleLength = 3.2f;
        vehicleWidth = 1.0f;

        smoothing = new fun::TrajectorySmoothing;
        h_A_star = new adore::fun::Hybrid_A_Star(smoothing);

        Depth = 360 / HeadingResolution;
        cco = new adore::fun::CollisionCheckOffline(2, 2, HeadingResolution, 10);

        OG.init(msg, grid_height, grid_width);
        OG.resize(grid_height, grid_width);
        NH_GRID.resize(height, width, Depth);
        h_A_star->setSize(height, width);

        avg_time = 0.0;
        iteration = 1;
    }

    void update(adore::fun::Node<3, double> Start, adore::fun::Node<3, double> End)
    {
        //RCLCPP_INFO(node_->get_logger(), "GraphSearch update triggered");

        while (iteration < 2 && validStart && validEnd)
        {
            start = std::chrono::system_clock::now();

            path = h_A_star->plan(&NH_GRID, &OG, cco, &Start, &End,
                                  HeadingResolution, 1000, vehicleWidth, vehicleLength);

            end = std::chrono::system_clock::now();
            time1 = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
            avg_time += time1;

            //RCLCPP_INFO(node_->get_logger(), "Elapsed time (us): %d", time1);
            //RCLCPP_INFO(node_->get_logger(), "Average time (ms): %.3f", (avg_time / iteration) / 1000.0);

            iteration++;

            if (!path.empty())
                //publishPath();
        }
    }
    TrajectoryVector path;

private:
    //rclcpp::Node* node_;

    // Algorithm members
    int grid_width;
    int grid_height;
    static constexpr int HeadingResolution = 45;
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

//} // namespace apps
} // namespace adore
