#pragma once

#include <env/occupancy_grid.h>
#include <eigen3/Eigen/Core>
#include <env/node.h>
#include <env/search_grid.h>
#include <hybrid_A_star.h>
#include <env/OccupancyGrid.h>
#include <adore/fun/collision_check_offline.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <env/trajectory_smoothing.h>
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <boost/container/vector.hpp>
#include <env/path.h>
#include <adore_if_ros_msg/msg/point_array2_d.hpp>
#include <adore_if_ros_msg/msg/point_with_rotation2_d.hpp>
#include <rclcpp/rclcpp.hpp>

namespace adore
{
namespace apps
{

class GraphSearch
{
public:
    GraphSearch(const nav_msgs::msg::OccupancyGrid::SharedPtr &msg,
                int test,
                uint32_t height,
                uint32_t width,
                rclcpp::Node* parentnode)
        : grid_height(height), grid_width(width), node_(parentnode)
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

        path_publisher_ = node_->create_publisher<adore_if_ros_msg::msg::PointArray2D>("Path", 10);

        start_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "StartPose", 1,
            std::bind(&GraphSearch::receiveStartPose, this, std::placeholders::_1));

        end_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "EndPose", 1,
            std::bind(&GraphSearch::receiveEndPose, this, std::placeholders::_1));
    }

    void update()
    {
        RCLCPP_INFO(node_->get_logger(), "GraphSearch update triggered");

        while (iteration < 2 && validStart && validEnd)
        {
            start = std::chrono::system_clock::now();

            path = h_A_star->plan(&NH_GRID, &OG, cco, &Start, &End,
                                  HeadingResolution, 1000, vehicleWidth, vehicleLength);

            end = std::chrono::system_clock::now();
            time1 = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
            avg_time += time1;

            RCLCPP_INFO(node_->get_logger(), "Elapsed time (us): %d", time1);
            RCLCPP_INFO(node_->get_logger(), "Average time (ms): %.3f", (avg_time / iteration) / 1000.0);

            iteration++;

            if (!path.empty())
                publishPath();
        }
    }

private:
    // ROS2 publishers/subscribers
    rclcpp::Publisher<adore_if_ros_msg::msg::PointArray2D>::SharedPtr path_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr end_pose_sub_;
    rclcpp::Node* node_;

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
    TrajectoryVector path;
    double avg_time;
    float vehicleLength;
    float vehicleWidth;
    int iteration;
    int time1;
    std::chrono::system_clock::time_point start, end;

    void receiveStartPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        double r, p, y;
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3(q).getRPY(r, p, y);

        if (OG.check_valid_position(msg->pose.pose.position.y, msg->pose.pose.position.x))
        {
            validStart = Start.setPosition(msg->pose.pose.position.x,
                                           msg->pose.pose.position.y,
                                           y, grid_height, grid_width, Depth,
                                           adore::mad::CoordinateConversion::DegToRad(HeadingResolution));
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "Invalid Start Pose");
        }
    }

    void receiveEndPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        double r, p, y;
        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
        tf2::Matrix3x3(q).getRPY(r, p, y);

        if (OG.check_valid_position(msg->pose.position.y, msg->pose.position.x))
        {
            validEnd = End.setPosition(msg->pose.position.x,
                                       msg->pose.position.y,
                                       y, grid_height, grid_width, Depth,
                                       adore::mad::CoordinateConversion::DegToRad(HeadingResolution));
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "Invalid End Pose");
        }
    }

    void publishPath()
    {
        adore_if_ros_msg::msg::PointArray2D msg;
        for (const auto &point : path)
        {
            adore_if_ros_msg::msg::PointWithRotation2D msg_point;
            msg_point.x = point.x;
            msg_point.y = point.y;
            msg_point.rotation = point.psi;
            msg.points.push_back(msg_point);
        }
        path_publisher_->publish(msg);
    }
};

} // namespace apps
} // namespace adore
