#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <memory>
#include <iostream>

// You’ll also need to include adore headers when available
// #include <algo/graph_search.h>
// #include <env/path.h>

namespace adore
{
namespace if_ROS
{
class GraphSearchNode : public rclcpp::Node
{
public:
    GraphSearchNode()
    : Node("graph_search_node"), first_set(false), validStart(false), validEnd(false)
    {
        std::cout << "GraphSearchNode constructor called" << std::endl;

        // ROS2 publisher
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("Path", 10);

        // ROS2 subscription
        sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10,
            std::bind(&GraphSearchNode::receive_map_data, this, std::placeholders::_1));

        std::cout << "init graph search node" << std::endl;
    }

private:
    bool first_set;
    bool validStart, validEnd;

    // ROS2 members
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

    // Placeholder type — replace with your real path type
    // TrajectoryVector path;

    void receive_map_data(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::cout << "receive map data" << std::endl;
        if (!first_set)
        {
            std::cout << "receive map data first time" << std::endl;
            first_set = true;

            // TODO: Construct GraphSearch here when available
            // gs_ = new adore::apps::GraphSearch(...);

            publishPath();
        }
    }

    void publishPath()
    {
        nav_msgs::msg::Path msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";

        std::cout << "publish path" << std::endl;

        // Example: fake loop for demonstration
        for (int i = 0; i < 5; i++)
        {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = this->now();
            pose_stamped.header.frame_id = "map";
            pose_stamped.pose.position.x = i;
            pose_stamped.pose.position.y = i * 0.5;
            pose_stamped.pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, 0);
            pose_stamped.pose.orientation.x = q.x();
            pose_stamped.pose.orientation.y = q.y();
            pose_stamped.pose.orientation.z = q.z();
            pose_stamped.pose.orientation.w = q.w();

            msg.poses.push_back(pose_stamped);
        }

        path_publisher_->publish(msg);
    }
};
} // namespace if_ROS
} // namespace adore

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<adore::if_ROS::GraphSearchNode>());
    rclcpp::shutdown();
    return 0;
}
