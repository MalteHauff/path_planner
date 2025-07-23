/********************************************************************************
 * Copyright (C) 2017-2020 German Aerospace Center (DLR). 
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the 
 *
 * SPDX-License-Identifier: EPL-2.0 
 *
 * Contributors: 
 *   Reza Dariani- initial API and implementation
 ********************************************************************************/
//#include <adore_if_ros_scheduling/baseapp.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <stdint.h>
//#include <adore/apps/gap_provider.h>
#include <algo/graph_search.h>
//#include <adore_if_ros/factorycollection.h>
#include <geometry_msgs/Pose.h>
#include <env/path.h>

namespace adore
{
    namespace if_ROS
    {
    class GraphSearchNode : public rclcpp::Node {
        public:
            GraphSearchNode() : Node("graph_search_node") {
                std::cout<<"GraphSearchNode constructor called"<<std::endl;
                path_publisher = this->create_publisher<adore_if_ros_msg::PointArray2D>("Path", 10);
                sub = this->create_subscription<nav_msgs::OccupancyGrid>("map", 10, &GraphSearchNode::receive_map_data, this);
                first_set = false;
                std::cout<<"init graph search node"<<std::endl;
            }
            adore::apps::GraphSearch* gs_;
            //ros::NodeHandle node;
            ros::Subscriber sub;
            ros::Publisher path_publisher;
            TrajectoryVector path;
            GraphSearchNode() {}
            bool validStart, validEnd;
            /*void init(int argc, char **argv, double rate, std::string nodename)
            {
                
                adore_if_ros_scheduling::Baseapp::init(argc, argv, rate, nodename);
                adore_if_ros_scheduling::Baseapp::initSim();
                FactoryCollection::init(getRosNodeHandle());
                //ros::NodeHandle node;
                sub = getRosNodeHandle()->subscribe<nav_msgs::OccupancyGrid>("map",10, &GraphSearchNode::receive_map_data, this);
                path_publisher = getRosNodeHandle()->advertise<adore_if_ros_msg::PointArray2D>("Path",10,this);
            
                first_set = false;
                std::cout<<"init graph search node"<<std::endl;
            }*/
        private:
            bool first_set;
            void receive_map_data(const nav_msgs::OccupancyGrid::ConstPtr &msg){
                std::cout<<"receive map data"<<std::endl;
                if(!first_set){
                    std::cout<<"receive map data first time"<<std::endl;
                    first_set=true;
                    int test=1;
                    gs_=  new adore::apps::GraphSearch(msg, test, (uint32_t)(msg->info.height), (uint32_t)(msg->info.width), Baseapp::getRosNodeHandle()); //->init_gs(data,1,1);//(new_data, (uint32_t)(msg->info.height), (uint32_t)(msg->info.width));//, msg->data, (uint32_t)msg->info.height, (uint32_t)msg->info.width)
                    // timer callbacks
                    std::function<void()>run_fcn(std::bind(&adore::apps::GraphSearch::update, gs_));
                    //adore_if_ros_scheduling::Baseapp::addTimerCallback(run_fcn);
                    path = gs_->path;
                    int i = 0;
                    std::cout << "befor publish path"<<std::endl;
                    for (const auto &point : path)
                    {
                        std::cout << "a star Point : "<<point.x << " "  << point.y<< std::endl;
                        i++;
                        if(i > 10) break;
                    } 
                    publishPath();
                }
                
            }
             void receiveStartPose(geometry_msgs::Pose msg)
            {
                        double r,p,y;
                        //tf::Matrix3x3(tf::Quaternion(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)).getRPY(r,p,y);
                        validStart = true;//Start.setPosition(msg.position.x,msg.position.y,y,Width,Length,Depth,adore::mad::CoordinateConversion::DegToRad(HeadingResolution), figure3);
                        //Start.print();
            }  
            void receiveEndPose(geometry_msgs::Pose msg)
            {
                        double r,p,y;
                        //tf::Matrix3x3(tf::Quaternion(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)).getRPY(r,p,y);
                        validEnd = true; //End.setPosition(msg.position.x,msg.position.y,y,Width,Length,Depth, adore::mad::CoordinateConversion::DegToRad(HeadingResolution),  figure3);
                        //End.print();
            } 

            void publishPath()
{
    nav_msgs::Path msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map"; // or your desired frame

    int i = 0;
    std::cout << "publish path" << std::endl;
    for (const auto &point : path)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = this->now();
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = point.x;
        pose_stamped.pose.position.y = point.y;
        pose_stamped.pose.position.z = 0.0;

        // Convert yaw (psi) to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, point.psi);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        std::cout << "GraphSearch Test Point : " << point.x << " " << point.y << std::endl;
        std::cout << "Msg Point : " << pose_stamped.pose.position.x << " " << pose_stamped.pose.position.y << std::endl;
        i++;
        //if (i > 10) break;

        msg.poses.push_back(pose_stamped);
    }

    path_publisher.publish(msg);
}   
                
        };
    } // namespace if_ROS
} // namespace adore

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GraphSearchNode>());
  rclcpp::shutdown();
  return 0;
}
