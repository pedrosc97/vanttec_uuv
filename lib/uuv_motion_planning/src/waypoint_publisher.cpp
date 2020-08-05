/** ----------------------------------------------------------------------------
 * @file: waypoint_publisher.cpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: Waypoint publisher class, used to test waypoint navigation.
 * -----------------------------------------------------------------------------
 * */

#include <waypoint_publisher.hpp>

WaypointPublisher::WaypointPublisher()
{ 
    this->trajectory_selector     = 0;
    this->path_publish_flag       = 0;
    this->circle_radius           = 1.0;
}

WaypointPublisher::~WaypointPublisher(){}

void WaypointPublisher::OnTrajectoryReceive(const std_msgs::UInt8& _trajectory)
{
    this->trajectory_selector     = (int) _trajectory.data;
    this->path_publish_flag       = 0;
}

void WaypointPublisher::OnRadiusReceive(const std_msgs::Float32& _radius)
{
    this->circle_radius           = (float) _radius.data;
}

void WaypointPublisher::WaypointSelection()
{
    if (this->path_publish_flag == 0)
    {
        switch((int) this->trajectory_selector)
        {
            case 0:
                this->waypoints = uuv_common::GenerateCircle(this->circle_radius, 1.2, 2.7, 0);
                this->waypoints.guidance_law = 1;
                break;
            case 1:
                this->waypoints = uuv_common::GenerateCircle(this->circle_radius, 1.2, 2.7, 0);
                this->waypoints.guidance_law = 2;
                break; 
            case 2:
                this->waypoints.guidance_law = 1;
                this->waypoints.waypoint_list_length = 2;
                this->waypoints.waypoint_list_x = {0,1};
                this->waypoints.waypoint_list_y = {0,2};
                this->waypoints.waypoint_list_z = {0,0};
                break;
            default:
                break;
        }
        
        this->path.header.stamp     = ros::Time::now();
        this->path.header.frame_id  = "world";
        this->path.poses.clear();

        for (int i = 0; i < this->waypoints.waypoint_list_length; i++)
        {
            geometry_msgs::PoseStamped      pose;

            pose.header.stamp       = ros::Time::now();
            pose.header.frame_id    = "world";
            pose.pose.position.x    = waypoints.waypoint_list_x[i];
            pose.pose.position.y    = -waypoints.waypoint_list_y[i];
            pose.pose.position.z    = -waypoints.waypoint_list_z[i];

            this->path.poses.push_back(pose);
        }
    }
}