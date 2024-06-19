#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"

// Placeholder function for the pathfinding algorithm
nav_msgs::Path findPath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, const std::vector<geometry_msgs::Point>& obstacles)
{
    // This function should be implemented to find a path given the start, goal, and obstacles.
    nav_msgs::Path path;
    // Dummy implementation returning an empty path
    return path;
}

ros::Publisher marker_pub;

void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    ROS_INFO("Received a new path with %lu poses", msg->poses.size());

    // // Print the actual path
    // for (size_t i = 0; i < msg->poses.size(); ++i)
    // {
    //     ROS_INFO("Pose %lu: [x: %f, y: %f, z: %f]", 
    //              i, 
    //              msg->poses[i].pose.position.x, 
    //              msg->poses[i].pose.position.y, 
    //              msg->poses[i].pose.position.z);
    // }

    if (msg->poses.empty())
    {
        ROS_WARN("Received path is empty.");
        return;
    }

    geometry_msgs::PoseStamped start = msg->poses.front();
    geometry_msgs::PoseStamped goal = msg->poses.back();

    // Iterate through the path to add obstacles and find the longest resulting path
    std::vector<geometry_msgs::Point> best_obstacles;
    double max_path_length = 0.0;

    for (size_t i = 0; i < msg->poses.size(); i += 5)
    {
        std::vector<geometry_msgs::Point> obstacles;
        geometry_msgs::Point obstacle;
        obstacle.x = msg->poses[i].pose.position.x;
        obstacle.y = msg->poses[i].pose.position.y;
        obstacle.z = msg->poses[i].pose.position.z;
        obstacles.push_back(obstacle);

        // Find the path with this obstacle
        nav_msgs::Path path = findPath(start, goal, obstacles);

        // Calculate the path length
        double path_length = 0.0;
        for (size_t j = 1; j < path.poses.size(); ++j)
        {
            double dx = path.poses[j].pose.position.x - path.poses[j-1].pose.position.x;
            double dy = path.poses[j].pose.position.y - path.poses[j-1].pose.position.y;
            path_length += std::sqrt(dx * dx + dy * dy);
        }

        if (path_length > max_path_length)
        {
            max_path_length = path_length;
            best_obstacles = obstacles;
        }
    }

    // Create a marker to visualize the best obstacles
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "obstacles";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    // Set marker scale
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // Set marker color
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // Add the best obstacles to the marker
    marker.points = best_obstacles;

    // Publish the marker
    marker_pub.publish(marker);
}

int main(int argc, char **argv)
{
    // Initialize the ROS system
    ros::init(argc, argv, "obstacle_attack");

    // Establish this program as a ROS node
    ros::NodeHandle nh;

    // Create a subscriber to the path planner topic
    ros::Subscriber sub = nh.subscribe("/move_base/GraphPlanner/plan", 1000, pathCallback);

    // Create a publisher for visualization markers
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    // Enter a loop, pumping callbacks
    ros::spin();

    return 0;
}
