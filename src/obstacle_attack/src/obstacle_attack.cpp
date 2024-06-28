#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

// #include <../../core/global_planner/graph_planner/include/graph_planner.h>
// using namespace graph_planner;

geometry_msgs::PoseStamped last_goal;
ros::Publisher marker_pub;

bool isSameGoal(const geometry_msgs::PoseStamped& goal1, const geometry_msgs::PoseStamped& goal2)
{
    const double epsilon = 1e-6;
    return std::fabs(goal1.pose.position.x - goal2.pose.position.x) < epsilon &&
           std::fabs(goal1.pose.position.y - goal2.pose.position.y) < epsilon;
}

double calculatePathLength(const std::vector<geometry_msgs::PoseStamped>& path)
{
    double path_length = 0.0;
    for (size_t j = 1; j < path.size(); ++j)
    {
        double dx = path[j].pose.position.x - path[j-1].pose.position.x;
        double dy = path[j].pose.position.y - path[j-1].pose.position.y;
        path_length += std::sqrt(dx * dx + dy * dy);
    }
    return path_length;
}

void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    if (msg->poses.empty())
    {
        ROS_WARN("Received path is empty.");
        return;
    }

    geometry_msgs::PoseStamped start = msg->poses.front();
    geometry_msgs::PoseStamped goal = msg->poses.back();

    if (isSameGoal(goal, last_goal))
    {
        // ROS_WARN("Callback already called for the current goal. Skipping further execution.");
        return;
    }

    ROS_INFO("Received a new path with %lu poses", msg->poses.size());

    // Print the actual path
    for (size_t i = 0; i < msg->poses.size(); ++i)
    {
        ROS_INFO("Pose %lu: [x: %f, y: %f]",
                 i, 
                 msg->poses[i].pose.position.x, 
                 msg->poses[i].pose.position.y);
    }

    // Calculate the current path length
    double default_path_length = calculatePathLength(msg->poses);

    // GraphPlanner planner;
    // std::vector<geometry_msgs::PoseStamped> path;

    // // Iterate through the path to iterate obstacles and find the longest resulting path
    // geometry_msgs::Point best_obstacle;
    // double max_path_length = default_path_length;

    // for (size_t i = 0; i < msg->poses.size(); ++i)
    // {
    //     geometry_msgs::Point obstacle;
    //     obstacle.x = msg->poses[i].pose.position.x;
    //     obstacle.y = msg->poses[i].pose.position.y;
    //     obstacle.z = msg->poses[i].pose.position.z;

    //     // Find the path with this obstacle using A* algorithm
    //     bool success = planner.makePlan(start, goal, path);

    //     if (!success) {
    //         ROS_WARN("The path finding algorithm failed to find a path.");
    //         continue;
    //     }

    //     // Calculate the path length
    //     double path_length = calculatePathLength(path);

    //     if (path_length > max_path_length)
    //     {
    //         max_path_length = path_length;
    //         best_obstacle = obstacle;
    //     }
    // }

    // // Create a marker to visualize the best obstacles
    // visualization_msgs::Marker marker;
    // marker.header.frame_id = "map";
    // marker.header.stamp = ros::Time::now();
    // marker.ns = "obstacles";
    // marker.id = 0;
    // marker.type = visualization_msgs::Marker::SPHERE_LIST;
    // marker.action = visualization_msgs::Marker::ADD;
    // marker.pose.orientation.w = 1.0;

    // // Set marker scale
    // marker.scale.x = 0.2;
    // marker.scale.y = 0.2;
    // marker.scale.z = 0.2;

    // // Set marker color
    // marker.color.r = 1.0;
    // marker.color.g = 0.0;
    // marker.color.b = 0.0;
    // marker.color.a = 1.0;

    // // Add the best obstacle to the marker
    // marker.pose.position = best_obstacle;

    // // Publish the marker
    // marker_pub.publish(marker);

    // Update the last goal and set the flag to true
    last_goal = goal;
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