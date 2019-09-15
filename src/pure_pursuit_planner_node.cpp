// Headers in ros
#include <ros/ros.h>

// Headers in this package
#include <pure_pursuit_planner/pure_pursuit_planner.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pure_pursuit_planner_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    PurePursuitPlanner planner(nh,pnh);
    ros::spin();
    return 0;
}