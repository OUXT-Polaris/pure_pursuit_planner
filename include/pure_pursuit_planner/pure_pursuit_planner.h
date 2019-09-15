#ifndef PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_H_INCLUDED
#define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_H_INCLUDED

// Headers in ROS
#include <ros/ros.h>
#include <usv_navigation_msgs/Path.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>
#include <quaternion_operation/quaternion_operation.h>

// Headers in this package
#include <pure_pursuit_planner/PurePursuitPlannerConfig.h>

// Headers in Boost
#include <boost/optional.hpp>

// Headers in STL
#include <memory>

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> SyncPolicy;

class PurePursuitPlanner
{
public:
    PurePursuitPlanner(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~PurePursuitPlanner();
private:
    void plan();
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    void pathCallback(const usv_navigation_msgs::Path::ConstPtr msg);
    void poseTwistCallback(const geometry_msgs::PoseStamped::ConstPtr pose,const geometry_msgs::TwistStamped::ConstPtr twist);
    boost::optional<usv_navigation_msgs::Path> path_;
    boost::optional<geometry_msgs::PoseStamped> pose_;
    boost::optional<geometry_msgs::TwistStamped> twist_;
    ros::Subscriber path_sub_;
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped> > pose_sub_ptr_;
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::TwistStamped> > twist_sub_ptr_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_ptr_;
    std::string path_topic_;
    std::string twist_stamped_topic_;
    std::string current_pose_topic_;
    std::string robot_frame_;
    ros::Publisher twist_cmd_pub_;
    double getLookAheadDistance(double linear_velocity);
    bool checkLineAndCircleIntersection(double a_x, double a_y, double b_x, double b_y , double p_x, double p_y, float radius,double& x,double& y);
    pure_pursuit_planner::PurePursuitPlannerConfig config_;
    void dynaparamCallback(pure_pursuit_planner::PurePursuitPlannerConfig &config, uint32_t level);
    dynamic_reconfigure::Server<pure_pursuit_planner::PurePursuitPlannerConfig> dynaparam_server_;
    dynamic_reconfigure::Server<pure_pursuit_planner::PurePursuitPlannerConfig>::CallbackType dynaparam_callback_func_;
};

#endif  //PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_H_INCLUDED