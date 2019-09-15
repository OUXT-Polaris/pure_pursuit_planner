// Headers in this package
#include <pure_pursuit_planner/pure_pursuit_planner.h>

// Headers in Eigen
#include <Eigen/Core>

PurePursuitPlanner::PurePursuitPlanner(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<std::string>("twist_stamped_topic", twist_stamped_topic_, nh_.getNamespace()+"/twist_stamped");
    pnh_.param<std::string>("path_topic", path_topic_, nh_.getNamespace()+"/path");
    pnh_.param<std::string>("current_pose_topic", current_pose_topic_, nh_.getNamespace()+"/current_pose_topic");
    twist_cmd_pub_ = pnh_.advertise<geometry_msgs::TwistStamped>("twist_cmd",1);
    twist_sub_ptr_ = std::make_shared<message_filters::Subscriber<geometry_msgs::TwistStamped> >(nh_,twist_stamped_topic_,10);
    pose_sub_ptr_ = std::make_shared<message_filters::Subscriber<geometry_msgs::PoseStamped> >(nh_,current_pose_topic_,10);
    sync_ptr_ = std::make_shared<message_filters::Synchronizer<SyncPolicy> >(SyncPolicy(10), *pose_sub_ptr_, *twist_sub_ptr_ );
    sync_ptr_->registerCallback(boost::bind(&PurePursuitPlanner::poseTwistCallback, this, _1, _2));
    path_sub_ = nh_.subscribe(path_topic_,1,&PurePursuitPlanner::pathCallback,this);
    dynaparam_callback_func_ = boost::bind(&PurePursuitPlanner::dynaparamCallback, this, _1, _2);
    dynaparam_server_.setCallback(dynaparam_callback_func_);
}

PurePursuitPlanner::~PurePursuitPlanner()
{

}

void PurePursuitPlanner::pathCallback(const usv_navigation_msgs::Path::ConstPtr msg)
{
    path_ = *msg;
    return;
}

void PurePursuitPlanner::poseTwistCallback(const geometry_msgs::PoseStamped::ConstPtr pose,const geometry_msgs::TwistStamped::ConstPtr twist)
{
    pose_ = *pose;
    twist_ = *twist;
    return;
}

void PurePursuitPlanner::plan()
{
    if(path_ && pose_ && twist_)
    {
        double look_ahead_distance = getLookAheadDistance(twist_->twist.linear.x);
        double yaw_angle = quaternion_operation::convertQuaternionToEulerAngle(pose_->pose.orientation).z;
        int target_waypoint_index = path_->waypoints.size();
        for(int i=0; i<path_->waypoints.size()-1; i++)
        {
            double x,y;
            bool crossed = checkLineAndCircleIntersection(path_->waypoints[i].pose.position.x,path_->waypoints[i].pose.position.y,
                path_->waypoints[i+1].pose.position.x,path_->waypoints[i+1].pose.position.y,pose_->pose.position.x,pose_->pose.position.y,look_ahead_distance,x,y);
            if(crossed)
            {
                target_waypoint_index = i;
                break;
            }
        }
    }
    return;
}

void PurePursuitPlanner::dynaparamCallback(pure_pursuit_planner::PurePursuitPlannerConfig &config, uint32_t level)
{
    config_ = config;
    return;
}

double PurePursuitPlanner::getLookAheadDistance(double linear_velocity)
{
    double ret = config_.lookahead_ratio*linear_velocity;
    if(std::fabs(ret) < config_.mimimum_lookahead_distance)
    {
        if(ret<0.0)
        {
            ret = config_.mimimum_lookahead_distance*-1;
        }
        else
        {
            ret = config_.mimimum_lookahead_distance;
        }
    }
    return ret;
}

bool PurePursuitPlanner::checkLineAndCircleIntersection(double a_x, double a_y, double b_x, double b_y , double p_x, double p_y, float radius, double& x, double& y)
{
    Eigen::Vector2d a,b,p,ap,bp,ab,normal_ab,normal_ap;
    a << a_x,a_y;
    b << b_x,b_y;
    p << p_x,p_y;
    ap = p-a;
    ab = b-a;
    bp = p-b;
    normal_ab = ab.normalized();
    double len_ax = normal_ab.dot(ap);
    double shortest_distance = 0.0;
    if ( len_ax < 0 )
    {
        shortest_distance = ap.norm();
    }
    else if( len_ax > ab.norm() )
    {
        shortest_distance = bp.norm();
    }
    else
    {
        shortest_distance = std::fabs(a_x*b_y-a_y*b_x);
    }
    x = a[0] + normal_ab[0]*len_ax;
    y = a[1] + normal_ab[1]*len_ax;
    return shortest_distance < radius;
}