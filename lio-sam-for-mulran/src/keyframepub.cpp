#include "utility.h"
#include "lio_sam/key_frame.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Empty.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"

lio_sam::key_frame keyframe_;
geometry_msgs::Point return_goal_tmp_;
geometry_msgs::Point cur_pose_;
geometry_msgs::Quaternion cur_ori_;
geometry_msgs::PoseStamped returnGoal_;
std::vector<geometry_msgs::Point> key_vec_;
vector<geometry_msgs::Point> a;
ros::Publisher pubKeyFrame;
ros::Publisher pubReturnGoal;
ros::Publisher pubLoopConstraintEdge;
ros::Publisher pubReGoalVisualize;

bool is_return = false;

float interval = 3.0f;
float goal_update = 0.5f;
int return_flag = 0;


void visualizeKeyFrame()
{
    if (key_vec_.empty())
        return;
    
    
    // For visualize Key Frame Extraction
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker markerNode;
    markerNode.header.frame_id = "odom";
    //markerNode.header.stamp = timeLaserInfoStamp;
    markerNode.action = visualization_msgs::Marker::ADD;
    markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
    markerNode.ns = "Key_Frame_Stack";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.5; markerNode.scale.y = 0.5; markerNode.scale.z = 0.5; 
    markerNode.color.r = 0.05; markerNode.color.g = 0.9; markerNode.color.b = 0.05;
    markerNode.color.a = 1;

    for (int it = 0; it < key_vec_.size(); ++it)
    {
        geometry_msgs::Point p;
        p.x = key_vec_[it].x;
        p.y = key_vec_[it].y;
        p.z = key_vec_[it].z;
        markerNode.points.push_back(p);
    }

    markerArray.markers.push_back(markerNode);
    pubLoopConstraintEdge.publish(markerArray);
}



float calc_point_dist(geometry_msgs::Point a, geometry_msgs::Point b){
    return sqrtf(powf(a.x - b.x,2) + powf(a.y - b.y,2) + powf(a.z - b.z,2));
}

void Callback(const visualization_msgs::MarkerArray::ConstPtr &msg){
    a = msg->markers[0].points;
    
    if(!is_return){
        key_vec_.clear();
        key_vec_.shrink_to_fit();
        for (int i=0; i<a.size(); i=i+2)
        {
            if (!key_vec_.empty()){
                int size = key_vec_.size() - 1;
                float dist = calc_point_dist(key_vec_[size], a[i]);
                if (dist < interval){
                    //  skip save
                }
                else{
                    key_vec_.push_back(a[i]);
                }
            }
            else{
                key_vec_.push_back(a[i]);
            }
        }
        keyframe_.key_frame = key_vec_;
    }
    pubKeyFrame.publish(keyframe_);
    visualizeKeyFrame();
}

void getOrientation(){  // Use Only Yaw Value to Calculate Orientation of Retun Goal Point
    double dx = return_goal_tmp_.x - cur_pose_.x;
    double dy = return_goal_tmp_.y - cur_pose_.y;
    double target_deg = atan2(dy, dx) * 180.0 / M_PI;

    tf::Quaternion q(
        cur_ori_.x,
        cur_ori_.y,
        cur_ori_.z,
        cur_ori_.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double theta_deg = target_deg - yaw;
    if (theta_deg > 180.0) theta_deg -= 360.0;
    else if (theta_deg < -180.0) theta_deg += 360.0;
    double theta = theta_deg * M_PI / 180.0;
    std::cout << "theta is ::  " << theta_deg << std::endl;

    returnGoal_.pose.orientation.x = 0.0;
    returnGoal_.pose.orientation.y = 0.0;
    returnGoal_.pose.orientation.z = sin(theta / 2);
    returnGoal_.pose.orientation.w = cos(theta / 2);
}

void ReturnCallback(const std_msgs::Empty::ConstPtr &msg){
    is_return = true;
    return_flag++;
    if ((return_flag % 2) == 0) is_return = false;

    return_goal_tmp_ = key_vec_[key_vec_.size()-1];
    float dist = calc_point_dist(cur_pose_, return_goal_tmp_);
    if(dist < goal_update){
        key_vec_.pop_back();
        return_goal_tmp_ = key_vec_[key_vec_.size()-1];
    }
    returnGoal_.pose.position = return_goal_tmp_;
    getOrientation();
    pubReturnGoal.publish(returnGoal_);

    // For visualize Return Goal Point
    visualization_msgs::Marker markerGoal;
    markerGoal.header.frame_id = "odom";
    //markerGoal.header.stamp = timeLaserInfoStamp;
    markerGoal.action = visualization_msgs::Marker::ADD;
    markerGoal.type = visualization_msgs::Marker::SPHERE_LIST;
    markerGoal.ns = "Return_Goal_Point";
    markerGoal.id = 1;
    markerGoal.pose.orientation.w = 1;
    if (is_return){
        markerGoal.scale.x = 0.8; markerGoal.scale.y = 0.8; markerGoal.scale.z = 0.8;
    }
    else{
        markerGoal.scale.x = 0.0; markerGoal.scale.y = 0.0; markerGoal.scale.z = 0.0;
    }
    markerGoal.color.r = 1.0; markerGoal.color.g = 0.0; markerGoal.color.b = 0.0;
    markerGoal.color.a = 1;
    markerGoal.points.push_back(return_goal_tmp_);
    pubReGoalVisualize.publish(markerGoal);
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg){
    cur_pose_ = msg->pose.pose.position;
    cur_ori_ = msg->pose.pose.orientation;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "keyframepub");
	ros::NodeHandle nh;
    pubKeyFrame = nh.advertise<lio_sam::key_frame>("key_frame", 1);
    pubReturnGoal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>("/key_frame_stack", 1);
    pubReGoalVisualize = nh.advertise<visualization_msgs::Marker>("/Return_Goal_Point", 1);
    nh.getParam("/lio_sam/interval", interval);
    nh.getParam("/lio_sam/goal_update", goal_update);
	ros::Subscriber sub = nh.subscribe("/lio_sam/mapping/loop_closure_constraints",1,Callback);
    ros::Subscriber subOdom = nh.subscribe("/lio_sam/mapping/odometry",1,OdomCallback);
    ros::Subscriber subReturn = nh.subscribe("/return",1,ReturnCallback);
	while (ros::ok()){
		ros::spinOnce();
	}
	return 0;
}