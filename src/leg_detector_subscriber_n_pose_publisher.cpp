#include"ros/ros.h"
#include"geometry_msgs/Pose.h"
#include"geometry_msgs/Point.h"
#include"geometry_msgs/PoseWithCovarianceStamped.h"
#include"sensor_msgs/LaserScan.h"
#include"people_msgs/PositionMeasurementArray.h"
#include"people_msgs/PositionMeasurement.h"
#include <move_base_msgs/MoveBaseAction.h>
#include"std_msgs/Float64.h"
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Float64MultiArray.h"
#include<math.h>
#include<iostream>
using namespace std;


people_msgs::PositionMeasurement legs_published;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

geometry_msgs::Pose bot_pos;
double heading_angle=0;
bool tracking=false;

void leg_detector_callback( const people_msgs::PositionMeasurementArray::ConstPtr & legs)
{
    
    double max;
    int length = legs->people.size();
    cout <<length <<endl;
    if(length!=0)
    {
        max=legs->people[0].reliability;
        legs_published=legs->people[0];

        for(int i=1;i<length;++i)
        {
            if(legs->people[i].reliability>max)
            {
                max=legs->people[i].reliability;
                legs_published=legs->people[i];
            }
        }
        tracking=true;
    }

    else 
    {
        tracking=false;  
    }


    
}

void headercallback(const std_msgs::Float64MultiArray::ConstPtr & msg)
{
  heading_angle=(msg->data[1]+msg->data[0])/2;
}


void robot_position_callback( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pos)
{
    bot_pos=pos->pose.pose;
}

int main (int argc, char ** argv)
{

    ros::init(argc,argv,"pose_publisher");
    ros::NodeHandle n;
    ros::Subscriber sub_1=n.subscribe("people_tracker_measurements",1,leg_detector_callback);
    ros::Subscriber sub_2=n.subscribe("tracker_angles",1,headercallback);
    ros::Subscriber sub_3=n.subscribe("amcl_pose",1,robot_position_callback);

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("/move_base", true);
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
    }
    
    while(ros::ok())
    {
        ros::Duration(2).sleep();
        if(tracking)
        {
        double x_arr=(legs_published.pos.x-bot_pos.position.x)*0.7;
        double y_arr=(legs_published.pos.y-bot_pos.position.y)*0.7;

        double x_final=bot_pos.position.x+x_arr;
        double y_final=bot_pos.position.y+y_arr;


        cout <<"x"<< x_final <<endl;
        cout <<"y"<< y_final <<endl;
        move_base_msgs::MoveBaseGoal goal;

        
        //we'll send a goal to the robot to move 1 meter forward
        goal.target_pose.header.frame_id = "base_link";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = x_final;
        goal.target_pose.pose.position.y = y_final;
        goal.target_pose.pose.orientation.z = sin(heading_angle/2);
        goal.target_pose.pose.orientation.w = cos(heading_angle/2);

        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("tracking");
        else
        ROS_INFO("no traget");
        }
        ros::spinOnce();
    }
    
   


}
