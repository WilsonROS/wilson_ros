#include "Controller.hpp"
using namespace std;
Controller::Controller(string pr)
{
    prefix = pr;
    pub=nh.advertise<geometry_msgs::Twist>(prefix+"/mobile_base/commands/velocity"/*"/cmd_vel"/**/, 10);
}

void Controller::SetLinearSpeed(double x,double y,double z){
    msg.linear.x = x;
    msg.linear.y = y;
    msg.linear.z = z;
    ROS_INFO("Linear Velocity:  [%.2f,%.2f,%.2f]",x,y,z);
}

void Controller::SetAngularSpeed(double roll,double pitch,double yaw){
    msg.angular.x = roll;
    msg.angular.y = pitch;
    msg.angular.z = yaw;
    ROS_INFO("Angular Velocity: [%.2f,%.2f,%.2f]",roll,pitch,yaw);
}

void Controller::SendMsg(){
    pub.publish(msg);
}
