#include <ros/ros.h>
#include "Controller.hpp"
#include "KeyHandler.hpp"

/**
  @file main.cpp

  @brief Short description of your code (one sentence)

  Complete description of your code.

  @param /parameter (optional)
  @param /parameter (optional)

  @autor Student
  @date dd/mm/yy
 */

///Main loop
int main(int argc, char **argv) {
    //Initializes ROS, and sets up a node
    // The name is random in order to not hit other student's node.
    ros::init(argc, argv, "jrg");
    Controller robot("");

    //Sets the loop to publish at a rate of 10Hz
    ros::Rate loop_rate(10);

    KeyHandler kh(400,400);
    // Sets the name shown in the titlebar and the taskbar
    kh.setWindowTitle("WilsonROS Controller");
    float speed = 1.0;
    while(ros::ok()) {
        bool dir = false;
        bool lin = false;
        kh.process();
        /*if(kh.isHeld('1')) speed = 1;
        if(kh.isHeld('2')) speed = 2;
        if(kh.isHeld('3')) speed = 3;
        if(kh.isHeld('4')) speed = 4;*/
        if (kh.isHeld('w') || kh.isHeld(KEY_Up)){
            robot.SetLinearSpeed(0.05 * speed, 0, 0);
            lin = true;
        }
        if (kh.isHeld('a') || kh.isHeld(KEY_Left)) {
            robot.SetAngularSpeed(0,0, speed * (0.33));
            dir = true;
        }
        if (kh.isHeld('s') || kh.isHeld(KEY_Down)) {
            robot.SetLinearSpeed((-0.05) * speed, 0, 0);
            lin = true;
        }
        if (kh.isHeld('d') || kh.isHeld(KEY_Right)) {
            robot.SetAngularSpeed(0,0, speed * (-0.33));
            dir = true;
        }
        if(!dir) robot.SetAngularSpeed(0, 0, 0);
        if(!lin) robot.SetLinearSpeed(0, 0, 0);

        //Publish the message
        robot.SendMsg();

        //Delays untill it is time to send another message
        loop_rate.sleep();
    }
    return 0;
}
