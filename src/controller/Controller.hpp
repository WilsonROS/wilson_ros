#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Twist.h>

using namespace std;
/**
  Description of the class
 */
class Controller
{
    public:
        // Constructor
            /**
             * @brief Short description of your code (one sentence)
             *
             * Complete description of your code.
             *
             * @note (optional) notes of the code
             * @param /parameter (optional)
             * @param /parameter (optional)
             *
             * @return (optional) return value of this function
             */
            Controller(string pr);

        //Public methods
            /**
             * @brief Short description of your code (one sentence)
             *
             * Complete description of your code.
             *
             * @note (optional) notes of the code
             * @param /parameter (optional)
             * @param /parameter (optional)
             *
             * @return (optional) return value of this function
             */
            void SetLinearSpeed(double x=0,double y=0,double z=0);

            /**
             * @brief Short description of your code (one sentence)
             *
             * Complete description of your code.
             *
             * @note (optional) notes of the code
             * @param /parameter (optional)
             * @param /parameter (optional)
             *
             * @return (optional) return value of this function
             */
            void SetAngularSpeed(double roll=0,double pitch=0,double yaw=0);

             /**
             * @brief Short description of your code (one sentence)
             *
             * Complete description of your code.
             *
             * @note (optional) notes of the code
             * @param /parameter (optional)
             * @param /parameter (optional)
             *
             * @return (optional) return value of this function
             */
        void SendMsg();

    private:
        // ROS private variables
        ros::NodeHandle nh;
        ros::Publisher pub;
        geometry_msgs::Twist msg;
        string prefix;
        // Private variables
        //...//

        // Private methods
        //...//
};

#endif // CONTROLLER_H
