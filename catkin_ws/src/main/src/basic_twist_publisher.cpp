#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"


//joy_msg.axes[] joy_msg.buttons[] reference
//# joystick input
//KEY_SQUARE = 0
//KEY_CROSS = 1
//KEY_CIRCLE = 2
//KEY_TRIANGLE = 3
//KEY_L1 = 4
//KEY_R1 = 5
//KEY_L2 = 6
//KEY_R2 = 7
//KEY_SELECT = 8
//KEY_START = 9
//KEY_L3 = 10
//KEY_R3 = 11
//KEY_PS = 12
//AXIS_LX = 0
//AXIS_LY = 1
//AXIS_RX = 2
//AXIS_RY = 3
//AXIS_CURSORX = 4
//AXIS_CURSORY = 5

geometry_msgs::Twist cmd_vel;
void joy_callback(const sensor_msgs::Joy& joy_msg){
    cmd_vel.linear.x = joy_msg.axes[1];
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = joy_msg.axes[2];
}

int main(int argc, char **argv){
    ros::init(argc, argv, "basic_twist_publisher");
    ros::NodeHandle n;

    //publish
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    //subscriibe
    ros::Subscriber joy_sub = n.subscribe("joy", 10, joy_callback);

    ros::Rate loop_rate(10);
    while (ros::ok()){
        cmd_pub.publish(cmd_vel);
    	ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
