#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "erGouZi");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::String>("fang", 10);

    std_msgs::String msg;
    int count = 0;

    ros::Rate rate(10);

    ros::Duration(3).sleep();

    while (ros::ok()) {
        std::stringstream ss;
        ss << "hello ---> " << count;
        msg.data = ss.str();

        pub.publish(msg);
        ROS_INFO("message : %s", msg.data.c_str());

        rate.sleep();
        count++;

        ros::spinOnce();
    }

    return 0;
}
