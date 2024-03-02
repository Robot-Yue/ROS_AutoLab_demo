#include "ros/ros.h"
#include "std_msgs/String.h"

void doMsg(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("cuiHua sub : %s", msg->data.c_str());
}

int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "cuiHua");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<std_msgs::String>("fang", 10, doMsg);

    ros::spin();

    return 0;
}
