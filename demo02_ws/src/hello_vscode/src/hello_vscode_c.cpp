#include "ros/ros.h"

int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "HelloVSCode");

    ROS_INFO("Hello VSCode!!!");

    return 0;
}
