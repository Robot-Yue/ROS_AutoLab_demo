#include "ros/ros.h"
#include "plumbing_pub_sub/Person.h"

void doPerson(const plumbing_pub_sub::Person::ConstPtr& person) {
    ROS_INFO("xinxi sub : %s, %d, %.2f", person->name.c_str(), person->age, person->height);
}

int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "jiaZhang");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<plumbing_pub_sub::Person>("liaoTian", 10, doPerson);

    ros::spin();

    return 0;
}
