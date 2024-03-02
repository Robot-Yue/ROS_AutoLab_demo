#include "ros/ros.h"
#include "plumbing_pub_sub/Person.h"

int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "banZhuRen");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<plumbing_pub_sub::Person>("liaoTian", 10);

    plumbing_pub_sub::Person person;
    person.name = "zhangsan";
    person.age = 1;
    person.height = 1.70;

    ros::Rate rate(1);

    while (ros::ok()) {
        person.age += 1;
        pub.publish(person);
        ROS_INFO("message pub: %s, %d, %.2f", person.name.c_str(), person.age, person.height);

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
