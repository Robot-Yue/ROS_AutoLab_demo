#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

void doPose(const turtlesim::Pose::ConstPtr& pose) {
    // 获取位姿信息，转换成坐标系相对关系并发布
    //     创建发布对象
    static tf2_ros::TransformBroadcaster pub;
    //     组织被发布的数据
    geometry_msgs::TransformStamped ts;
    ts.header.frame_id = "world";
    ts.header.stamp = ros::Time::now();
    ts.child_frame_id = "turtle1";

    // 坐标系偏移量设置
    ts.transform.translation.x = pose->x;
    ts.transform.translation.y = pose->y;
    ts.transform.translation.z = 0.0; // 二维实现，pose 中没有z，z 是 0
    // 坐标系四元数：位姿信息中没有四元数，但是有个偏航角度，又已知乌龟是 2D ，没有翻滚与俯仰角度
    // 所以可以得出乌龟的欧拉角：0 0 theta 
    tf2::Quaternion qtn;
    qtn.setRPY(0, 0, pose->theta);

    ts.transform.rotation.x = qtn.getX();
    ts.transform.rotation.y = qtn.getY();
    ts.transform.rotation.z = qtn.getZ();
    ts.transform.rotation.w = qtn.getW();

    // 发布数据
    pub.sendTransform(ts);
}

int main(int argc, char *argv[]) {
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"dynamic_pub");
    ros::NodeHandle nh;
    // 创建订阅对象，订阅 /turtle1/pose
    ros::Subscriber sub = nh.subscribe<turtlesim::Pose>("/turtle1/pose", 100, doPose);
    
    // 回调函数 doPose 处理订阅的信息：将位姿信息转换成坐标相对关系并发布     
    ros::spin();
    return 0;
}
