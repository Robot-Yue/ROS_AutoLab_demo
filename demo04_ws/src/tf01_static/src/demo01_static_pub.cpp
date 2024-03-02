#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "static_pub");
    ros::NodeHandle nh;

    // 创建发布对象（静态坐标转换广播器）
    tf2_ros::StaticTransformBroadcaster pub;
    // 组织被发布的消息（创建坐标系信息）
    geometry_msgs::TransformStamped tfs;
    
    // 设置头信息
    tfs.header.seq = 100;
    tfs.header.stamp = ros::Time::now();
    tfs.header.frame_id = "base_link"; // 相对坐标系关系中被参考的那一个
    // 设置子级坐标系
    tfs.child_frame_id = "laser";
    // 设置子级相对于父级的偏移量
    tfs.transform.translation.x = 0.2;
    tfs.transform.translation.y = 0.0;
    tfs.transform.translation.z = 0.5;
    
    // 设置四元数：将欧拉角数据转换成四元数
    tf2::Quaternion qtn;
    // 向该对象设置欧拉角，这个对象可以将欧拉角转换成四元数
    qtn.setRPY(0, 0, 0); // 欧拉角的单位是弧度
    tfs.transform.rotation.x = qtn.getX();
    tfs.transform.rotation.y = qtn.getY();
    tfs.transform.rotation.z = qtn.getZ();
    tfs.transform.rotation.w = qtn.getW();

    // 广播器发布坐标系信息
    pub.sendTransform(tfs);

    ros::spin();
    return 0;
}
