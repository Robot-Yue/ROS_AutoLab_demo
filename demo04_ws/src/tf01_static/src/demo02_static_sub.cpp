#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //调用 transform 必须包含该头文件

int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "static_sub");
    ros::NodeHandle nh;
    
    // 创建一个 buffer 缓存
    tf2_ros::Buffer buffer;
    // 创建监听对象（监听对象可以将订阅的数据存入 buffer）
    tf2_ros::TransformListener listener(buffer);
    // 组织一个坐标点数据
    geometry_msgs::PointStamped ps;
    ps.header.frame_id = "laser";
    ps.header.stamp = ros::Time::now();
    ps.point.x = 2.0;
    ps.point.y = 3.0;
    ps.point.z = 5.0;
    
    // 转换算法，需要调用 TF 内置实现
    //ros::Duration(2).sleep(); // 添加休眠
    ros::Rate rate(10);
    while (ros::ok()) {
        try {
            // 将 ps 转换成相对于 base_link 的坐标点
            geometry_msgs::PointStamped ps_out;

            /*
                调用了 buffer 的转换函数 transform
                参数1: 被转换的坐标点
                参数2: 目标坐标系
                返回值: 输出的坐标点
                
                PS1：调用时必须包含头文件 tf2_geometry_msgs/tf2_geometry_msgs.h
                PS2：运行时存在的问题：抛出一个异常 base link 不存在
                    原因：订阅数据是一个耗时操作，可能在调用 transform 转换函数时，坐标系的相对关系还没有订阅到，因此出现异常
                    解决：
                        方案1: 在调用转换函数前，执行休眠
                        方案2: 进行异常处理（建议使用该方案）
            */
            ps_out = buffer.transform(ps, "base_link");
            // 最后输出转换后的坐标点
            ROS_INFO("转换后的坐标:(%.2f,%.2f,%.2f), 参考的坐标系: %s",
                        ps_out.point.x,
                        ps_out.point.y,
                        ps_out.point.z,
                        ps_out.header.frame_id.c_str());
        }
        catch(const std::exception& e) {
            std::cerr << e.what() << '\n';
        }
        
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
