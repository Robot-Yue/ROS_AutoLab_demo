#include <ros/ros.h>
#include <plumbing_server_client/AddInts.h>

int main(int argc, char *argv[]) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "heiShui");
    ros::NodeHandle nh;
    // 创建一个客户端对象
    ros::ServiceClient client = nh.serviceClient<plumbing_server_client::AddInts>("addInts");
    
    // 提交请求并处理响应
    plumbing_server_client::AddInts ai;
        // 组织请求
    ai.request.num1 = 100;
    ai.request.num2 = 200;
    
    // 如果客户端先启动则挂起（而不是报错），等待服务器启动后再正常请求
    // 这是一个阻塞式函数，只有服务启动成功后才会继续执行
    ros::service::waitForService("addInts");
        // 处理响应
    bool flag = client.call(ai);
    if(flag) {
        ROS_INFO("请求正常处理,响应结果 = %d", ai.response.sum);
    } else {
        ROS_ERROR("请求处理失败....");
        return 1;
    }

    return 0;
}
