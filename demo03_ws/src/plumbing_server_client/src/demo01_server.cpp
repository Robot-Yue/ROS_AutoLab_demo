#include <ros/ros.h>
#include <plumbing_server_client/AddInts.h>

// bool 返回值由于标志是否处理成功
bool doNums(plumbing_server_client::AddInts::Request &request,
            plumbing_server_client::AddInts::Response &response) {
    int num1 = request.num1;
    int num2 = request.num2;

    ROS_INFO("服务器接收到的请求数据为:num1 = %d, num2 = %d",num1, num2);

    // 逻辑处理
    if(num1 < 0 || num2 < 0) {
        ROS_ERROR("提交的数据异常:数据不可以为负数");
        return false;
    }

    // 如果没有异常，那么相加并将结果赋值给 response
    response.sum = num1 + num2;            
    
    return true;
}

int main(int argc, char *argv[]) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "heiShui");
    ros::NodeHandle nh;
    ros::ServiceServer server = nh.advertiseService("addInts", doNums);

    ROS_INFO("服务已经启动....");
    ros::spin(); // 由于请求有多个，需要调用 ros::spin();

    return 0;
}
