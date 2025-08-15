#include "rclcpp/rclcpp.hpp"
#include "aim_interfaces/msg/aim_info.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sstream"

class ImageTopicSubscribe : public rclcpp::Node{

public:
    ImageTopicSubscribe(std::string name) : Node(name){
        RCLCPP_INFO(this->get_logger(),"节点已经启动:%s",name.c_str());
        aim_target =this->create_subscription<aim_interfaces::msg::AimInfo>("aim",10,std::bind(&ImageTopicSubscribe::aim_callback,this,std::placeholders::_1));
    }
    
private:
    rclcpp::Subscription<aim_interfaces::msg::AimInfo>::SharedPtr aim_target;
    void aim_callback(const aim_interfaces::msg::AimInfo::SharedPtr msg){
        std::ostringstream oss;
        oss << "装甲板坐标：[";
        for(size_t i = 0; i < msg->coordinate.size(); ++i){
            oss << msg->coordinate[i] << (i + 1 == msg->coordinate.size() ? "" : ", ");
        }
        oss << "],装甲板图案:"<< msg->type;
        RCLCPP_INFO(this->get_logger(),"%s",oss.str().c_str());
    }
};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ImageTopicSubscribe>("Image_Topic_Subscribe");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}