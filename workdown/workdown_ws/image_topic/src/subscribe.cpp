#include "rclcpp/rclcpp.hpp"
#include "aim_interfaces/msg/aim_info.hpp"

class ImageTopicSubscribe : public rclcpp::Node{

public:
    ImageTopicSubscribe(std::string name) : Node(name){
        RCLCPP_INFO(this->get_logger(),"节点已经启动:%s",name.c_str());
        aim_target =this->create_subscription<aim_interfaces::msg::AimInfo>("aim",10,std::bind(&ImageTopicSubscribe::aim_callback,this,std::placeholders::_1));
    }
    
private:
    rclcpp::Subscription<aim_interfaces::msg::AimInfo>::SharedPtr aim_target;
    void aim_callback(){

    }
};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ImageTopicSubscribe>("Image_Topic_Subscribe");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}