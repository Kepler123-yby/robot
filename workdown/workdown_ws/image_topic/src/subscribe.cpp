#include "rclcpp/rclcpp.hpp"

class ImageTopicSubscribe : public rclcpp::Node{

public:
    ImageTopicSubscribe(std::string name) : Node(name){
        RCLCPP_INFO(this->get_logger(),"节点已经启动:%s",name.c_str());
    }
    
private:

};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ImageTopicSubscribe>("Image_Topic_Subscribe");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}