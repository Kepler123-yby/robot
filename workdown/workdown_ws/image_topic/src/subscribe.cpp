#include "rclcpp/rclcpp.hpp"

class ImageTopicSubscribe : public rclcpp::Node{

public:
    
private:

};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ImageTopicSubscribe>("Image_Topiic_Subscribe");
    rclcpp::spin(node);
    rclcpp::shutdown;
    return 0;
}