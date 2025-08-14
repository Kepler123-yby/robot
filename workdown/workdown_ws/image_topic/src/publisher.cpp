#include "rclcpp/rclcpp.hpp"

class ImageTopicPublisher : public rclcpp::Node{

public:
    
private:

};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ImageTopicPublisher>("Image_Topiic_Publisher");
    rclcpp::spin(node);
    rclcpp::shutdown;
    return 0;
}