#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class ImageTopicPublisher : public rclcpp::Node{

public:
    ImageTopicPublisher(std::string name) : Node(name){
        RCLCPP_INFO(this->get_logger(),"节点已经启动:%s",name.c_str());
        sensor_img = this->create_publisher<sensor_msgs::msg::Image>("sensor",10);
        time = this->create_wall_timer(std::chrono::milliseconds(500),std::bind(&ImageTopicPublisher::timer_callback,this));
    }
private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr sensor_img;
    rclcpp::TimerBase::SharedPtr time;
    void timer_callback(){

    }
};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ImageTopicPublisher>("Image_Topic_Publisher");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}