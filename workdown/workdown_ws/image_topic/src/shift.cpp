#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/msg/image.hpp"
#include "aim_interfaces/msg/aim_info.hpp"

class Shift : public rclcpp::Node{
public:
    Shift(std::string name):Node(name){
        RCLCPP_INFO(this->get_logger(),"%s节点已经启动。",name.c_str());
    }
private:

};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<Shift>("Shift");
    rclcpp::spin(node);
    rclcpp::shutdown();
}