#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/msg/image.hpp"
#include "aim_interfaces/msg/aim_info.hpp"
using namespace cv;

class Shift : public rclcpp::Node{
public:
    Shift(std::string name):Node(name){
        RCLCPP_INFO(this->get_logger(),"%s节点已经启动。",name.c_str());
        shift_subscribe = this->create_subscription<sensor_msgs::msg::Image>("sensor",10,std::bind(&Shift::sensor_callback,this,std::placeholders::_1));
        shift_publisher = this->create_publisher<aim_interfaces::msg::AimInfo>("array",10);
        shift_timer = this->create_wall_timer(std::chrono::milliseconds(500),std::bind(&Shift::aim_timer_callback,this));
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr shift_subscribe;
    rclcpp::Publisher<aim_interfaces::msg::AimInfo>::SharedPtr shift_publisher;
    rclcpp::TimerBase::SharedPtr shift_timer;
    void sensor_callback(const sensor_msgs::msg::Image::SharedPtr ros_img){
        Mat cv_img_in = cv_bridge::toCvCopy(ros_img, "bgr8")->image;
    }

    void aim_timer_callback(){

    }

};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<Shift>("Shift");
    rclcpp::spin(node);
    rclcpp::shutdown();
}