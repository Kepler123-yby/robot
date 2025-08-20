#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
using namespace cv;

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
        Mat image = imread("/home/vboxuser/robot/workdown/workdown_ws/picture/3.png",IMREAD_COLOR);
        if (image.empty()){
            RCLCPP_WARN(this->get_logger(), "图片读取失败，请检查路径！");
            return;
    }
        auto picture = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        sensor_img->publish(*picture);
        RCLCPP_DEBUG(this->get_logger(), "已发布图片消息");
    }
};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ImageTopicPublisher>("Image_Topic_Publisher");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}